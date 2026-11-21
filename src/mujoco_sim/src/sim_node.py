#!/usr/bin/env python3
"""MuJoCo 仿真节点（物理就在本进程跑，键盘/恢复控制它）

修复了之前"被动回放"的问题：
  ✅ 物理在本进程跑（mj_step）→ 碰撞真实生效、桨叶真转
  ✅ 电机模型施加推力/反扭矩（和最初 view.py 一致）
  ✅ 键盘 + 恢复节点发期望姿态/推力，本节点转成电机转速
  ✅ MuJoCo viewer 交互窗口（可拖 Joint/Control 面板、可以自己加 Reset）

话题：
  订阅 /fmu/in/vehicle_attitude_setpoint_v1 (px4_msgs/VehicleAttitudeSetpoint)
  发布 /fmu/out/vehicle_odometry            (px4_msgs/VehicleOdometry)
  发布 /mujoco/contacts                     (std_msgs/Float64MultiArray)

    ros2 run mujoco_sim sim_node [--xml ...] [--body x500]
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry, VehicleAttitudeSetpoint, VehicleStatus
from std_msgs.msg import Float64MultiArray

# ---------- 电机模型常量（与最初 view.py / motor_model.py 一致） ----------
KF = 8.54858e-06          # N/(rad/s)^2
KM = 1.71e-07             # N·m/(rad/s)^2
MAX_OMEGA = 1000.0        # rad/s @ full throttle
ROTOR_POS = np.array([[0.174, -0.174, 0.06],
                      [-0.174, 0.174, 0.06],
                      [0.174, 0.174, 0.06],
                      [-0.174, -0.174, 0.06]])
# 反扭矩方向（已修正：和 PX4 混控器 yaw 一致，避免自旋）
DIRS = np.array([-1.0, -1.0, +1.0, +1.0])

# 混控矩阵：  [F_z, T_x, T_y, T_z]^T = M @ [f0,f1,f2,f3]^T
M_MIX = np.array([
    [1.0, 1.0, 1.0, 1.0],
    ROTOR_POS[:, 1],                    # T_x = sum(r_y * f)
    -ROTOR_POS[:, 0],                   # T_y = -sum(r_x * f)
    (KM / KF) * DIRS,                   # T_z
])
M_INV = np.linalg.inv(M_MIX)

MAX_THRUST = 4.0 * KF * MAX_OMEGA * MAX_OMEGA   # 34.19 N


def q_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def q_conj(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])


def q_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw*bw - ax*bx - ay*by - az*bz,
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
    ])


def q_norm(q):
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def ned_q_to_flu_q(q_ned_frd):
    """NED/FRD 姿态 -> ENU/FLU 姿态 = Cne * R * Cff"""
    Cne = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
    Cff = np.diag([1.0, -1.0, -1.0])
    R = Cne @ q_to_mat(q_ned_frd) @ Cff
    return R


def apply_motor_model(m, d, body_id):
    """电机模型：按 d->ctrl(转速) 给机体质心加推力+反扭矩"""
    omega = np.asarray(d.ctrl[:4], dtype=float)
    R = d.xmat[body_id].reshape(3, 3)
    axis = R[:, 2]
    F = np.zeros(3)
    T = np.zeros(3)
    for i in range(4):
        f = KF * omega[i] ** 2
        force = axis * f
        F += force
        T += np.cross(R @ ROTOR_POS[i], force)
        T += axis * (KM * omega[i] ** 2 * DIRS[i])
    d.xfrc_applied[body_id, :3] = F
    d.xfrc_applied[body_id, 3:] = T


class SimNode(Node):
    def __init__(self, m, d, body):
        super().__init__("mujoco_sim")
        self.m = m
        self.d = d
        self.body_id = m.body(body).id
        self.free_adr = m.jnt_qposadr[m.body_jntadr[self.body_id]]

        self.sp_q = None      # 最新期望姿态(FLU, wxyz)
        self.sp_thrust = 0.0  # 期望推力(归一化 0..1)
        self.sp_stamp = 0.0
        self.armed = False

        self.odom_pub = self.create_publisher(VehicleOdometry, "/fmu/out/vehicle_odometry", 10)
        self.contact_pub = self.create_publisher(Float64MultiArray, "/mujoco/contacts", 10)
        self.status_pub = self.create_publisher(VehicleStatus, "/fmu/out/vehicle_status_v1", 10)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.sp_sub = self.create_subscription(
            VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint_v1",
            self.on_setpoint, qos)

        # 姿态控制器增益
        self.kp_att = 6.0
        self.kd_att = 0.35
        # 推力限幅
        self.thrust_max = MAX_THRUST
        self.t0 = time.time()
        self.step = 0

        # 发布/渲染频率
        self.odom_every = 1     # 每步都发(500Hz)
        self.contact_every = 10

    # ---------- 订阅 ----------
    def on_setpoint(self, msg):
        # q_d 在 NED/FRD 里，转到 FLU 用旋转矩阵
        q_ned = np.array([msg.q_d[0], msg.q_d[1], msg.q_d[2], msg.q_d[3]], float)
        if np.linalg.norm(q_ned) < 1e-6:
            return
        q_ned = q_norm(q_ned)
        R_flu = ned_q_to_flu_q(q_ned)
        # 转回四元数
        tr = np.trace(R_flu)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            w = 0.25 * S
            x = (R_flu[2, 1] - R_flu[1, 2]) / S
            y = (R_flu[0, 2] - R_flu[2, 0]) / S
            z = (R_flu[1, 0] - R_flu[0, 1]) / S
        elif R_flu[0, 0] > R_flu[1, 1] and R_flu[0, 0] > R_flu[2, 2]:
            S = math.sqrt(1.0 + R_flu[0, 0] - R_flu[1, 1] - R_flu[2, 2]) * 2
            w = (R_flu[2, 1] - R_flu[1, 2]) / S
            x = 0.25 * S
            y = (R_flu[0, 1] + R_flu[1, 0]) / S
            z = (R_flu[0, 2] + R_flu[2, 0]) / S
        elif R_flu[1, 1] > R_flu[2, 2]:
            S = math.sqrt(1.0 + R_flu[1, 1] - R_flu[0, 0] - R_flu[2, 2]) * 2
            w = (R_flu[0, 2] - R_flu[2, 0]) / S
            x = (R_flu[0, 1] + R_flu[1, 0]) / S
            y = 0.25 * S
            z = (R_flu[1, 2] + R_flu[2, 1]) / S
        else:
            S = math.sqrt(1.0 + R_flu[2, 2] - R_flu[0, 0] - R_flu[1, 1]) * 2
            w = (R_flu[1, 0] - R_flu[0, 1]) / S
            x = (R_flu[0, 2] + R_flu[2, 0]) / S
            y = (R_flu[1, 2] + R_flu[2, 1]) / S
            z = 0.25 * S
        self.sp_q = q_norm(np.array([w, x, y, z]))
        # thrust_body[2] 负数=向上，归一化 0..1
        self.sp_thrust = float(np.clip(-msg.thrust_body[2], 0.0, 1.0))
        self.sp_stamp = time.time()

    # ---------- 控制 ----------
    def control(self):
        """期望姿态+推力 -> 电机转速"""
        now = time.time()
        has_sp = (self.sp_q is not None) and (now - self.sp_stamp < 0.5)
        if not has_sp:
            # 没收到 setpoint：怠速不转
            self.d.ctrl[:] = 0.0
            self.armed = False
            return

        self.armed = True
        q_cur = self.d.qpos[self.free_adr + 3:self.free_adr + 7].copy()
        q_cur = q_norm(q_cur)
        q_des = self.sp_q

        # 姿态误差（机体系）： q_err = q_cur^-1 * q_des
        q_err = q_norm(q_mul(q_conj(q_cur), q_des))
        if q_err[0] < 0:
            q_err = -q_err
        err_vec = 2.0 * q_err[1:4]

        # 角速度（机体系）
        w_body = self.d.qvel[3:6].copy()

        # 期望力矩（机体系）
        tau = self.kp_att * err_vec - self.kd_att * w_body

        # 期望总推力（沿机体 z）
        T = self.sp_thrust * self.thrust_max

        # 混控： [T, tau_x, tau_y, tau_z] = M @ f
        f = M_INV @ np.array([T, tau[0], tau[1], tau[2]])
        f = np.clip(f, 0.0, MAX_THRUST * 0.35)   # 每个电机最大
        omega = np.sqrt(np.maximum(f, 0.0) / KF)
        omega = np.clip(omega, 0.0, MAX_OMEGA)
        # 桨叶转向：奇偶相反（视觉）
        self.d.ctrl[:4] = omega * DIRS

    # ---------- 发布 ----------
    def publish_odometry(self):
        bid = self.body_id
        pos = self.d.xpos[bid].copy()
        R = self.d.xmat[bid].reshape(3, 3)
        vel = self.d.cvel[bid][3:6].copy() if hasattr(self.d, 'cvel') else np.zeros(3)
        # 转发 NED（相对起飞点近似绝对）
        msg = VehicleOdometry()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        # FLU -> NED： x, -y, -z
        msg.position = [float(pos[0]), float(-pos[1]), float(-pos[2])]
        # 速度（世界系）取自 qvel
        v = self.d.qvel[0:3] if hasattr(self.d, 'qvel') else np.zeros(3)
        msg.velocity = [float(v[0]), float(-v[1]), float(-v[2])]

        # 姿态 FLU -> NED/FRD
        Cne = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
        Cff = np.diag([1.0, -1.0, -1.0])
        R_ned = Cne @ R @ Cff
        tr = np.trace(R_ned)
        if tr > 0:
            S = math.sqrt(tr + 1.0) * 2
            w = 0.25 * S
            x = (R_ned[2, 1] - R_ned[1, 2]) / S
            y = (R_ned[0, 2] - R_ned[2, 0]) / S
            z = (R_ned[1, 0] - R_ned[0, 1]) / S
        else:
            w, x, y, z = 1.0, 0.0, 0.0, 0.0
        msg.q = [float(w), float(x), float(y), float(z)]
        # 角速度 FRD
        wb = self.d.qvel[3:6]
        msg.angular_velocity = [float(wb[0]), float(wb[1]), float(wb[2])]
        self.odom_pub.publish(msg)

    def publish_status(self):
        msg = VehicleStatus()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.arming_state = (VehicleStatus.ARMING_STATE_ARMED if self.armed
                            else VehicleStatus.ARMING_STATE_DISARMED)
        msg.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.status_pub.publish(msg)

    def publish_contacts(self):
        data = []
        n = min(self.d.ncon, 8)
        # 找最深接触
        if self.d.ncon > 0:
            best_i, best_d = 0, -1e9
            for i in range(self.d.ncon):
                depth = -self.d.contact[i].dist
                if depth > best_d:
                    best_d, best_i = depth, i
            c = self.d.contact[best_i]
            data = [1.0, float(c.pos[0]), float(c.pos[1]), float(c.pos[2]),
                    float(c.frame[0]), float(c.frame[1]), float(c.frame[2]),
                    float(c.mu), float(best_d),
                    float(self.d.qvel[0]), float(self.d.qvel[1]), float(self.d.qvel[2])]
        else:
            data = [0.0] * 12
        msg = Float64MultiArray()
        msg.data = data
        self.contact_pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default="/data/DroneNav/src/mujoco_sim/rmuc_2025_drone.xml")
    ap.add_argument("--body", default="x500")
    ap.add_argument("--dist", type=float, default=2.5)
    ap.add_argument("--headless", action="store_true", help="不开窗口，只跑物理+ROS（测试用）")
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(args.xml)
    d = mujoco.MjData(m)
    rclpy.init()
    node = SimNode(m, d, args.body)

    viewer = None
    if not args.headless:
        viewer = mujoco.viewer.launch_passive(m, d)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.distance = args.dist
        viewer.cam.elevation = -20
        viewer.cam.azimuth = 90

    dt = m.opt.timestep
    t_next = time.time()
    try:
        while rclpy.ok() and (viewer is None or viewer.is_running()):
            # 物理
            node.control()
            apply_motor_model(m, d, node.body_id)
            mujoco.mj_step(m, d)

            # 渲染 + 相机跟随
            if viewer is not None:
                viewer.cam.lookat[:] = d.xpos[node.body_id]
                viewer.sync()

            # 发布（odom 每步都发）
            node.publish_odometry()
            node.publish_status()
            if node.step % 100 == 0:
                q = d.qpos[node.free_adr+3:node.free_adr+7]
                fz = d.xfrc_applied[node.body_id][2]
                node.get_logger().info(
                    f"step={node.step} z={d.xpos[node.body_id][2]:.3f} vz={d.qvel[2]:.2f} "
                    f"Fz={fz:.2f}N thrust={node.sp_thrust:.2f} q=({q[0]:.2f},{q[1]:.2f},{q[2]:.2f},{q[3]:.2f})")
            if node.step % 10 == 0:
                node.publish_contacts()
            node.step += 1
            rclpy.spin_once(node, timeout_sec=0.0)

            # 按真实时间配速
            t_next += dt
            sleep = t_next - time.time()
            if sleep > 0:
                time.sleep(sleep)
            else:
                t_next = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
