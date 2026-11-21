#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""碰撞恢复节点（常驻运行，仿真/真机通用）

直接复用 dq_collision_recovery 的对偶四元数参考实现，所有关键参数都从 ROS2 参数
读取（见 config/recovery_params.yaml），仿真和真机只需改一份配置文件。

职责：
  1. 一直运行，订阅 PX4 里程计 + 接触/碰撞源；
  2. 检测到碰撞（接触深度超阈值）时，用论文 Eq.22~23a 计算导纳让步参考；
  3. 每个控制周期用 Eq.24 几何控制算出期望力，转成 PX4 姿态+推力 setpoint；
  4. 收敛后停止接管，交还飞行节点。

话题（默认，可在参数里改）：
  订阅 /fmu/out/vehicle_odometry     (px4_msgs/VehicleOdometry)
       /fmu/out/vehicle_status_v1    (px4_msgs/VehicleStatus)
       /mujoco/contacts              (std_msgs/Float64MultiArray，仿真接触；真机可换成别的)
  发布 /recovery/setpoint            (px4_msgs/VehicleAttitudeSetpoint，仅恢复期间)
       /recovery/status              (std_msgs/String)

坐标：内部 ENU 世界 / FLU 机体（论文约定），只在接口处转 PX4 NED/FRD。
"""
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import (VehicleOdometry, VehicleStatus, VehicleAttitudeSetpoint,
                         VehicleThrustSetpoint, VehicleTorqueSetpoint)
from std_msgs.msg import Float64MultiArray, String

# 让 dq_lib 可以被 import（安装后在 share/recovery/dq_lib，源码运行在 recovery/dq_lib）
_here = os.path.dirname(os.path.abspath(__file__))
for _p in [
    os.path.join(_here, "..", "dq_lib"),
    os.path.join(_here, "..", "..", "share", "recovery", "dq_lib"),
    os.path.join(_here, "dq_lib"),
]:
    if os.path.isdir(_p):
        sys.path.insert(0, os.path.abspath(_p))
        break

from dual_quaternion import quat_inv_rotate, quat_rotate, rot_matrix, quat_conj, quat_mult, quat_exp  # noqa
from recovery_controller import (kinetic_energy, potential_error, admittance_gains,  # noqa
                                 dq_admittance_shift, geometric_control)
from reset_map import dq_reset, contact_tangent  # noqa

CNE = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
CFF = np.diag([1.0, -1.0, -1.0])


def q_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def R_to_q(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    if R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s])
    s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s])


def q_enu_to_ned(q):
    return R_to_q(CNE @ q_to_R(q) @ CFF)


def attitude_from_force(F_W, yaw):
    """世界系期望力 -> (q_des, thrust)，b3 对准 F_W，航向 yaw(ENU)"""
    n = np.linalg.norm(F_W)
    if n < 1e-6:
        F_W = np.array([0.0, 0.0, 1e-3])
        n = np.linalg.norm(F_W)
    b3 = F_W / n
    # Collision recovery safety: never command an inverted or near-inverted attitude.
    max_tilt = np.deg2rad(15.0)
    horizontal = np.array([b3[0], b3[1]], float)
    horizontal_norm = float(np.linalg.norm(horizontal))
    max_horizontal = float(np.sin(max_tilt))
    if horizontal_norm > max_horizontal:
        horizontal *= max_horizontal / horizontal_norm
        b3 = np.array([horizontal[0], horizontal[1],
                       np.sqrt(max(0.0, 1.0 - float(horizontal @ horizontal)))])
        b3 /= np.linalg.norm(b3)
    b1c = np.array([np.cos(yaw), np.sin(yaw), 0.0])
    b2 = np.cross(b3, b1c)
    if np.linalg.norm(b2) < 1e-6:
        b2 = np.cross(b3, np.array([1.0, 0.0, 0.0]))
    b2 = b2 / np.linalg.norm(b2)
    b1 = np.cross(b2, b3)
    Rd = np.column_stack([b1, b2, b3])
    return R_to_q(Rd), n


class RecoveryNode(Node):
    def __init__(self):
        super().__init__("collision_recovery")

        # ---- 机体/惯性 ----
        self.declare_parameter("mass", 2.0643)
        self.declare_parameter("jx", 0.02167)
        self.declare_parameter("jy", 0.02167)
        self.declare_parameter("jz", 0.04)
        # ---- 几何控制增益 (Eq.24) ----
        self.declare_parameter("k_q", 2.0)
        self.declare_parameter("k_p", 2.0)
        self.declare_parameter("k_dw", 0.6)
        self.declare_parameter("k_dv", 1.2)
        # ---- 导纳让步 (Eq.22~23a) ----
        self.declare_parameter("alpha", 0.5)
        self.declare_parameter("admittance_factor", 0.25)
        self.declare_parameter("max_admittance_shift", 0.6)
        self.declare_parameter("recovery_shift", 0.6)
        self.declare_parameter("restitution", 0.2)
        # ---- 推力限幅 / 恢复高度 ----
        self.declare_parameter("thrust_max_accel", 16.6)
        self.declare_parameter("min_height", 0.4)
        # ---- 触发 / 收敛 / 时序 ----
        self.declare_parameter("contact_depth_threshold", 1e-4)
        self.declare_parameter("contact_timeout", 0.2)
        self.declare_parameter("cooldown", 0.5)
        self.declare_parameter("settle_vel_threshold", 0.15)
        self.declare_parameter("settle_rate_threshold", 0.2)
        self.declare_parameter("settle_pos_threshold", 0.25)
        self.declare_parameter("settle_time", 0.6)
        self.declare_parameter("max_recover_time", 4.0)
        self.declare_parameter("control_rate", 50.0)
        # ---- 话题（真机可改） ----
        self.declare_parameter("odometry_topic", "/fmu/out/vehicle_odometry")
        self.declare_parameter("status_topic", "/fmu/out/vehicle_status_v1")
        self.declare_parameter("contact_topic", "/mujoco/contacts")
        self.declare_parameter("setpoint_topic", "/recovery/setpoint")
        self.declare_parameter("thrust_setpoint_topic", "/recovery/thrust_setpoint")
        self.declare_parameter("torque_setpoint_topic", "/recovery/torque_setpoint")
        self.declare_parameter("recovery_status_topic", "/recovery/status")
        self.declare_parameter("reset_topic", "/mujoco/reset")

        p = self.get_parameter
        self.m = float(p("mass").value)
        self.J = np.diag([float(p("jx").value), float(p("jy").value), float(p("jz").value)])
        self.k_q = float(p("k_q").value)
        self.k_p = float(p("k_p").value)
        self.k_dw = float(p("k_dw").value)
        self.k_dv = float(p("k_dv").value)
        self.alpha = float(p("alpha").value)
        self.factor = float(p("admittance_factor").value)
        self.max_adm_shift = float(p("max_admittance_shift").value)
        self.recovery_shift = float(p("recovery_shift").value)
        self.restitution = float(p("restitution").value)
        self.max_accel = float(p("thrust_max_accel").value)
        self.min_height = float(p("min_height").value)

        self.depth_thr = float(p("contact_depth_threshold").value)
        self.contact_timeout = float(p("contact_timeout").value)
        self.cooldown_s = float(p("cooldown").value)
        self.settle_v = float(p("settle_vel_threshold").value)
        self.settle_w = float(p("settle_rate_threshold").value)
        self.settle_p = float(p("settle_pos_threshold").value)
        self.settle_s = float(p("settle_time").value)
        self.max_recover = float(p("max_recover_time").value)
        self.rate = float(p("control_rate").value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(VehicleOdometry, str(p("odometry_topic").value), self.on_odom, qos)
        self.create_subscription(VehicleStatus, str(p("status_topic").value), self.on_status, qos)
        self.create_subscription(Float64MultiArray, str(p("contact_topic").value), self.on_contacts, 10)
        self.create_subscription(String, str(p("reset_topic").value), self.on_reset, 10)
        self.sp_pub = self.create_publisher(
            VehicleAttitudeSetpoint, str(p("setpoint_topic").value), qos)
        self.thrust_sp_pub = self.create_publisher(
            VehicleThrustSetpoint, str(p("thrust_setpoint_topic").value), qos)
        self.torque_sp_pub = self.create_publisher(
            VehicleTorqueSetpoint, str(p("torque_setpoint_topic").value), qos)
        self.status_pub = self.create_publisher(String, str(p("recovery_status_topic").value), 10)

        # 状态
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0])
        self.w = np.zeros(3)
        self.v_body = np.zeros(3)
        self.odom_ok = False
        self.odom_stamp = None
        self.armed = False
        self.contact_depth = 0.0
        self.contact_msg = None
        self.contact_stamp = None
        self.contact_clear = True
        self.cooldown_until = None

        self.recovering = False
        self.q_c = None
        self.p_c = None
        self.q_d = None
        self.p_d = None
        self.preimpact_q = None
        self.preimpact_p = None
        self.recovery_away = np.zeros(3)
        self.recover_t = 0.0
        self.settle_t = None
        self._last_status_ns = 0

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.get_logger().info("碰撞恢复节点已启动（常驻，等待接触事件）")

    # ---------- 回调 ----------
    def on_odom(self, msg: VehicleOdometry):
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return
        p_ned = np.array(msg.position, float)
        v_ned = np.array(msg.velocity, float)
        q_ned = np.array(msg.q, float)
        w_frd = np.array(msg.angular_velocity, float)

        self.p = CNE @ p_ned
        self.v = CNE @ v_ned
        self.q = R_to_q(CNE @ q_to_R(q_ned) @ CFF)
        self.w = CFF @ w_frd
        self.v_body = quat_inv_rotate(self.q, self.v)
        self.odom_ok = True
        self.odom_stamp = self.get_clock().now()

    def on_status(self, msg: VehicleStatus):
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)

    def on_reset(self, msg: String):
        self.recovering = False
        self.contact_clear = True
        self.contact_depth = 0.0
        self.contact_msg = None
        self.cooldown_until = self.get_clock().now()
        self.q_c = None
        self.p_c = None
        self.q_d = None
        self.p_d = None
        self.preimpact_q = None
        self.preimpact_p = None
        self.publish_status("RECOVER reset")

    def on_contacts(self, msg: Float64MultiArray):
        if len(msg.data) < 1:
            return
        n = int(msg.data[0])
        if n <= 0 or len(msg.data) < 1 + 11:
            self.contact_depth = 0.0
            return
        self.contact_depth = float(msg.data[8])
        self.contact_msg = msg.data
        self.contact_stamp = self.get_clock().now()

    # ---------- 恢复 ----------
    def start_recovery(self, data):
        # px4_bridge 已将接触点、法向和速度转换为 Recovery 的 ENU 世界系。
        px, py, pz = data[1], data[2], data[3]
        nx, ny, nz = data[4], data[5], data[6]
        vx, vy, vz = data[9], data[10], data[11]
        n_w = np.array([nx, ny, nz], float)
        n_w /= max(np.linalg.norm(n_w), 1e-9)
        # 接触法向统一成远离撞击方向，并使用完整撞击重置映射。
        v_c = np.array([vx, vy, vz], float)
        if v_c @ n_w > 0:
            n_w = -n_w

        self.q_c = self.q.copy()
        self.p_c = self.p.copy()
        self.preimpact_q = self.q.copy()
        self.preimpact_p = self.p.copy()
        r_world = np.array([px, py, pz], float) - self.p_c
        r_body = quat_inv_rotate(self.q_c, r_world)
        t_world = contact_tangent(v_c, self.w, self.q_c, r_body, n_w)
        omega_plus, v_world_plus = dq_reset(r_body, n_w, t_world, self.q_c,
                                             self.restitution, float(data[7]),
                                             self.w, v_c, self.J, self.m)
        v_body_plus = quat_inv_rotate(self.q_c, v_world_plus)

        ke_before = kinetic_energy(self.w, self.v_body, self.J, self.m)
        ke_after = kinetic_energy(omega_plus, v_body_plus, self.J, self.m)
        e_diss = max(ke_before - ke_after, 0.0)
        q_e_pre = quat_mult(quat_conj(self.preimpact_q), self.q_c)
        if q_e_pre[0] < 0.0:
            q_e_pre = -q_e_pre
        p_e_pre = quat_inv_rotate(self.preimpact_q, self.p_c - self.preimpact_p)
        e_budget = e_diss + potential_error(q_e_pre, p_e_pre, self.k_q, self.k_p)
        g_om, g_v = admittance_gains(omega_plus, v_body_plus, e_budget,
                                     self.k_q, self.k_p, self.alpha, self.factor)
        self.q_d, self.p_d = dq_admittance_shift(self.q_c, self.p_c, omega_plus,
                                                 v_body_plus, g_om, g_v)
        # 论文 DQ shift 之外再施加一个确定的远离接触点位移。
        # 这避免碰撞法向符号或接触点局部坐标出错时仍指向障碍物。
        r_world = np.array([px, py, pz], float) - self.p_c
        if np.linalg.norm(r_world) > 1e-6:
            away = -r_world / np.linalg.norm(r_world)
        else:
            away = -n_w
        if np.dot(away, v_c) > 0.0:
            away = -away
        shift = float(np.clip(self.recovery_shift, 0.15, self.max_adm_shift))
        away_p = self.p_c + shift * away
        self.p_d = np.clip(
            away_p,
            np.asarray([self.p_c[0] - shift, self.p_c[1] - shift, self.min_height]),
            np.asarray([self.p_c[0] + shift, self.p_c[1] + shift, np.inf]),
        )
        # 反应恢复只做平移让步，不在碰撞瞬间叠加旋转让步；航向保持碰撞瞬间姿态。
        self.q_d = self.q_c.copy()
        self.recovery_away = away
        self.recovering = True
        self.recover_t = self.get_clock().now()
        self.settle_t = None
        self.get_logger().warn(
            "碰撞！p=(%.2f,%.2f,%.2f) |v|=%.2f |w|=%.2f Γω=%.3f Γv=%.3f depth=%.3f"
            % (self.p[0], self.p[1], self.p[2], np.linalg.norm(v_body_plus),
               np.linalg.norm(omega_plus), g_om, g_v, self.contact_depth))
        self.publish_status("RECOVER started")

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def publish_direct_setpoints(self, tau_flu, f_flu):
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        tau_frd = CFF @ tau_flu
        f_frd = CFF @ f_flu

        thrust = VehicleThrustSetpoint()
        thrust.timestamp = now_us
        thrust.timestamp_sample = now_us
        thrust.xyz = [float(np.clip(f_frd[i] / (self.m * self.max_accel), -1.0, 1.0))
                      for i in range(3)]

        torque = VehicleTorqueSetpoint()
        torque.timestamp = now_us
        torque.timestamp_sample = now_us
        tau_limit = 1.0
        torque.xyz = [float(np.clip(tau_frd[i] / tau_limit, -1.0, 1.0)) for i in range(3)]
        self.thrust_sp_pub.publish(thrust)
        self.torque_sp_pub.publish(torque)

    def publish_setpoint(self, q_des_enu, thrust):
        msg = VehicleAttitudeSetpoint()
        q_ned = q_enu_to_ned(q_des_enu)
        msg.q_d = [float(q_ned[0]), float(q_ned[1]), float(q_ned[2]), float(q_ned[3])]
        n = float(np.clip(thrust / (self.m * self.max_accel), 0.0, 1.0))
        msg.thrust_body = [0.0, 0.0, -n]
        msg.yaw_sp_move_rate = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.sp_pub.publish(msg)

    # ---------- 主循环 ----------
    def tick(self):
        now = self.get_clock().now()
        if not self.odom_ok or (now - self.odom_stamp).nanoseconds > 5e8:
            return

        # 接触消息超时 -> 认为已脱离接触
        if self.contact_stamp is not None and (now - self.contact_stamp).nanoseconds > self.contact_timeout * 1e9:
            self.contact_depth = 0.0
        if self.contact_depth <= self.depth_thr:
            self.contact_clear = True

        # 触发：已解锁 + 有高度 + 有穿透深度 + 本次接触尚未恢复过
        cooling = (self.cooldown_until is not None
                   and (now - self.cooldown_until).nanoseconds < self.cooldown_s * 1e9)
        if (not self.recovering and self.contact_clear and not cooling
                and self.armed and self.contact_depth > self.depth_thr
                and self.p[2] > self.min_height and self.contact_msg is not None):
            self.contact_clear = False
            self.start_recovery(self.contact_msg)
            return

        if not self.recovering:
            return

        # 论文 Eq.24
        tau, f = geometric_control(self.q, self.p, self.v_body, self.w,
                                   self.q_d, self.p_d, self.J, self.m,
                                   self.k_q, self.k_p, self.k_dw, self.k_dv)
        F_W = rot_matrix(self.q) @ f
        yaw = 2.0 * np.arctan2(self.q[3], self.q[0])
        q_des, thrust = attitude_from_force(F_W, yaw)
        self.publish_setpoint(q_des, thrust)

        pos_error = np.linalg.norm(quat_inv_rotate(self.q_d, self.p - self.p_d))
        settled = (np.linalg.norm(self.v) < self.settle_v and np.linalg.norm(self.w) < self.settle_w
                   and pos_error < self.settle_p and np.linalg.norm(tau) < 0.5)
        if settled:
            if self.settle_t is None:
                self.settle_t = now
        else:
            self.settle_t = None
        elapsed = (now - self.recover_t).nanoseconds * 1e-9
        if (self.settle_t is not None and (now - self.settle_t).nanoseconds > self.settle_s * 1e9) \
                or elapsed > self.max_recover:
            self.recovering = False
            self.cooldown_until = now
            self.get_logger().info("恢复完成（%.2fs），交还飞行节点" % elapsed)
            self.publish_status("RECOVER done")
        elif (now.nanoseconds - self._last_status_ns) > 2e8:
            self._last_status_ns = now.nanoseconds
            self.publish_status("RECOVER |v|=%.2f |w|=%.2f t=%.1f" % (
                np.linalg.norm(self.v), np.linalg.norm(self.w), elapsed))


def main():
    rclpy.init()
    node = RecoveryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
