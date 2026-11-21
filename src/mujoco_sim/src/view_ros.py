#!/usr/bin/env python3
"""ROS2 实时渲染 MuJoCo 无人机（订阅 /fmu/out/vehicle_odometry）

和外部 px4_bridge 配套：PX4 HIL 中的 MuJoCo 物理（无头），EKF 位姿经
uXRCE-DDS 桥成 ROS2 话题，本 viewer 订阅并把无人机渲染到 MuJoCo 场景里。

    ros2 run mujoco_sim view_ros --ros-args -p odom_topic:=/fmu/out/vehicle_odometry
    # 或直接用脚本（需先 source 工作空间）:
    #   python3 src/view_ros.py --xml rmuc_2025_drone.xml

坐标：odometry.position 是 NED（相对起飞点），本脚本转回 MuJoCo 世界 FLU(x前/y左/z上)，
q(body FRD→NED) 转回 body FLU→world FLU。
"""
import argparse
import os
import sys
import threading
import time

import numpy as np
import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry


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


def ned_to_mujoco_q(q_ned):
    """body FRD→NED 四元数 -> body FLU→world FLU 四元数"""
    R_b2n = q_to_R(q_ned)
    S = np.diag([1.0, -1.0, -1.0])
    R = S @ R_b2n @ S          # R = N·R_b2n·B，N=B=diag(1,-1,-1)
    q = R_to_q(R)
    # MuJoCo 用 w,x,y,z
    return np.array([q[0], q[1], q[2], q[3]])


class OdometryViewer(Node):
    def __init__(self, m, d, origin, body_name="x500"):
        super().__init__("mujoco_viewer")
        self.declare_parameter("odom_topic", "/fmu/out/vehicle_odometry")
        topic = self.get_parameter("odom_topic").value
        # PX4 的 uxrce 走 BEST_EFFORT，必须匹配否则收不到
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(VehicleOdometry, topic, self.on_odom, qos)
        self.m = m
        self.d = d
        self.origin = origin
        self.body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_name)
        self.latest = None
        self.get_logger().info(f"订阅 {topic}，渲染到 body[{self.body_id}]")

    def on_odom(self, msg: VehicleOdometry):
        if msg.pose_frame != VehicleOdometry.POSE_FRAME_NED:
            return
        p_ned = np.array(msg.position, float)
        q_ned = np.array(msg.q, float)
        # NED(相对起飞点) -> MuJoCo 世界 FLU 绝对位置
        pos_flu = np.array([
            self.origin[0] + p_ned[0],
            self.origin[1] - p_ned[1],
            self.origin[2] - p_ned[2],
        ])
        self.latest = (pos_flu, ned_to_mujoco_q(q_ned))

    def apply(self):
        if self.latest is None:
            return
        pos_flu, q_flu = self.latest
        if self.body_id >= 0:
            # 更新无人机 free joint 的 qpos（前 3 位置 + 后 4 四元数 wxyz）
            adr = self.m.jnt_qposadr[self.m.body_jntadr[self.body_id]]
            self.d.qpos[adr:adr + 3] = pos_flu
            self.d.qpos[adr + 3:adr + 7] = q_flu
        mujoco.mj_forward(self.m, self.d)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default="/data/DroneNav/src/mujoco_sim/rmuc_2025_drone.xml")
    ap.add_argument("--body", default="x500")
    ap.add_argument("--dist", type=float, default=2.5)
    ap.add_argument("--blink", type=float, default=0.6,
                    help="中心灯闪烁周期(秒)，0 关闭")
    args, unknown = ap.parse_known_args()   # 忽略并透传 ROS2 的 --ros-args

    m = mujoco.MjModel.from_xml_path(args.xml)
    d = mujoco.MjData(m)
    origin = d.qpos[:3].copy()          # 起飞点（MuJoCo 世界 FLU）

    # 中心灯（用于闪烁）
    lamp_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_LIGHT, "center_lamp")
    lamp_diffuse = m.light_diffuse[lamp_id].copy() if lamp_id >= 0 else None

    rclpy.init(args=unknown)
    node = OdometryViewer(m, d, origin, args.body)
    body_id = node.body_id

    viewer = mujoco.viewer.launch_passive(m, d)
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.distance = args.dist
    viewer.cam.elevation = -20
    viewer.cam.azimuth = 90

    t0 = time.time()
    last_blink = 0.0
    blink_on = True

    while rclpy.ok() and viewer.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        node.apply()

        # 镜头实时跟随无人机
        if body_id >= 0:
            viewer.cam.lookat[:] = d.xpos[body_id]

        # 中心灯闪烁
        if lamp_id >= 0 and args.blink > 0:
            t = time.time() - t0
            if t - last_blink >= args.blink:
                last_blink = t
                blink_on = not blink_on
                m.light_diffuse[lamp_id] = lamp_diffuse if blink_on else [0.0, 0.0, 0.0]

        viewer.sync()

    viewer.close()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == "__main__":
    main()
