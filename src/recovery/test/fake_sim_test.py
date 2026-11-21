#!/usr/bin/env python3
"""离线闭环测试：假 PX4 里程计/状态 + 假接触，验证 collision_recovery_node 状态机。

不依赖 MuJoCo/PX4：只起 recovery 节点 + 本假数据源，观察碰撞触发/恢复状态。
用法（两个终端）：
    终端1: ros2 run recovery collision_recovery_node
    终端2: python3 fake_sim_test.py
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry, VehicleStatus, VehicleAttitudeSetpoint
from std_msgs.msg import Float64MultiArray, String


class FakeSim(Node):
    def __init__(self):
        super().__init__("fake_px4")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.odom_pub = self.create_publisher(VehicleOdometry, "/fmu/out/vehicle_odometry", 10)
        self.status_pub = self.create_publisher(VehicleStatus, "/fmu/out/vehicle_status_v1", 10)
        self.contact_pub = self.create_publisher(Float64MultiArray, "/mujoco/contacts", 10)
        self.create_subscription(String, "/recovery/status", self.on_recovery, 10)
        self.create_subscription(VehicleAttitudeSetpoint, "/recovery/setpoint", self.on_setpoint, qos)

        self.t0 = time.time()
        self.last_status = ""
        self.got_setpoint = False
        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info("fake sim started")

    def on_recovery(self, msg: String):
        print(f"[recovery {time.time()-self.t0:5.2f}s] {msg.data}", flush=True)

    def on_setpoint(self, msg: VehicleAttitudeSetpoint):
        if not self.got_setpoint:
            self.got_setpoint = True
            print(f"[recovery {time.time()-self.t0:5.2f}s] 收到 setpoint thrust={msg.thrust_body}", flush=True)

    def tick(self):
        t = time.time() - self.t0

        # 假高度曲线：0 -> 1.5m，然后沿 +x 飞
        z = min(1.5, 1.5 * t / 3.0) if t > 0.5 else 0.0
        x = 0.0
        if t > 3.5:
            x = min(2.0, 1.0 * (t - 3.5))
        self.publish_odom(x, 0.0, z)

        st = VehicleStatus()
        st.timestamp = int(time.time() * 1e6)
        st.arming_state = VehicleStatus.ARMING_STATE_ARMED
        st.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
        st.pre_flight_checks_pass = True
        self.status_pub.publish(st)

        # 8.5s 时给一个碰撞（持续 0.4s）
        if 8.5 < t < 8.9:
            msg = Float64MultiArray()
            msg.data = [1.0,
                        x, 0.0, z,      # 接触点
                        1.0, 0.0, 0.0,  # 法向 (指向 +x)
                        0.3, 0.01,      # mu, depth
                        1.0, 0.0, 0.0]  # 接触点速度
            self.contact_pub.publish(msg)

    def publish_odom(self, x, y, z):
        od = VehicleOdometry()
        od.timestamp = int(time.time() * 1e6)
        od.timestamp_sample = od.timestamp
        od.pose_frame = VehicleOdometry.POSE_FRAME_NED
        od.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        # ENU -> NED: (e, n, -u)
        od.position = [float(y), float(x), float(-z)]
        od.q = [1.0, 0.0, 0.0, 0.0]
        od.velocity = [0.0, 0.0, 0.0]
        od.angular_velocity = [0.0, 0.0, 0.0]
        self.odom_pub.publish(od)


def main():
    rclpy.init()
    node = FakeSim()
    end = time.time() + 14.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.01)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
