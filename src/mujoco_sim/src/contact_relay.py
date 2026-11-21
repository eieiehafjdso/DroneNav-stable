#!/usr/bin/env python3
"""接触 relay：把外部 MuJoCo bridge 发的 UDP 接触数据转成 /mujoco/contacts

px4_bridge.py 在外部进程跑 MuJoCo，通过 UDP 127.0.0.1:14577 发最深的接触
（12 个 double：n, px,py,pz, nx,ny,nz, mu, depth, vx,vy,vz），本节点收下来转成
std_msgs/Float64MultiArray 发布，供 recovery 节点订阅。

    ros2 run mujoco_sim contact_relay
"""
import socket
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class ContactRelay(Node):
    def __init__(self):
        super().__init__("contact_relay")
        self.declare_parameter("port", 14577)
        port = self.get_parameter("port").value
        self.pub = self.create_publisher(Float64MultiArray, "/mujoco/contacts", 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", port))
        self.sock.settimeout(0.01)

        self.timer = self.create_timer(0.02, self.tick)
        self.get_logger().info(f"接触 relay 监听 127.0.0.1:{port} -> /mujoco/contacts")

    def tick(self):
        try:
            data, _ = self.sock.recvfrom(4096)
        except socket.timeout:
            return
        if len(data) < 96:  # 12 double
            return
        vals = struct.unpack("<12d", data[:96])
        msg = Float64MultiArray()
        msg.data = list(vals)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ContactRelay()
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
