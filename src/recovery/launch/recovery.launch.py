"""启动碰撞恢复节点（常驻），加载 config/recovery_params.yaml 参数。

    ros2 launch recovery recovery.launch.py
    ros2 launch recovery recovery.launch.py params_file:=<路径>

注意：恢复节点只负责"检测碰撞并接管恢复"，不负责正常飞行/键盘控制。
正常飞行由 drone_teleop（键盘）或其他飞行节点负责，它订阅 /recovery/setpoint，
在恢复期间转发该 setpoint 到 PX4。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "config", "recovery_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="碰撞恢复参数文件路径"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="用 /clock (MuJoCo 时间源)"),
        Node(
            package="recovery",
            executable="collision_recovery_node",
            name="collision_recovery",
            output="screen",
            parameters=[LaunchConfiguration("params_file"),
                        {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
