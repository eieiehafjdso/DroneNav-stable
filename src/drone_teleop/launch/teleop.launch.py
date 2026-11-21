from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_params = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "config", "teleop_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=default_params),
        Node(
            package="drone_teleop",
            executable="drone_keyboard_teleop",
            name="keyboard_teleop",
            output="screen",
            emulate_tty=True,
            parameters=[LaunchConfiguration("params_file"),
                        {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
