"""启动 PX4 SITL（默认使用外部 MuJoCo/MAVLink HIL 桥）

    ros2 launch mujoco_sim px4_sitl.launch.py
    ros2 launch mujoco_sim px4_sitl.launch.py world:=/path/rmuc_2025_drone.xml

默认 PX4_SIMULATOR=mavlink，由 px4-rc.mavlinksim 启动 simulator_mavlink，
再由 px4_bridge.py 接收 PX4 电机输出并驱动外部 MuJoCo。
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    px4_dir = LaunchConfiguration("px4_dir")
    world = LaunchConfiguration("world")
    simulator = LaunchConfiguration("simulator")

    mujoco_lib = "/data/MuJoCo/MuJoCo-drones-gym/.venv/lib/python3.10/site-packages/mujoco"

    return LaunchDescription([
        DeclareLaunchArgument(
            "px4_dir",
            default_value="/data/DroneNav/PX4-Autopilot/build/px4_sitl_default",
            description="PX4 SITL build 目录（含 bin/px4 和 etc/init.d-posix/rcS）"),
        DeclareLaunchArgument(
            "world",
            default_value="/data/DroneNav/src/mujoco_sim/rmuc_2025_drone.xml",
            description="MuJoCo 世界路径"),
        DeclareLaunchArgument(
            "simulator",
            default_value="mavlink",
            description="PX4 模拟器后端：mavlink 用外部 MuJoCo bridge，mujoco 用进程内 MuJoCo"),

        LogInfo(msg=["[mujoco_sim] 启动 PX4 SITL（simulator=", simulator, "）..."]),

        # 先清掉上一次残留的 px4（instance 0 锁），否则新 px4 会报 "already running" 直接退出
        ExecuteProcess(
            cmd=["bash", "-c", "pkill -9 -x px4 2>/dev/null || true"],
            output="log",
        ),

        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=["./bin/px4", "-i", "0", "-s", "etc/init.d-posix/rcS"],
                    cwd=px4_dir,
                    env=dict(os.environ,
                             PX4_SIMULATOR=simulator,
                             PX4_SYS_AUTOSTART="10016",
                             MUJOCO_MODEL=world,
                             LD_LIBRARY_PATH=mujoco_lib + ":" + os.environ.get("LD_LIBRARY_PATH", "")),
                    output="screen",
                ),
            ],
        ),
    ])
