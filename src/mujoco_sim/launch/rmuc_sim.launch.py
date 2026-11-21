"""RMUC 2025 仿真总启动：外部 MuJoCo/MAVLink HIL + PX4 SITL + ROS2

    ros2 launch mujoco_sim rmuc_sim.launch.py                     # Agent + PX4(内含 MuJoCo)
    ros2 launch mujoco_sim rmuc_sim.launch.py agent:=false        # 不自动起 Agent（手动起）
    ros2 launch mujoco_sim rmuc_sim.launch.py px4:=false          # 只起 Agent
    ros2 launch mujoco_sim rmuc_sim.launch.py world:=/path/xxx.xml # 换 MuJoCo 世界

顺序：桥接（MuJoCo + /clock）→ Agent → PX4 SITL → 接触 relay →恢复节点。
该入口与 bringup sim.launch.py 使用相同的外部 HIL 架构。
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, Shutdown, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

PKG_SHARE = "/data/DroneNav/src/mujoco_sim/launch"


def generate_launch_description():
    px4 = LaunchConfiguration("px4")
    agent = LaunchConfiguration("agent")
    px4_delay = LaunchConfiguration("px4_delay")
    world = LaunchConfiguration("world")
    px4_dir = LaunchConfiguration("px4_dir")
    agent_port = LaunchConfiguration("agent_port")
    gui = LaunchConfiguration("gui")

    bridge = ExecuteProcess(
        cmd=["ros2", "run", "mujoco_sim", "px4_bridge", "--xml", world, "--headless"],
        output="screen",
        condition=UnlessCondition(gui),
        on_exit=Shutdown(),
    )
    bridge_gui = ExecuteProcess(
        cmd=["ros2", "run", "mujoco_sim", "px4_bridge", "--xml", world, "--dist", "3.0"],
        output="screen",
        condition=IfCondition(gui),
        on_exit=Shutdown(),
    )

    agent_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([PKG_SHARE, "agent.launch.py"])),
        launch_arguments={"port": agent_port}.items(),
        condition=IfCondition(agent),
    )

    contact_relay = Node(
        package="mujoco_sim",
        executable="contact_relay",
        name="contact_relay",
        output="screen",
    )

    px4_sitl = TimerAction(
        period=px4_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([PKG_SHARE, "px4_sitl.launch.py"])),
                launch_arguments={"px4_dir": px4_dir, "world": world,
                                  "simulator": "mavlink"}.items(),
            )
        ],
        condition=IfCondition(px4),
    )

    recovery = IncludeLaunchDescription(
        PythonLaunchDescriptionSource("/data/DroneNav/src/recovery/launch/recovery.launch.py"),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("px4", default_value="true", description="是否自动启动 PX4 SITL"),
        DeclareLaunchArgument("px4_delay", default_value="3.0", description="PX4 相对启动的延时(秒)"),
        DeclareLaunchArgument("world",
                                  default_value="/data/DroneNav/src/mujoco_sim/rmuc_2025_drone.xml"),
        DeclareLaunchArgument("px4_dir",
                                  default_value="/data/DroneNav/PX4-Autopilot/build/px4_sitl_default"),
        DeclareLaunchArgument("agent", default_value="true", description="是否自动启动 MicroXRCEAgent"),
        DeclareLaunchArgument("agent_port", default_value="8888", description="Agent UDP 端口"),
        DeclareLaunchArgument("gui", default_value="true", description="是否启动 MuJoCo GUI"),

        LogInfo(msg="======== RMUC 2025 MuJoCo/MAVLink HIL 仿真闭环========"),
        bridge,
        bridge_gui,
        agent_launch,
        contact_relay,
        px4_sitl,
        recovery,
    ])
