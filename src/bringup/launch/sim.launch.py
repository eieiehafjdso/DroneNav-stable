"""一键启动完整仿真闭环（正确架构：MuJoCo 独立进程 + MAVLink HIL）

    ros2 launch bringup sim.launch.py                  # 桥(MuJoCo+HIL) + PX4 + Agent + relay + recovery
    ros2 launch bringup sim.launch.py gui:=false       # 无窗口
    ros2 launch bringup sim.launch.py px4:=false       # 只起 ROS 侧（桥/Agent/relay/recovery）

启动后另开一个终端做键盘遥控（必须用 teleop.launch.py，它带 use_sim_time）：

    ros2 launch drone_teleop teleop.launch.py

数据流：
    键盘 --offboard--> PX4 --电机指令--> px4_bridge(MuJoCo) --接触UDP--> contact_relay --> /mujoco/contacts --> recovery
    MuJoCo --IMU/EV/GPS(MAVLink HIL)--> PX4
    MuJoCo --/clock--> 全部 ROS2 节点 (use_sim_time=true)
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, RegisterEventHandler,
                            Shutdown, TimerAction)
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

_MUJOCO_SHARE = "/data/DroneNav/src/mujoco_sim/launch"
_RECOVERY_SHARE = "/data/DroneNav/src/recovery/launch"


def generate_launch_description():
    world = LaunchConfiguration("world")
    px4_dir = LaunchConfiguration("px4_dir")
    agent_port = LaunchConfiguration("agent_port")
    px4_delay = LaunchConfiguration("px4_delay")
    gui = LaunchConfiguration("gui")
    px4 = LaunchConfiguration("px4")

    # 1) 桥：MuJoCo 物理 + MAVLink HIL + /clock（唯一时间源）
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

    # 2) uXRCE-DDS Agent
    agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([_MUJOCO_SHARE, "agent.launch.py"])),
        launch_arguments={"port": agent_port, "use_sim_time": "true"}.items(),
    )

    # 3) PX4 SITL（用自带 simulator_mavlink 连桥）
    px4_sitl = TimerAction(
        period=px4_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([_MUJOCO_SHARE, "px4_sitl.launch.py"])),
                launch_arguments={"world": world, "px4_dir": px4_dir}.items(),
            )
        ],
        condition=IfCondition(px4),
    )

    # 4) 接触 relay（UDP -> /mujoco/contacts）
    contact_relay = Node(
        package="mujoco_sim", executable="contact_relay", name="contact_relay", output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # 5) 碰撞恢复节点（常驻）
    recovery = TimerAction(
        period=px4_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([_RECOVERY_SHARE, "recovery.launch.py"])),
                launch_arguments={"use_sim_time": "true"}.items(),
            )
        ],
    )

    startup_cleanup = ExecuteProcess(
        cmd=["bash", "-c",
             "fuser -k 4560/tcp 2>/dev/null || true; "
             "fuser -k 8888/udp 2>/dev/null || true; "
             "fuser -k 14577/udp 2>/dev/null || true; "
             "pkill -9 -x px4 2>/dev/null || true; "
             "pkill -9 -x MicroXRCEAgent 2>/dev/null || true"],
        output="log",
    )

    bridge_gui_delayed = TimerAction(period=0.5, actions=[bridge_gui])
    bridge_delayed = TimerAction(period=0.5, actions=[bridge])
    agent_delayed = TimerAction(period=0.5, actions=[agent])

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="/data/DroneNav/src/mujoco_sim/rmuc_2025_drone.xml",
                              description="MuJoCo 世界路径"),
        DeclareLaunchArgument("px4_dir", default_value="/data/DroneNav/PX4-Autopilot/build/px4_sitl_default",
                              description="PX4 SITL build 目录"),
        DeclareLaunchArgument("agent_port", default_value="8888", description="MicroXRCEAgent UDP 端口"),
        DeclareLaunchArgument("px4_delay", default_value="3.0", description="PX4/recovery 相对桥的启动延时(秒)"),
        DeclareLaunchArgument("gui", default_value="true", description="是否开 MuJoCo 可视化窗口"),
        DeclareLaunchArgument("px4", default_value="true", description="是否启动 PX4 SITL"),

        LogInfo(msg="======== RMUC 2025 仿真闭环（MuJoCo 独立进程 + MAVLink HIL）========"),
        RegisterEventHandler(OnShutdown(on_shutdown=[
            ExecuteProcess(
                cmd=["bash", "-c",
                     "fuser -k 4560/tcp 2>/dev/null || true; "
                     "fuser -k 8888/udp 2>/dev/null || true; "
                     "fuser -k 14577/udp 2>/dev/null || true; "
                     "pkill -9 -x px4 2>/dev/null || true; "
                     "pkill -9 -x MicroXRCEAgent 2>/dev/null || true"],
                output="log",
            ),
        ])),
        startup_cleanup,
        bridge_delayed,
        bridge_gui_delayed,
        agent_delayed,
        contact_relay,
        px4_sitl,
        recovery,
    ])
