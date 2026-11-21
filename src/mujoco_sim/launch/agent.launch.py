"""MicroXRCEAgent 独立启动（常驻）

用法：
    ros2 launch mujoco_sim agent.launch.py
    ros2 launch mujoco_sim agent.launch.py port:=8888

作用：把 PX4 的 uORB 话题桥接成 ROS2 话题（/fmu/out/*、/fmu/in/*），
PX4 侧的 uxrce_dds_client 通过它建 DDS 实体。

注意：
  * 必须比 PX4 先起、且常驻（PX4 启动时连 Agent，晚了就收不到话题）。
  * PX4 实例 0 的话题是 /fmu/...，-i N 是 /px4_N/fmu/...
  * 前提：PX4 里要 param set UXRCE_DDS_SYNCT 0（否则 uxrce 卡在时间同步，
    一个 topic 都不建）。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    port = LaunchConfiguration("port")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="（Agent 为二进制，忽略；仅为 launch 参数一致）"),
        DeclareLaunchArgument("port", default_value="8888",
                              description="Agent UDP 端口（PX4 侧 uxrce_dds_client 默认也是 8888）"),
        LogInfo(msg=["MicroXRCEAgent udp4 -p ", port]),
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", port],
            output="screen",
        ),
    ])
