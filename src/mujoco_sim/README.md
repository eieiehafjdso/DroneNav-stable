# mujoco_sim —— RMUC 2025 场地 + PX4 x500（MuJoCo）

给 PX4 SITL 用的 MuJoCo 仿真环境：RMUC 2025 场地 + PX4 x500 机体（真实网格/质量/惯量）。

当前闭环使用外部 `px4_bridge.py` + MAVLink HIL：
`PX4 -> HIL_ACTUATOR_CONTROLS -> MuJoCo -> HIL_SENSOR/ODOMETRY -> PX4`

## 快速开始（一条命令）

```bash
# 1) 编译一次
cd /data/DroneNav/src
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_sim

# 2) 一条命令起全部：Agent + PX4（PX4 内起 MuJoCo）
ros2 launch mujoco_sim rmuc_sim.launch.py
```

常用参数：

```bash
ros2 launch mujoco_sim rmuc_sim.launch.py agent:=false        # 不自动起 Agent
ros2 launch mujoco_sim rmuc_sim.launch.py px4:=false          # 只起 Agent
ros2 launch mujoco_sim rmuc_sim.launch.py px4_delay:=5.0      # PX4 延后 5 秒启动
ros2 launch mujoco_sim rmuc_sim.launch.py world:=/path/rmuc_2025_field.xml
```

单独启动：

```bash
ros2 launch mujoco_sim agent.launch.py        # 只起 MicroXRCEAgent
ros2 launch mujoco_sim px4_sitl.launch.py     # 只起 PX4（配合外部 MuJoCo bridge）
./launch/start_agent.sh                       # 不用 ros2 launch 起 Agent
```

> PX4 侧需要一次性 `param set UXRCE_DDS_SYNCT 0`（否则 uxrce 卡时间同步，ROS2 收不到话题）。

## 目录结构

```
mujoco_sim/
├── rmuc_2025_drone.xml      成品世界（场地 + x500），仿真入口
├── rmuc_2025_field.xml      只有场地
├── package.xml / CMakeLists.txt   ROS2 ament 包（包名 mujoco_sim）
├── launch/
│   ├── rmuc_sim.launch.py   总启动：MuJoCo bridge + Agent + PX4
│   ├── px4_sitl.launch.py   只起 PX4 SITL（PX4_SIMULATOR=mavlink）
│   ├── agent.launch.py      只起 MicroXRCEAgent
│   ├── start_agent.sh       不用 ROS 起 Agent 的脚本
│   └── view_only.sh         只看模型（viewer，镜头跟随 x500）
├── src/
│   ├── motor_model.py       旋翼推力模型 F=KF·ω²、反扭矩（viewer 拖动测试用）
│   └── view.py              MuJoCo 查看器（只显示，不接 PX4）
├── tools/
│   ├── sdf2mjcf.py          Gazebo SDF → MuJoCo MJCF 转换器
│   ├── build_x500.py        从 PX4 的 x500 SDF/网格生成无人机
│   └── build.sh             一键重建世界
├── meshes/                  场地网格 + x500 网格
├── assets/field_floor.png   场地贴图
└── docs/参数与用法.md        场地生成参数说明
```

## 与 PX4 的接口

```
px4_bridge.py（外部进程）：
  ├─ 接收 PX4 HIL_ACTUATOR_CONTROLS（4 路 0~1）
  ├─ 转成转速并施加推力、反扭矩
  ├─ 发 HIL_SENSOR/HIL_STATE_QUATERNION/HIL_GPS 与 ODOMETRY
  └─ 发布 /clock 与 /mujoco/reset
```

ROS2 侧仍走 MicroXRCEAgent 把 uORB 桥成 `/fmu/*` 话题。

## 实时可视化（看 PX4 里的实际飞行）

可用 `view_ros.py` 订阅 EKF 位姿渲染：

```bash
source /data/DroneNav/install/setup.bash
ros2 run mujoco_sim view_ros               # 需 Agent + PX4 已在跑
# 或 NVIDIA 独显渲染（和 view_only.sh 一样）：
#   __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
#   __VK_LAYER_NV_optimus=NVIDIA_only ros2 run mujoco_sim view_ros
```

它订阅 `/fmu/out/vehicle_odometry`（PX4 EKF 位姿，BEST_EFFORT QoS），把 NED 位姿
转回 MuJoCo 世界 FLU 并渲染到场景里。

## 只看模型（不接 PX4）

```bash
./launch/view_only.sh                 # 镜头跟随 x500
./launch/view_only.sh --no-track      # 不跟随
./launch/view_only.sh --speed 0.5     # 慢放
```

`view.py` 是独立查看器（有自己的物理 + 电机滑块自测），用于检查模型。

## 物理参数

| 项 | 值 | 来源 |
|---|---|---|
| 机体质量 | 2.0 kg（总 2.0643 kg） | PX4 x500_base/model.sdf |
| 机体惯量 | 0.02167 / 0.02167 / 0.04 | 同上 |
| 推力系数 KF | 8.54858e-06 N/(rad/s)² | PX4 motorConstant |
| 最大转速 | 1000 rad/s | maxRotVelocity |
| 反扭矩系数 KM | 1.71e-07（= KF×0.02） | 物理修正值 |
| 碰撞 | 外壳网格凸包（视觉面=碰撞面） | 自动生成 |
| 场地碰撞 | 4383 个 box，5 cm 栅格 | 高度图贪心合并 |
| 时间步 | 0.002 s（物理 500Hz，IMU 250Hz） | mujoco_bridge |

## 重建世界

```bash
./tools/build.sh --drone        # 场地 + 无人机
./tools/build.sh                # 只重建场地
```

源世界在 `~/one_drone/worlds/rmuc_2025_3m_vio_columns.sdf`，PX4 模型只读自
`/data/DroneNav/PX4-Autopilot/Tools/simulation/gz/models/x500_base`。
