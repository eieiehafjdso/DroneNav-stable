# drone_teleop

PX4 offboard **键盘手动遥控**（仅此一个节点，职责单一）。

> 碰撞恢复已拆到独立包 [`recovery`](../recovery/)（仿真/真机通用，参数可配）。
> 本包只负责键盘遥控，碰撞恢复期间转发 `/recovery/setpoint` 到 PX4。

## 节点

* `drone_keyboard_teleop`（C++）—— 终端键盘遥控：
  * `W/S` 前后，`A/D` 左右，`R/F` 上下，`Q/E` 左右旋；
  * `T` 切 OFFBOARD、解锁并起飞；`L` 进入 PX4 降落并以约 0.5 m/s 本地高度目标辅助下降，在落地后上锁；`O` 仅切 OFFBOARD（不会解锁或起飞）；`X` 上锁；空格悬停；
  * 起飞、悬停、移动和降落通过 `/fmu/in/trajectory_setpoint` 发布位置和航向指令，由 PX4 原生控制器稳定姿态；
  * 订阅 `/recovery/setpoint`，新鲜（<0.2s）时转发该 setpoint（碰撞恢复接管），
    停发后恢复自己的键盘控制。

## 依赖

ROS2 Humble、Eigen3、`px4_msgs`（release/1.17）、`std_msgs`。

## 编译

```bash
cd /data/DroneNav/src
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs drone_teleop recovery
source /data/DroneNav/install/setup.bash
```

## 运行

```bash
# 1) 仿真 + Agent
ros2 launch mujoco_sim rmuc_sim.launch.py        # 起 MuJoCo + Agent + PX4

# 2) 碰撞恢复（常驻）
ros2 launch recovery recovery.launch.py

# 3) 键盘遥控（终端里操作）
ros2 launch drone_teleop teleop.launch.py
```

## 话题

| 方向 | 话题 | 类型 |
|---|---|---|
| 发布 | `/fmu/in/offboard_control_mode` | px4_msgs/OffboardControlMode |
| 发布 | `/fmu/in/vehicle_attitude_setpoint_v1` | px4_msgs/VehicleAttitudeSetpoint |
| 发布 | `/fmu/in/vehicle_command` | px4_msgs/VehicleCommand |
| 订阅 | `/fmu/out/vehicle_odometry` | px4_msgs/VehicleOdometry |
| 订阅 | `/fmu/out/vehicle_status_v1` | px4_msgs/VehicleStatus |
| 订阅 | `/recovery/setpoint` | px4_msgs/VehicleAttitudeSetpoint（碰撞恢复接管） |
