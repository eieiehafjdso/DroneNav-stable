# recovery

对偶四元数碰撞恢复控制器（论文阶段 4），**仿真/真机通用**，常驻运行。

- 检测碰撞（接触穿透深度超阈值）→ 用论文 Eq.22~23a 计算导纳让步参考；
- 每个控制周期用 Eq.24 几何控制算期望力 → 转成 PX4 姿态+推力 setpoint；
- 收敛后停止接管，交还飞行节点。

## 目录

```
recovery/
├── scripts/collision_recovery_node.py   # 常驻恢复节点（唯一入口）
├── dq_lib/                              # 对偶四元数参考实现（dual_quaternion / reset_map / recovery_controller）
├── config/recovery_params.yaml          # ★ 所有关键参数都在这里改
├── launch/recovery.launch.py            # 启动（加载 params）
├── CMakeLists.txt / package.xml
```

## 运行

```bash
# 1. 编译
cd /data/DroneNav/src
colcon build --packages-select recovery

# 2. 启动（会加载 config/recovery_params.yaml）
ros2 launch recovery recovery.launch.py
# 或指定参数文件
ros2 launch recovery recovery.launch.py params_file:=$(pwd)/recovery/config/recovery_params.yaml
```

## 话题

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `/fmu/out/vehicle_odometry` | px4_msgs/VehicleOdometry |
| 订阅 | `/fmu/out/vehicle_status_v1` | px4_msgs/VehicleStatus |
| 订阅 | `/mujoco/contacts` | std_msgs/Float64MultiArray（仿真接触源） |
| 发布 | `/recovery/setpoint` | px4_msgs/VehicleAttitudeSetpoint（仅恢复期间） |
| 发布 | `/recovery/status` | std_msgs/String |

话题名都是参数（`config/recovery_params.yaml` 里 `*_topic`），**真机**把
`contact_topic` 换成你自己的碰撞/接触源即可，其余逻辑不变。

## 与飞行节点的约定

恢复节点不负责正常飞行。飞行节点（如 `drone_teleop` 的键盘节点）订阅
`/recovery/setpoint`，在收到新鲜 setpoint 的期间转发给 PX4（接管），恢复结束
（节点停发）后飞行节点恢复自己的控制。
