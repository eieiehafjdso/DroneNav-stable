# DroneNav

ROS2 Humble + PX4 SITL + MuJoCo HIL 的无人机键盘遥控与对偶四元数碰撞恢复。

> **当前稳定基线：2026-11-21 20:16（倒数第二个版本）**
>
> 这一版是实际能用的版本：键盘负责正常飞行和把 recovery 的输出转发给 PX4；
> `recovery` 只在**真正恢复期间**发布 `/recovery/*`，空闲时不发 PX4 心跳、
> 不切 OFFBOARD，因此把它加到其他飞行节点旁边也不会影响别人。
>
> 后来的 20:38 版本把 recovery 改成“直接与 PX4 握手”，会无条件抢 OFFBOARD，
> 不作为基线使用。历史快照备份在：
>
> - `build/reconstruct_2016/snap/` —— 20:16 精确快照（本次恢复所用）
> - `build/backup_1616_mixed/` —— 16:16 姿态版
> - `build/backup_split_20261121/` —— 20:38 拆分/直接 PX4 版

---

## 1. 特性

- MuJoCo 独立进程做物理仿真，通过 MAVLink HIL 与 PX4 SITL 闭环；
- MuJoCo 时间作为唯一时间源，桥以 50 Hz 发布 `/clock`；
- 键盘节点通过 PX4 Offboard 位置控制飞行，起飞/悬停/移动/降落；
- 对偶四元数碰撞恢复（论文 Eq.22~24）：接触检测 → 导纳让步 → 几何控制；
- recovery 与键盘解耦：recovery 只产出 `/recovery/*`，键盘在收到新鲜 setpoint 时
  转发给 PX4；recovery 空闲时完全不碰 PX4；
- 墙世界、接触 relay、恢复状态监视、键盘接管保护。

---

## 2. 系统架构

```text
                    ┌──────────────────────────────┐
                    │        keyboard_teleop       │
                    │  位置控制 / 起飞 / 降落       │
                    └───────┬───────────────▲──────┘
        /fmu/in/*           │               │  /recovery/*
        offboard/trajectory │               │  setpoint / thrust / torque
                            ▼               │
                    ┌──────────────┐        │
                    │   PX4 SITL   │        │
                    └───┬──────▲───┘        │
       HIL_ACTUATOR_... │      │ IMU/EV/GPS │
                        ▼      │             │
                    ┌──────────────┐        │
                    │  px4_bridge  │        │
                    │  MuJoCo 物理  │        │
                    └───┬──────▲───┘        │
          UDP 14577     │      │ /clock      │
                        ▼      │             │
                    ┌──────────────┐        │
                    │ contact_relay│        │
                    └──────┬───────┘        │
                           │ /mujoco/contacts
                           ▼                │
                    ┌──────────────────────────────┐
                    │     collision_recovery       │
                    │  只在 recovering 时发 /recovery/* │
                    └──────────────────────────────┘
```

- **正常飞行**：键盘 → `/fmu/in/trajectory_setpoint` + heartbeat → PX4；
- **碰撞恢复**：`contact_relay` → `/mujoco/contacts` → recovery 计算 →
  `/recovery/setpoint` + `/recovery/thrust_setpoint` + `/recovery/torque_setpoint`
  → 键盘转发到 PX4；
- 恢复期间键盘暂停按键，恢复结束后自动交还。

---

## 3. 包结构

```text
src/
├── bringup/            # 一键启动仿真闭环（bridge + Agent + PX4 + relay + recovery）
├── mujoco_sim/         # MuJoCo 桥、接触 relay、世界模型
├── drone_teleop/       # 键盘遥控节点 + DQ 数学头文件 + 单元测试
├── recovery/           # 对偶四元数碰撞恢复节点（非侵入式）
├── px4_msgs/           # PX4 uORB 消息定义
└── ego-planner/        # 规划器（当前键盘流程不依赖）
```

> 旧的 `src/launch` 骨架包已删除，统一用 `src/bringup` 作为总启动入口。

---

## 4. 环境依赖

| 组件 | 版本/路径 | 说明 |
|---|---|---|
| Ubuntu | 22.04 | — |
| ROS2 | Humble | 需先 `source /opt/ros/humble/setup.bash` |
| PX4-Autopilot | v1.17.0 | 已编译到 `PX4-Autopilot/build/px4_sitl_default` |
| Micro-XRCE-DDS-Agent | `/usr/local/bin/MicroXRCEAgent` | uXRCE-DDS 桥，UDP 8888 |
| MuJoCo | `/data/MuJoCo/MuJoCo-drones-gym/.venv` | `px4_bridge` 的 Python 环境 |
| QGroundControl | 可选 | 只用于地面站观察，不参与键盘链路 |

从零安装 PX4/QGC/Qt/Agent 的旧步骤见 git 历史中的 `README.md`：

```bash
git show HEAD:README.md
```

> 本文件面向已经装好环境的 `/data/DroneNav` 工作区。

---

## 5. 编译

```bash
cd /data/DroneNav
source /opt/ros/humble/setup.bash

colcon build --packages-select px4_msgs drone_teleop recovery mujoco_sim bringup

source install/setup.bash
```

只改键盘时：

```bash
colcon build --packages-select drone_teleop
source install/setup.bash
```

只改 recovery 时：

```bash
colcon build --packages-select recovery
source install/setup.bash
```

---

## 6. 运行

### 终端 1：仿真闭环

```bash
cd /data/DroneNav
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bringup sim.launch.py
```

会依次启动 MuJoCo 桥、MicroXRCEAgent、PX4 SITL、接触 relay、recovery。
等约 10 s，直到 PX4 打印 `Ready for takeoff!` / 控制台不再刷初始化日志。

可选参数：

```bash
# 不开 MuJoCo 窗口
ros2 launch bringup sim.launch.py gui:=false

# 有墙的世界（撞墙恢复测试）
ros2 launch bringup sim.launch.py \
  world:=/data/DroneNav/src/mujoco_sim/rmuc_2025_drone_wall.xml

# 只起 ROS 侧（Agent + relay + recovery，不启 PX4）
ros2 launch bringup sim.launch.py px4:=false
```

> `mujoco_sim/rmuc_sim.launch.py` 也能启动同样的闭环；二选一，不要同时启动。

### 终端 2：键盘遥控

**必须在真实终端里运行**，不能放在 VSCode 任务、后台 `&`、`nohup` 或重定向里：

```bash
cd /data/DroneNav
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drone_teleop teleop.launch.py
```

这个版本的键盘节点用 ROS 时间（仿真 `/clock`）做超时和消息时间戳，所以
**请通过 `teleop.launch.py` 启动**（它设置 `use_sim_time:=true`）。
如果直接 `ros2 run`，必须自己带上：

```bash
ros2 run drone_teleop drone_keyboard_teleop --ros-args \
  --params-file src/drone_teleop/config/teleop_params.yaml \
  -p use_sim_time:=true
```

---

## 7. 键盘操作

| 按键 | 作用 |
|---|---|
| `t` | 切 OFFBOARD + 解锁 + 起飞到 1.2 m |
| `l` | 降落（PX4 AUTO_LAND + 本地下降辅助，接近地面后上锁） |
| `o` | 只切 OFFBOARD，不解锁、不起飞 |
| `x` | 立即上锁（空中会掉高） |
| 空格 | 悬停 |
| `w` / `s` | 前进 / 后退 |
| `a` / `d` | 左移 / 右移 |
| `r` / `f` | 上升 / 下降 |
| `q` / `e` | 左转 / 右转 |
| Esc / Ctrl-C | 退出并恢复终端 |

- 按住方向键连续移动，松手约 0.35 s 后自动悬停；
- 碰撞恢复接管期间按键会被忽略，恢复结束后自动恢复；
- 起飞前请确认 PX4 `pre_flight_checks_pass` 为 true。

---

## 8. 碰撞恢复

recovery 常驻运行，但**只有 `recovering=true` 时才发布**：

| 话题 | 类型 | 说明 |
|---|---|---|
| `/recovery/setpoint` | `px4_msgs/VehicleAttitudeSetpoint` | 姿态 + 推力 |
| `/recovery/thrust_setpoint` | `px4_msgs/VehicleThrustSetpoint` | 直接推力 |
| `/recovery/torque_setpoint` | `px4_msgs/VehicleTorqueSetpoint` | 直接力矩 |
| `/recovery/status` | `std_msgs/String` | `RECOVER started` / `RECOVER done` / 进度 |

键盘节点在 setpoint 新鲜（<0.2 s）时转发给 PX4：
- thrust/torque 都新鲜 → heartbeat 用 `direct_actuator`；
- 只有姿态 setpoint 新鲜 → heartbeat 用 `attitude`；
- 否则 → 正常 `position` 控制。

recovery 空闲时不发布 PX4 心跳、不发 `DO_SET_MODE`，所以：

> 把 recovery 加到其他飞行节点旁边，不会改变别人的 OFFBOARD 状态。

### 撞墙测试

```bash
ros2 launch bringup sim.launch.py \
  world:=/data/DroneNav/src/mujoco_sim/rmuc_2025_drone_wall.xml
```

起飞后向墙飞，接触深度超过阈值（默认 `1e-4 m`）会触发恢复；约 1~4 s 后
`RECOVER done`，键盘重新接管。

### 参数

- 键盘：`src/drone_teleop/config/teleop_params.yaml`
- 恢复：`src/recovery/config/recovery_params.yaml`（`k_q=2.0`、`k_p=2.0`、
  `admittance_factor=0.08`、`max_recover_time=4.0` 等）

---

## 9. 主要话题

| 方向 | 话题 | 类型 |
|---|---|---|
| 发布 | `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` |
| 发布 | `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` |
| 发布 | `/fmu/in/vehicle_attitude_setpoint_v1` | `px4_msgs/VehicleAttitudeSetpoint` |
| 发布 | `/fmu/in/vehicle_thrust_setpoint` | `px4_msgs/VehicleThrustSetpoint` |
| 发布 | `/fmu/in/vehicle_torque_setpoint` | `px4_msgs/VehicleTorqueSetpoint` |
| 发布 | `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` |
| 发布 | `/drone_teleop/keyboard_status` | `std_msgs/String` |
| 订阅 | `/fmu/out/vehicle_odometry` | `px4_msgs/VehicleOdometry` |
| 订阅 | `/fmu/out/vehicle_status_v1` | `px4_msgs/VehicleStatus` |
| 订阅 | `/fmu/out/vehicle_land_detected` | `px4_msgs/VehicleLandDetected` |
| 订阅 | `/recovery/setpoint` | `px4_msgs/VehicleAttitudeSetpoint` |
| 订阅 | `/recovery/thrust_setpoint` | `px4_msgs/VehicleThrustSetpoint` |
| 订阅 | `/recovery/torque_setpoint` | `px4_msgs/VehicleTorqueSetpoint` |
| 订阅 | `/recovery/status` | `std_msgs/String` |
| 订阅 | `/mujoco/reset` | `std_msgs/String` |

> PX4 的 uXRCE-DDS 会给带 `MESSAGE_VERSION` 的话题加 `_v1` 后缀；
> 本环境实际是 `/fmu/out/vehicle_status_v1`，以 `ros2 topic list` 为准。

---

## 10. 测试

```bash
cd /data/DroneNav
source /opt/ros/humble/setup.bash
source install/setup.bash

# DQ 数学/恢复单元测试（9 个）
colcon test --packages-select drone_teleop --event-handlers console_direct+

# Python 语法检查
python3 -m py_compile src/recovery/scripts/collision_recovery_node.py
```

运行时可快速检查：

```bash
ros2 topic list | grep -E "fmu|recovery|mujoco|clock"
ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic echo /fmu/out/vehicle_status_v1 --once
```

---

## 11. 故障排查

### 键盘没反应 / `stdin 不是终端`

节点必须在真实 TTY 里运行。不要用 VSCode 任务、`nohup`、`&`、重定向启动。
用 `ros2 launch drone_teleop teleop.launch.py` 或在一个正常终端里 `ros2 run`。

### `未收到 PX4 状态`

```bash
ros2 topic list | grep fmu
```

应看到 `/fmu/out/vehicle_status_v1`、`/fmu/out/vehicle_odometry`。
没有的话检查：

1. `MicroXRCEAgent` 是否在跑，端口是否 8888；
2. PX4 是否先于 Agent 启动、`UXRCE_DDS_SYNCT` 是否为 0；
3. PX4 控制台是否有 EKF/传感器错误。

### `等待里程计`

PX4 EKF 还没拿到有效 local position；等几秒，或检查：

```bash
ros2 topic echo /fmu/out/vehicle_odometry --once
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
```

### 按 `t` 不起飞

```bash
ros2 topic echo /fmu/out/vehicle_status_v1 --once
```

重点看 `arming_state`、`pre_flight_checks_pass`、`nav_state`。
如果 preflight 不通过，先排查 EKF/传感器；`force_arm:=true` 只建议在仿真里用。

### 降落后桨叶不停止

MuJoCo HIL 的 land detector 有时不置 `landed`。本版键盘在
“接近起飞地面 + 速度足够小”时会上锁；如果仍未停，落地后按 `x` 上锁，
或检查：

```bash
ros2 topic echo /fmu/out/vehicle_land_detected --once
```

### 恢复不触发

- 确认用的是有墙世界：`world:=.../rmuc_2025_drone_wall.xml`；
- 确认 `contact_relay` 在跑、`/mujoco/contacts` 有数据；
- 确认 PX4 已解锁且高度 > `min_height`（0.4 m）。

### recovery 空闲时在抢 OFFBOARD

说明你跑到了 20:38 那个“recovery 直接与 PX4 握手”的版本。
本稳定版的 recovery 空闲时不会发 PX4 任何东西；用
`build/reconstruct_2016/snap/` 恢复，或检查
`src/recovery/scripts/collision_recovery_node.py` 里是否出现
`ensure_offboard` / `OffboardControlMode`。

### 进程残留 / 端口占用

```bash
pkill -9 -x px4
pkill -9 -x MicroXRCEAgent
fuser -k 4560/tcp 2>/dev/null || true
fuser -k 8888/udp 2>/dev/null || true
fuser -k 14577/udp 2>/dev/null || true
```

---

## 12. 版本与备份

- **稳定基线**：2026-11-21 20:16（倒数第二个版本，键盘 651 行 + recovery 434 行）
- `build/reconstruct_2016/snap/`：本次恢复的精确快照；
- `build/backup_1616_mixed/`：16:16 姿态+推力版；
- `build/backup_split_20261121/`：20:38 拆分/直接 PX4 版（不作为基线）。

> 注意：`build/` 已被 `.gitignore` 忽略，以上备份只在本机；长期保存请提交当前源码，
> 或把快照复制到非 `build/` 目录。
>
> 后续如果还要改，建议先 `git commit` 或复制一份 `src/drone_teleop`/`src/recovery`，
> 再让 AI 做小步修改。这个项目已经被整体重写破坏过一次，**不要直接让它重写整个键盘节点**。
