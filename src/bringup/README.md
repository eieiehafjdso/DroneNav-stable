# bringup

一键启动仿真闭环：**MuJoCo 独立进程（MAVLink HIL） + PX4 SITL + uXRCE-DDS Agent + 接触 relay + 碰撞恢复**。

架构（和 Gazebo 一致）：

```
键盘(offboard) ──> PX4 ──电机指令──> px4_bridge(MuJoCo 物理+桨叶)
                     ↑                      │
                     └──IMU/EV/GPS(HIL)─────┘
                     │
                  /clock ──> 所有 ROS2 节点(use_sim_time=true)
MuJoCo 接触 ──UDP──> contact_relay ──> /mujoco/contacts ──> recovery
```

## 启动（两个终端）

**终端 1 —— 仿真主体**

```bash
cd /data/DroneNav
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bringup sim.launch.py
```

会打开 MuJoCo 窗口（独立显卡渲染），镜头实时跟随无人机，中心顶灯闪烁。

**终端 2 —— 键盘遥控（必须用这个，不能直接 ros2 run）**

```bash
cd /data/DroneNav
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drone_teleop teleop.launch.py
```

> ⚠️ 为什么必须用 `teleop.launch.py`：它带 `use_sim_time:=true`。
> 键盘节点把 heartbeat/命令的时间戳设为 `nowUs()`。如果不用仿真时间，
> `nowUs()` 返回墙钟（约 1.7e15 µs），而 PX4 的 `hrt` 是 MuJoCo 时间
> （约 40e6 µs），相差 9 个数量级 → PX4 判定 offboard 信号丢失 →
> failsafe → Land + Disarm。

## 操作顺序

1. 终端 1 起来后等 PX4 打印 `Ready for takeoff!`（约 10s）
2. 终端 2 按 `o` 解锁（切 OFFBOARD + ARM）
3. 按 `t` 起飞到 1.2m
4. `w/s/a/d` 平移、`r/f` 升降、`q/e` 转向、空格悬停
5. 撞到墙/障碍后恢复节点自动接管，约 1~4s 后交还

> `auto_start` 默认已关闭。**不要**开自动解锁——贴地状态下强制进入
> offboard 姿态控制会让姿态环超调，导致地面上就翻车。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `world` | `rmuc_2025_drone.xml` | MuJoCo 世界（**默认无墙**，天空正常） |
| `px4_dir` | PX4 build 目录 | PX4 SITL 位置 |
| `agent_port` | 8888 | MicroXRCEAgent UDP 端口 |
| `px4_delay` | 3.0 | PX4/recovery 相对桥的启动延时(秒) |
| `gui` | true | 是否开 MuJoCo 窗口 |
| `px4` | true | 是否启动 PX4 SITL |

### 可选撞击目标（墙）

默认世界没有墙。需要撞墙测试时显式指定：

```bash
ros2 launch bringup sim.launch.py \
  world:=/data/DroneNav/src/mujoco_sim/rmuc_2025_drone_wall.xml
```

墙的几何单独放在 `src/mujoco_sim/wall_obstacle.xml`，通过 MuJoCo `<include>`
组合，不污染默认世界模型（`rmuc_2025_drone.xml` 保持 4407 geoms / 蓝天）。

### 无画面（真机 / 无显示器）

```bash
ros2 launch bringup sim.launch.py gui:=false
```

### 只起 ROS 侧（真机：Agent + relay + recovery，无 PX4 SITL）

```bash
ros2 launch bringup sim.launch.py px4:=false
```

## 参数文件

- 恢复节点增益：**`src/recovery/config/recovery_params.yaml`**（不在 MuJoCo XML 里）
  * `k_q`/`k_p`/`k_dw`/`k_dv` —— 几何控制增益 (Eq.24)
  * `alpha`/`admittance_factor` —— 导纳让步 (Eq.22~23a)
  * `max_admittance_shift` —— 让步位移上限（防弹飞）
  * `contact_depth_threshold`/`contact_timeout` —— 触发判定

## 时间源

- **MuJoCo 是唯一时间源**：`px4_bridge` 主循环用 `while d.time < wall_target`
  把物理时间追到墙钟，严格 1:1。
- **PX4**：桥每 4ms 发 `HIL_SENSOR.time_usec = d.time`，PX4 的
  `simulator_mavlink` 立刻 `px4_clock_settime()` 覆写自己的时钟（并锁步推进），
  所以 PX4 不会漂移、也不会因启动顺序而超前。
- **ROS2**：桥以 50Hz 发 `/clock`，所有节点必须 `use_sim_time:=true`。
  `sim.launch.py` 已给 relay/recovery 设置；键盘用 `teleop.launch.py`。

## 真机部署要点

1. 把 `px4_bridge` 换成真机传感器/电机接口，`/clock` 换成系统时间
2. 所有节点 `use_sim_time:=false`
3. 接触源 `contact_topic` 换成真机碰撞检测
4. 恢复节点其余逻辑不变（只依赖 `/fmu/out/vehicle_odometry` + 接触 + offboard）
