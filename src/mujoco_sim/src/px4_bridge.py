#!/data/MuJoCo/MuJoCo-drones-gym/.venv/bin/python
"""PX4 SITL ↔ MuJoCo 桥（正确架构：MuJoCo 独立进程 + MAVLink HIL）

和 Gazebo 一模一样：
  * MuJoCo 在本进程跑物理 + 开窗口（走独立显卡渲染）
  * 通过 MAVLink HIL 把 IMU/GPS/状态 发给 PX4
  * PX4 跑控制器，回传 HIL_ACTUATOR_CONTROLS（4 个电机指令）
  * 本进程按电机指令驱动 MuJoCo 桨叶 + 施加推力/反扭矩

PX4 侧用自带 simulator_mavlink（不是进程内模块）：
    PX4_SIMULATOR 不设(或=mavlinksim) → px4-rc.mavlinksim → simulator_mavlink start -c 4560

    ros2 run mujoco_sim px4_bridge --xml ... --body x500
"""
import argparse
import math
import os
import struct
import sys
import time

import socket
import struct

import numpy as np
import mujoco
import mujoco.viewer

os.environ.setdefault('MAVLINK20', '1')
from pymavlink import mavutil

# ROS2：把 MuJoCo 时间发成 /clock（让 ROS2 节点 use_sim_time=true 时共用同一时间源）
try:
    import rclpy
    from rclpy.node import Node as RosNode
    from rosgraph_msgs.msg import Clock as RosClock
    from std_msgs.msg import String as RosString
    _HAS_ROS = True
except Exception:
    _HAS_ROS = False

# ---------- 电机模型（与最初 view.py 一致） ----------
KF = 8.54858e-06
KM = 1.71e-07
MAX_OMEGA = 1000.0
ROTOR_POS = np.array([[0.174, -0.174, 0.06],
                      [-0.174, 0.174, 0.06],
                      [0.174, 0.174, 0.06],
                      [-0.174, -0.174, 0.06]])
DIRS = np.array([-1.0, -1.0, +1.0, +1.0])

# 起降点 GPS（和 PX4 simulator_mavlink 默认一致）
GPS_LAT = 47.397742
GPS_LON = 8.545594
GPS_ALT = 488.0

# 碰撞接触回传（UDP 14577 -> /mujoco/contacts, 和 contact_relay 对接）
CONTACT_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
CONTACT_ADDR = ("127.0.0.1", 14577)
MAX_PENETRATION = {"depth": 0.0, "geom": "", "pos": (0,0,0), "t": 0.0}



def apply_motor_model(m, d, body_id, controls):
    """controls[4]：PX4 HIL 电机指令 0..1 → 桨转速 → 推力/反扭矩"""
    normalized = np.clip(np.asarray(controls, float), 0.0, 1.0)
    omega = np.sqrt(normalized) * MAX_OMEGA
    R = d.xmat[body_id].reshape(3, 3)
    axis = R[:, 2]
    F = np.zeros(3)
    T = np.zeros(3)
    for i in range(4):
        # 线性推力（匹配 PX4 控制分配器线性假设）
        f = KF * omega[i] ** 2
        force = axis * f
        F += force
        T += np.cross(R @ ROTOR_POS[i], force)
        T += axis * (KM * omega[i] ** 2 * DIRS[i])
    d.xfrc_applied[body_id, :3] = F
    d.xfrc_applied[body_id, 3:] = T
    # 桨叶真转
    d.ctrl[:4] = DIRS * omega


def publish_contacts(m, d, body_id):
    """把 MuJoCo 里最深接触发给恢复节点（12 floats）"""
    if d.ncon == 0:
        return
    best = None
    best_idx = 0
    best_depth = 0.0
    for i in range(d.ncon):
        c = d.contact[i]
        if c.dist < best_depth:
            best_depth = c.dist
            best = c
            best_idx = i
    if best is None:
        return
    # 接触位置（世界）
    pos_mujoco = np.array(best.pos, dtype=float)
    # 接触数据统一成 Recovery 内部 ENU：NED/FRD 到 ENU/FLU 的世界轴旋转。
    pos = np.array([-pos_mujoco[1], pos_mujoco[0], pos_mujoco[2]], dtype=float)
    # 法向
    nrm_mujoco = np.array(best.frame[:3], dtype=float)
    nrm = np.array([-nrm_mujoco[1], nrm_mujoco[0], nrm_mujoco[2]], dtype=float)
    # 接触力
    f = np.zeros(6)
    mujoco.mj_contactForce(m, d, best_idx, f)
    mag = float(f[0])
    body_geoms = [i for i in range(m.ngeom) if m.geom_bodyid[i] == body_id]
    other = best.geom2 if best.geom1 in body_geoms else best.geom1
    gname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, other) or f"geom{other}"
    mu = float(best.mu)
    # 接触点速度
    vel = np.zeros(6)
    mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, body_id, vel, 0)
    vel_mujoco = vel[3:6]
    vel_enu = np.array([-vel_mujoco[1], vel_mujoco[0], vel_mujoco[2]], dtype=float)
    buf = struct.pack("12d", 1.0, pos[0], pos[1], pos[2], nrm[0], nrm[1], nrm[2],
                      mu, float(-best_depth), vel_enu[0], vel_enu[1], vel_enu[2])   # 穿透深度取正
    CONTACT_SOCK.sendto(buf, CONTACT_ADDR)
    pen = -best_depth
    if pen > MAX_PENETRATION["depth"]:
        MAX_PENETRATION["depth"] = pen
        MAX_PENETRATION["geom"] = gname
        MAX_PENETRATION["pos"] = (pos[0], pos[1], pos[2])
        MAX_PENETRATION["t"] = d.time
    if not hasattr(publish_contacts, "_logged"):
        publish_contacts._logged = True
        print(f"[bridge] 首次接触: {gname} depth={best_depth:.3f} mu={mu:.2f}", flush=True)


def q_to_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    pitch = math.asin(max(-1.0, min(1.0, 2*(w*y - z*x))))
    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    return roll, pitch, yaw



def _R_to_quat(R):
    tr = np.trace(R)
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        return (0.25*S, (R[2,1]-R[1,2])/S, (R[0,2]-R[2,0])/S, (R[1,0]-R[0,1])/S)
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        return ((R[2,1]-R[1,2])/S, 0.25*S, (R[0,1]+R[1,0])/S, (R[0,2]+R[2,0])/S)
    elif R[1,1] > R[2,2]:
        S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        return ((R[0,2]-R[2,0])/S, (R[0,1]+R[1,0])/S, 0.25*S, (R[1,2]+R[2,1])/S)
    else:
        S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        return ((R[1,0]-R[0,1])/S, (R[0,2]+R[2,0])/S, (R[1,2]+R[2,1])/S, 0.25*S)


def publish_hil(conn, m, d, body_id, time_usec, controls):
    """250Hz IMU（比力 + 角速度，FRD）"""
    pos = d.xpos[body_id]
    R = d.xmat[body_id].reshape(3, 3)
    ax = d.qacc[0]; ay = d.qacc[1]; az = d.qacc[2] + 9.81
    f_flu = R.T @ np.array([ax, ay, az])
    xacc = float(f_flu[0]); yacc = float(-f_flu[1]); zacc = float(-f_flu[2])
    res = np.zeros(6)
    mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, body_id, res, 1)
    wx = float(res[0]); wy = float(-res[1]); wz = float(-res[2])
    conn.mav.hil_sensor_send(
        time_usec=int(time_usec),
        xacc=xacc, yacc=yacc, zacc=zacc,
        xgyro=wx, ygyro=wy, zgyro=wz,
        xmag=0.2, ymag=-0.1, zmag=0.4,
        abs_pressure=101325.0 + 100.0 * pos[2],
        diff_pressure=0.0,
        pressure_alt=GPS_ALT + pos[2],
        temperature=20.0,
        fields_updated=0x1FFF,
    )


def publish_odom(conn, m, d, body_id, time_usec, reset_counter=0):
    """50Hz EV(ODOMETRY) + 真值 + GPS"""
    pos = d.xpos[body_id]
    R = d.xmat[body_id].reshape(3, 3)
    # 和 module 一致：body FRD -> NED，列 = 机体 FRD 轴在 NED 下的分量
    xb = R[:, 0]; yb = R[:, 1]; zb = R[:, 2]
    R_b2n = np.array([
        [ xb[0], -yb[0], -zb[0]],
        [-xb[1],  yb[1],  zb[1]],
        [-xb[2],  yb[2],  zb[2]],
    ])
    oqw, oqx, oqy, oqz = _R_to_quat(R_b2n)
    p_ned = (pos[0], -pos[1], -pos[2])
    v_ned = (d.qvel[0], -d.qvel[1], -d.qvel[2])
    res = np.zeros(6)
    mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, body_id, res, 1)
    wx, wy, wz = float(res[0]), float(-res[1]), float(-res[2])
    conn.mav.odometry_send(
        time_usec=int(time_usec),
        frame_id=1, child_frame_id=20,
        x=float(p_ned[0]), y=float(p_ned[1]), z=float(p_ned[2]),
        q=[oqw, oqx, oqy, oqz],
        vx=float(v_ned[0]), vy=float(v_ned[1]), vz=float(v_ned[2]),
        rollspeed=wx, pitchspeed=wy, yawspeed=wz,
        pose_covariance=[0.01]*21, velocity_covariance=[0.01]*21,
        reset_counter=reset_counter, estimator_type=3,
    )
    conn.mav.hil_state_quaternion_send(
        time_usec=int(time_usec),
        attitude_quaternion=[oqw, oqx, oqy, oqz],
        rollspeed=wx, pitchspeed=wy, yawspeed=wz,
        lat=int(GPS_LAT * 1e7), lon=int(GPS_LON * 1e7),
        alt=int((GPS_ALT - p_ned[2]) * 1000),
        vx=int(v_ned[0]*100), vy=int(v_ned[1]*100), vz=int(v_ned[2]*100),
        ind_airspeed=0, true_airspeed=0,
        xacc=0, yacc=0, zacc=0,
    )
    conn.mav.hil_gps_send(
        time_usec=int(time_usec), fix_type=3,
        lat=int(GPS_LAT * 1e7), lon=int(GPS_LON * 1e7),
        alt=int((GPS_ALT - p_ned[2]) * 1000),
        eph=10, epv=10,
        vel=int(math.hypot(v_ned[0], v_ned[1]) * 100),
        vn=int(v_ned[0]*100), ve=int(v_ned[1]*100), vd=int(v_ned[2]*100),
        cog=0, satellites_visible=10,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default="/data/DroneNav/src/mujoco_sim/rmuc_2025_drone.xml")
    ap.add_argument("--body", default="x500")
    ap.add_argument("--port", type=int, default=4560, help="PX4 simulator_mavlink TCP 端口")
    ap.add_argument("--dist", type=float, default=2.5)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--track", dest="track", action="store_true", default=True,
                    help="镜头实时跟随无人机（默认开）")
    ap.add_argument("--no-track", dest="track", action="store_false")
    ap.add_argument("--blink", type=float, default=0.6, help="中心顶灯闪烁周期(s)，0=不闪")
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(args.xml)
    d = mujoco.MjData(m)
    body_id = m.body(args.body).id

    # MuJoCo viewer（独立显卡渲染）
    viewer = None
    if not args.headless:
        viewer = mujoco.viewer.launch_passive(m, d)
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.distance = args.dist
        viewer.cam.elevation = -20
        viewer.cam.azimuth = 90
        print(f"[px4_bridge] viewer: 镜头跟随={args.track}, 灯光闪烁周期={args.blink}s", flush=True)

    # ROS2 /clock：MuJoCo 时间是唯一时间源，ROS2 节点 use_sim_time=true 即可共用
    clock_pub = None
    reset_pub = None
    reset_sub = None
    reset_requested = [False]
    ros_node = None
    if _HAS_ROS:
        try:
            rclpy.init(args=None)
            ros_node = RosNode("mujoco_clock")
            clock_pub = ros_node.create_publisher(RosClock, "/clock", 10)
            reset_pub = ros_node.create_publisher(RosString, "/mujoco/reset", 10)
            def request_reset(_msg):
                reset_requested[0] = True
            reset_sub = ros_node.create_subscription(RosString, "/mujoco/reset_request", request_reset, 10)
            print("[px4_bridge] /clock 和 /mujoco/reset 已启动", flush=True)
        except Exception as e:
            print(f"[px4_bridge] rclpy 初始化失败，/clock 不可用: {e}", flush=True)
            clock_pub = None

    # MAVLink HIL：监听 PX4 连接
    print(f"[px4_bridge] 监听 TCP {args.port}，等待 PX4 simulator_mavlink 连接...", flush=True)
    conn = mavutil.mavlink_connection(f"tcpin:0.0.0.0:{args.port}", robust_parsing=True)
    print("[px4_bridge] PX4 已连接", flush=True)

    dt = m.opt.timestep
    t_start = time.time()
    _last_render = 0.0
    _LAMP_ID = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_LIGHT, "center_lamp")
    if _LAMP_ID < 0:
        _LAMP_ID = 0
    _LAMP_BASE = np.array(m.light_diffuse[_LAMP_ID], dtype=float).copy()
    _LAMP_SPEC = np.array(m.light_specular[_LAMP_ID], dtype=float).copy()
    if args.blink > 0 and viewer is not None:
        print(f"[px4_bridge] 中心灯 id={_LAMP_ID} 基准 diffuse={_LAMP_BASE}", flush=True)
    step = 0
    last_controls = [0.0, 0.0, 0.0, 0.0]
    sim_time_us = 0
    clock_drift_max = 0.0
    clock_drift_last = 0.0
    last_d_time = d.time
    reset_counter = 0

    try:
        while True:
            if viewer is not None and not viewer.is_running():
                break

            if ros_node is not None:
                rclpy.spin_once(ros_node, timeout_sec=0)

            if reset_requested[0]:
                reset_requested[0] = False
                mujoco.mj_resetData(m, d)
                free_adr = m.jnt_qposadr[m.body_jntadr[body_id]]
                d.qpos[free_adr:free_adr+3] = np.array([1.3, 9.4, 0.4276])
                d.qpos[free_adr+3:free_adr+7] = np.array([1.0, 0.0, 0.0, 0.0])
                d.qvel[:] = 0.0
                d.ctrl[:4] = 0.0
                mujoco.mj_forward(m, d)
                last_controls = [0.0, 0.0, 0.0, 0.0]
                reset_counter += 1
                t_start = time.time()
                last_d_time = d.time
                print("[px4_bridge] 收到 Reset 请求：MuJoCo 已恢复初始姿态", flush=True)
                try:
                    conn.mav.command_long_send(
                        target_system=1, target_component=1,
                        command=400, confirmation=0,
                        param1=0.0, param2=0.0, param3=0.0, param4=0.0,
                        param5=0.0, param6=0.0, param7=0.0,
                    )
                except Exception:
                    pass
                if reset_pub is not None:
                    reset_pub.publish(RosString(data="reset"))

            # 收 PX4 的 HIL 指令（非阻塞）
            while True:
                msg = conn.recv_match(type="HIL_ACTUATOR_CONTROLS", blocking=False)
                if msg is None:
                    break
                last_controls = list(msg.controls[:4])

            wall_target = time.time() - t_start

            reset_detected = d.time + 0.05 < last_d_time

            if reset_detected:
                reset_counter += 1
                last_controls = [0.0, 0.0, 0.0, 0.0]
                d.ctrl[:4] = 0.0
                t_start = time.time() - d.time
                print("[px4_bridge] 检测到 MuJoCo Reset：清零控制并取消 PX4 解锁", flush=True)
                try:
                    conn.mav.command_long_send(
                        target_system=1, target_component=1,
                        command=400, confirmation=0,
                        param1=0.0, param2=0.0, param3=0.0, param4=0.0,
                        param5=0.0, param6=0.0, param7=0.0,
                    )
                except Exception:
                    pass
                if reset_pub is not None:
                    reset_pub.publish(RosString(data="reset"))
                if viewer is not None:
                    viewer.cam.lookat[:] = d.xpos[body_id]
                    viewer.cam.distance = args.dist
                    viewer.cam.elevation = -20.0
                    viewer.cam.azimuth = 90.0

            last_d_time = d.time

            # MuJoCo 是唯一时间源：物理时间 d.time 追到墙钟（1:1 实时，不漂）
            # 循环慢就多步几拍补齐，循环快就 sleep 等墙钟
            while d.time < wall_target:
                apply_motor_model(m, d, body_id, last_controls)
                mujoco.mj_step(m, d)
                step += 1
                if step % 2 == 0:   # 250Hz IMU
                    publish_hil(conn, m, d, body_id, int(d.time*1e6), last_controls)
                if step % 5 == 0:  # 100Hz 接触检测
                    publish_contacts(m, d, body_id)
                if step % 10 == 0:  # 50Hz 真值/EV + /clock
                    publish_odom(conn, m, d, body_id, int(d.time*1e6), reset_counter)
                    if clock_pub is not None:
                        t_us = int(d.time * 1e6)
                        cm = RosClock()
                        cm.clock.sec = t_us // 1000000
                        cm.clock.nanosec = (t_us % 1000000) * 1000
                        try:
                            clock_pub.publish(cm)
                        except Exception:
                            pass
                if step % 500 == 0:
                    clock_drift_last = d.time - (time.time() - t_start)
                    clock_drift_max = max(clock_drift_max, abs(clock_drift_last))
                    R_ = d.xmat[body_id].reshape(3,3)
                    rpy = q_to_rpy(_R_to_quat(R_))
                    vel = d.qvel[:3]
                    print(f"[bridge] step={step} t={d.time:.1f}s drift={clock_drift_last:+.4f}s max_drift={clock_drift_max:.4f}s pos=({d.xpos[body_id][0]:.2f},{d.xpos[body_id][1]:.2f},{d.xpos[body_id][2]:.2f}) rpy=({math.degrees(rpy[0]):.1f},{math.degrees(rpy[1]):.1f},{math.degrees(rpy[2]):.1f})deg v=({vel[0]:.2f},{vel[1]:.2f},{vel[2]:.2f}) ctrl={[round(c,2) for c in last_controls]}", flush=True)
                    if MAX_PENETRATION["depth"] > 1e-4:
                        print(f"[PENETRATION] max={MAX_PENETRATION['depth']:.4f}m geom={MAX_PENETRATION['geom']} pos={MAX_PENETRATION['pos']} t={MAX_PENETRATION['t']:.2f}s", flush=True)

            # 渲染（60Hz 上限）
            if viewer is not None:
                now_r = time.time()
                if now_r - _last_render >= 0.016:
                    _last_render = now_r
                    if args.track:
                        viewer.cam.lookat[:] = d.xpos[body_id]
                    # 中心顶灯闪烁
                    if args.blink > 0:
                        ph = (now_r % args.blink) / args.blink
                        lit = 1.0 if ph < 0.55 else 0.25
                        m.light_diffuse[_LAMP_ID][:] = _LAMP_BASE * lit
                        m.light_specular[_LAMP_ID][:] = _LAMP_SPEC * lit
                    viewer.sync()
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer.close()
        if ros_node is not None:
            try:
                ros_node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass


def q_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


if __name__ == "__main__":
    main()
