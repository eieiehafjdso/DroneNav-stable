#!/usr/bin/env python3
"""把 PX4 的 x500 无人机做成 MuJoCo 模型，并可合并进 RMUC 2025 场地。

只读 PX4 目录，不改它。生成的网格放在 ./meshes/x500/。

用法:
  python build_x500.py                 # 生成 x500.xml（无人机 + 地面）
  python build_x500.py --merge         # 生成 rmuc_2025_drone.xml（无人机 + 场地）
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys

import numpy as np

BUILD_DIR = os.path.dirname(os.path.abspath(__file__))
HERE = os.path.dirname(BUILD_DIR)   # 包根目录
X500_SRC = "/data/DroneNav/PX4-Autopilot/Tools/simulation/gz/models/x500_base"
MESH_DIR = os.path.join(HERE, "meshes", "x500")

# 来自 PX4 x500_base/model.sdf
MASS_BODY = 2.0
INERTIA_BODY = (0.02166666666666667, 0.02166666666666667, 0.04000000000000001)
MASS_ROTOR = 0.016076923076923075
INERTIA_ROTOR = (3.8464910483993325e-07, 2.6115851691700804e-05, 2.649858234714004e-05)
# rotor_i: (x, y, z, 旋转方向 +1=ccw)
ROTORS = [
    ("rotor_0", 0.174, -0.174, 0.06, +1),
    ("rotor_1", -0.174, 0.174, 0.06, +1),
    ("rotor_2", 0.174, 0.174, 0.06, -1),
    ("rotor_3", -0.174, -0.174, 0.06, -1),
]
# 来自 PX4 x500/model.sdf 的电机参数
KF = 8.54858e-06          # N / (rad/s)^2
KM = 0.016                # N·m / (rad/s)^2
MAX_OMEGA = 1000.0        # rad/s
TAU_UP, TAU_DN = 0.0125, 0.025
MESH_X500 = {
    "x500_shell": "x500_shell.obj",
    "x500_arm": "5010Base.obj",
    "x500_bell": "5010Bell.obj",
    "x500_prop_ccw": "1345_prop_ccw.stl",
    "x500_prop_cw": "1345_prop_cw.stl",
}


def ensure_meshes(decimate: float = 0.0):
    """把 PX4 的 DAE/STL 转成 MuJoCo 能用的网格, 并把 SDF 的 pose/scale 烘进顶点"""
    os.makedirs(MESH_DIR, exist_ok=True)
    try:
        import trimesh
        import numpy as np
        import fast_simplification
    except ImportError as e:
        raise SystemExit(f"需要 trimesh / fast-simplification: {e}")

    def bake(src_name, out_name, scale=1.0, offset=(0, 0, 0), yaw=0.0, reduce=None, force=False):
        dst = os.path.join(MESH_DIR, out_name)
        if os.path.exists(dst) and not force:
            return
        m = trimesh.load(os.path.join(X500_SRC, "meshes", src_name), force="mesh", process=False)
        if reduce:
            v, f = fast_simplification.simplify(
                np.asarray(m.vertices, np.float32), np.asarray(m.faces, np.int32),
                target_reduction=reduce)
            m = trimesh.Trimesh(v, f, process=False)
        v = np.asarray(m.vertices) * scale
        if yaw:
            c, s_ = np.cos(yaw), np.sin(yaw)
            v = v @ np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]]).T
        v = v + np.asarray(offset)
        trimesh.Trimesh(v, np.asarray(m.faces), process=False).export(dst)
        print(f"  [烘焙] {src_name} -> {out_name} ({len(m.faces)} 面)")

    # 机体外壳: 绕 z 180° + 上移 0.025 (SDF visual pose)
    bake("NXP-HGD-CF.dae", "x500_shell.obj", 1.0, (0, 0, 0.025), yaw=np.pi, reduce=decimate or None)
    # 机臂
    bake("5010Base.dae", "5010Base.obj")
    # 电机座: 下移 0.032
    bake("5010Bell.dae", "motor_bell.obj", 1.0, (0, 0, -0.032))
    # 桨叶: 缩放 0.846 + 偏移
    for src, out in (("1345_prop_ccw.stl", "prop_ccw.obj"), ("1345_prop_cw.stl", "prop_cw.obj")):
        bake(src, out, 0.846153846, (-0.022, -0.146, -0.016))


ASSETS = """    <mesh name="x500_shell" file="x500/x500_shell.obj"/>
    <mesh name="x500_arm" file="x500/5010Base.obj"/>
    <mesh name="x500_bell" file="x500/motor_bell.obj"/>
    <mesh name="x500_prop_ccw" file="x500/prop_ccw.obj"/>
    <mesh name="x500_prop_cw" file="x500/prop_cw.obj"/>
    <material name="x500_carbon" rgba="0.34 0.34 0.36 1" specular="0.45" shininess="0.7"/>
    <material name="x500_led" rgba="1 0.25 0.15 1" emission="0.9" specular="0.1" shininess="0.5"/>
    <material name="x500_prop" rgba="0.30 0.30 0.33 1" specular="0.25" shininess="0.4"/>
"""


def shell_bottom():
    """读外壳网格，返回 (最低点 z, 底部着地面半宽 x, 半宽 y)"""
    try:
        import trimesh
        import numpy as np
        m = trimesh.load(os.path.join(MESH_DIR, "x500_shell.obj"), force="mesh", process=False)
        v = np.asarray(m.vertices)
        zmin = float(v[:, 2].min())
        low = v[v[:, 2] < zmin + 0.02]
        hx = float(max(abs(low[:, 0]).max(), 0.05))
        hy = float(max(abs(low[:, 1]).max(), 0.05))
        return zmin, hx, hy
    except Exception:
        return -0.2276, 0.127, 0.147


ZMIN, HX, HY = shell_bottom()


def body_fragment(spawn=(1.3, 9.4, 0.42), name="x500"):
    """机体坐标系 = SDF 的 model 坐标系（base_link 在原点，桨在 z=0.06）。

    之前的版本把 base_link 抬到 0.24，导致桨比机身低 0.18 m，
    看起来像"桨悬在机身下面"，这里修正。
    """
    lines = [
        f'    <body name="{name}" pos="{spawn[0]} {spawn[1]} {spawn[2]}">',
        f'      <freejoint name="{name}_freejoint"/>',
        f'      <inertial pos="0 0 0.12" mass="{MASS_BODY}" '
        f'diaginertia="{INERTIA_BODY[0]} {INERTIA_BODY[1]} {INERTIA_BODY[2]}"/>',
        '      <!-- 碰撞：直接用外壳网格(凸包)，视觉面=碰撞面，翻倒也不会悬空 -->',
        '      <geom name="x500_col_shell" type="mesh" mesh="x500_shell" density="0" '
        'condim="3" friction="0.8 0.01 0.001" solref="0.004 1" solimp="0.9 0.95 0.001" '
        'group="3" rgba="0.15 0.15 0.17 1"/>',
        '      <!-- 视觉 -->',
        '      <geom name="x500_shell" type="mesh" mesh="x500_shell" density="0" '
        'material="x500_carbon" contype="0" conaffinity="0" group="2"/>',
        '      <geom name="x500_led_front" type="sphere" pos="0.11 0 0.09" size="0.018" '
        'density="0" material="x500_led" contype="0" conaffinity="0" group="2"/>',
        '      <geom name="x500_led_rear" type="sphere" pos="-0.11 0 0.09" size="0.018" '
        'density="0" material="x500_led" contype="0" conaffinity="0" group="2"/>',
    ]
    for k in range(4):
        sx, sy = (1 if k in (0, 2) else -1), (1 if k > 1 else -1)
        lines.append(
            f'      <geom name="x500_arm_{k}" type="mesh" mesh="x500_arm" density="0" '
            f'pos="{sx * 0.174:.3f} {sy * 0.174:.3f} 0.032" quat="0.974795 0 0 -0.222520" '
            f'material="x500_prop" contype="0" conaffinity="0" group="2"/>')

    for rname, x, y, z, d in ROTORS:
        prop = "x500_prop_ccw" if d > 0 else "x500_prop_cw"
        lines += [
            f'      <body name="{rname}" pos="{x:.3f} {y:.3f} {z:.3f}">',
            f'        <joint name="{rname}_joint" type="hinge" axis="0 0 1" damping="1e-6"/>',
            f'        <inertial pos="0 0 0" mass="{MASS_ROTOR}" '
            f'diaginertia="{INERTIA_ROTOR[0]} {INERTIA_ROTOR[1]} {INERTIA_ROTOR[2]}"/>',
            f'        <geom name="{rname}_motor" type="mesh" mesh="x500_bell" density="0" '
            f'material="x500_prop" contype="0" conaffinity="0" group="2"/>',
            f'        <geom name="{rname}_prop" type="mesh" mesh="{prop}" density="0" '
            f'material="x500_prop" contype="0" conaffinity="0" group="2"/>',
            '      </body>',
        ]
    lines.append('    </body>')
    return "\n".join(lines)


ACTUATORS = """  <actuator>
    <!-- 4 路电机转速指令(rad/s)。推力由 view.py 里的电机模型按 F=KF*ω² 施加 -->
    <velocity name="rotor_0_motor" joint="rotor_0_joint" kv="0.01" forcerange="-0.05 0.05" ctrlrange="0 1000"/>
    <velocity name="rotor_1_motor" joint="rotor_1_joint" kv="0.01" forcerange="-0.05 0.05" ctrlrange="0 1000"/>
    <velocity name="rotor_2_motor" joint="rotor_2_joint" kv="0.01" forcerange="-0.05 0.05" ctrlrange="0 1000"/>
    <velocity name="rotor_3_motor" joint="rotor_3_joint" kv="0.01" forcerange="-0.05 0.05" ctrlrange="0 1000"/>
  </actuator>
"""


def standalone_xml(spawn):
    return f"""<!-- 由 build_x500.py 生成：x500（PX4 网格 + SDF 质量/惯量/电机参数） -->
<mujoco model="x500_standalone">
  <option timestep="0.002" gravity="0 0 -9.81" cone="elliptic" impratio="8"/>
  <compiler angle="radian" meshdir="meshes" autolimits="true"/>
  <visual>
    <global offwidth="1600" offheight="900"/>
    <headlight ambient="0.45 0.45 0.47" diffuse="0.40 0.40 0.40" specular="0.06 0.06 0.06"/>
    <quality shadowsize="2048" offsamples="4"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.02 0.03 0.05" rgb2="0.07 0.09 0.14" width="512" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.05 0.05 0.06" rgb2="0.62 0.62 0.65" width="64" height="64"/>
    <material name="grid_mat" texture="grid" texrepeat="30 30" texuniform="false"/>
{ASSETS}  </asset>
  <worldbody>
    <light name="sun" type="directional" dir="0 0 -1" diffuse="1 1 1" specular="0.1 0.1 0.1" castshadow="true"/>
    <geom name="floor" type="plane" size="40 40 0.1" rgba="0.3 0.3 0.32 1" material="grid_mat" friction="1 0.005 0.0001"/>
{body_fragment(spawn)}
  </worldbody>
{ACTUATORS}</mujoco>
"""


def merge_into_world(field_xml, out_xml, spawn):
    s = open(field_xml, encoding="utf-8").read()
    if "</asset>" not in s or "</worldbody>" not in s:
        raise SystemExit(f"{field_xml} 结构不符合预期")
    s = s.replace("  </asset>", ASSETS + "  </asset>", 1)
    s = s.replace("  </worldbody>", body_fragment(spawn) + "\n  </worldbody>", 1)
    s = s.replace("</mujoco>", ACTUATORS + "</mujoco>", 1)
    s = s.replace("<statistic ", f'<statistic center="{spawn[0]} {spawn[1]} {spawn[2]}" extent="8"/>\n  <!-- ', 1)
    s = s.replace('<mujoco model="rmuc_2025_3m_vio_columns">',
                  '<mujoco model="rmuc_2025_drone">', 1)
    open(out_xml, "w", encoding="utf-8").write(s)
    print(f"已生成 {out_xml}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--merge", action="store_true", help="合并到场地: rmuc_2025_drone.xml")
    ap.add_argument("--spawn", nargs=3, type=float, default=[1.3, 9.4, 0.4276],
                    help="出生点; 默认值 = 场地地面 0.2 + 外壳最低点 0.2276, 开局即触地")
    ap.add_argument("--decimate", type=float, default=0.0,
                    help="机体外壳抽稀比例(0=不抽稀, 用 PX4 原始 17.9 万面)")
    ap.add_argument("--field", default=os.path.join(HERE, "rmuc_2025_field.xml"))
    args = ap.parse_args()

    ensure_meshes(args.decimate)
    if args.merge:
        merge_into_world(args.field, os.path.join(HERE, "rmuc_2025_drone.xml"), args.spawn)
    else:
        out = os.path.join(HERE, "x500.xml")
        open(out, "w", encoding="utf-8").write(standalone_xml(args.spawn))
        print(f"已生成 {out}")


if __name__ == "__main__":
    main()
