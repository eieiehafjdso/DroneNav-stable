"""打开 MuJoCo 世界（只负责显示，不打印、不接管键盘）

    ./run_view.sh               默认 rmuc_2025_drone.xml
    ./run_view.sh --no-track    关闭镜头跟随
    ./run_view.sh --speed 0.5   慢放

镜头每帧锁定到无人机位置（lookat），鼠标仍可绕着它转。
"""
import argparse
import os
import sys
import time

import mujoco
import mujoco.viewer

SRC_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(SRC_DIR)
sys.path.insert(0, SRC_DIR)

from motor_model import apply as apply_thrust

ap = argparse.ArgumentParser()
ap.add_argument("xml", nargs="?", default=os.path.join(ROOT, "rmuc_2025_drone.xml"))
ap.add_argument("--track", default="x500")
ap.add_argument("--no-track", action="store_true")
ap.add_argument("--speed", type=float, default=1.0)
ap.add_argument("--dist", type=float, default=2.2, help="跟随距离(米)")
args = ap.parse_args()

m = mujoco.MjModel.from_xml_path(args.xml)
d = mujoco.MjData(m)

viewer = mujoco.viewer.launch_passive(m, d)

track_id = -1
if args.track and not args.no_track:
    track_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, args.track)
    if track_id >= 0:
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.distance = args.dist
        viewer.cam.elevation = -18.0
        viewer.cam.azimuth = 90.0

t0 = time.time()
try:
    while viewer.is_running():
        target = (time.time() - t0) * args.speed
        while d.time < target:
            apply_thrust(m, d)          # 电机模型: ctrl(rad/s) -> 推力
            mujoco.mj_step(m, d)
            if track_id >= 0:
                viewer.cam.lookat[:] = d.xpos[track_id]   # 每步跟随，避免跳变
        viewer.sync()
        time.sleep(0.001)
except KeyboardInterrupt:
    pass
finally:
    viewer.close()
import os
os._exit(0)
