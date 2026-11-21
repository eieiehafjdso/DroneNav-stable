"""x500 电机模型（MuJoCo 没有旋翼气动，这一步必须由外部施加）

    F_i = KF * ω_i²        推力（N），沿机体 z 轴
    M_i = KM * ω_i²        反扭矩（N·m），方向随旋向
    ω_i 直接取 4 个 motor 执行器的 ctrl（rad/s）

在 viewer 里拖动 Control 面板的 rotor_i_motor 滑块，飞机会真的起飞。
"""
import numpy as np

KF = 8.54858e-06        # N/(rad/s)²   (PX4 motorConstant)
KM = 1.71e-07           # N·m/(rad/s)² (kq ≈ KF*0.02)
ROTOR_POS = np.array([[0.174, -0.174, 0.06],
                      [-0.174, 0.174, 0.06],
                      [0.174, 0.174, 0.06],
                      [-0.174, -0.174, 0.06]])
DIRS = np.array([-1.0, -1.0, +1.0, +1.0])


def apply(model, data, body="x500"):
    """按当前 ctrl(转速) 给机体施加推力与反扭矩"""
    bid = model.body(body).id
    omega = np.asarray(data.ctrl[:4], dtype=float)
    data.xfrc_applied[bid] = 0.0
    R = data.xmat[bid].reshape(3, 3)
    axis = R[:, 2]
    F = np.zeros(3)
    T = np.zeros(3)
    for i in range(4):
        f = KF * omega[i] ** 2
        force = axis * f
        F += force
        T += np.cross(R @ ROTOR_POS[i], force)
        T += axis * (KM * omega[i] ** 2 * DIRS[i])
    data.xfrc_applied[bid, :3] = F
    data.xfrc_applied[bid, 3:] = T
    return float(np.sum(KF * omega ** 2))
