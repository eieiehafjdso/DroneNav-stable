# -*- coding: utf-8 -*-
"""
对偶四元数代数工具库 (Phase 0)
================================

实现论文:
  V. Gaucher, W. Zhang, "Dual Quaternion Based Contact Modeling for Fast
  and Smooth Collision Recovery of Quadrotors", IEEE L-CSS / arXiv:2603.14698

约定 (论文 §II-A / §III-A):
  * 单位四元数 q = (w, x, y, z), Hamilton 乘法, q ⊙ v = q ⊗ v ⊗ q*
  * 位姿对偶四元数  q̂ = q + ε q', 其中 q' = ½ p ⊗ q (p 为世界系位置)
  * 对偶扭转        ξ̂ = ω + ε v_B   (ω 机体角速度, v_B 机体线速度)
  * 对偶动量        Ĥ = M(ξ̂) = J ω + ε (m v_B)

内部表示:
  * "纯对偶向量" (实部/对偶部都是 3 维向量) 用元组 (real, dual) 表示
  * "对偶四元数" (实部/对偶部都是 4 维四元数) 用元组 (real_q, dual_q) 表示

依赖: numpy
"""

import numpy as np

# ---------------------------------------------------------------------------
# 四元数基础 (Hamilton 约定)
# ---------------------------------------------------------------------------


def quat_mult(a, b):
    """四元数乘法 a ⊗ b (Hamilton)"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_conj(q):
    """四元数共轭 q*"""
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q):
    """归一化四元数"""
    n = np.linalg.norm(q)
    if n < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return np.asarray(q, dtype=float) / n


def quat_rotate(q, v):
    """q ⊙ v : 用四元数 q 旋转向量 v (机体系 → 世界系)"""
    vq = np.array([0.0, v[0], v[1], v[2]])
    return quat_mult(quat_mult(q, vq), quat_conj(q))[1:]


def quat_inv_rotate(q, v):
    """q* ⊙ v : 反向旋转 (世界系 → 机体系)"""
    vq = np.array([0.0, v[0], v[1], v[2]])
    return quat_mult(quat_mult(quat_conj(q), vq), q)[1:]


def quat_exp(w):
    """纯四元数指数 exp(w), w 为旋转向量 (返回单位四元数)"""
    theta = np.linalg.norm(w)
    if theta < 1e-15:
        return np.array([1.0, 0.0, 0.0, 0.0])
    s = np.sin(theta / 2.0) / theta
    return np.array([np.cos(theta / 2.0), s * w[0], s * w[1], s * w[2]])


def quat_log(q):
    """四元数对数 log(q) → 旋转向量"""
    q = quat_normalize(q)
    w = q[0]
    v = q[1:]
    vn = np.linalg.norm(v)
    if vn < 1e-15:
        return np.zeros(3)
    theta = 2.0 * np.arctan2(vn, w)
    return theta * v / vn


def rot_matrix(q):
    """旋转矩阵 R(q): 机体系 → 世界系"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def skew(v):
    """向量 → 反对称矩阵"""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


# ---------------------------------------------------------------------------
# 对偶代数: 纯对偶向量 (实部/对偶部都是 3 维)
# ---------------------------------------------------------------------------


def dual_dot(a, b):
    """对偶点积 ⟨â, b̂⟩ = a_r·b_r + a_d·b_d  (论文 §II-A)"""
    return float(np.dot(a[0], b[0]) + np.dot(a[1], b[1]))


def dual_cross(a, b):
    """对偶叉积 × (运动学) : a×c + ε(a×d + b×c)"""
    return (
        np.cross(a[0], b[0]),
        np.cross(a[0], b[1]) + np.cross(a[1], b[0]),
    )


def dual_cross_star(a, b):
    """对偶伴随叉积 ×* (动力学) : (a×c + b×d) + ε(a×d)"""
    return (
        np.cross(a[0], b[0]) + np.cross(a[1], b[1]),
        np.cross(a[0], b[1]),
    )


def dual_inertia(xi, J, m):
    """对偶惯量算子 M(ξ̂) = J ω + ε (m v_B)"""
    omega, vB = xi
    return (J @ omega, m * vB)


def dual_inertia_inv(xi, J, m):
    """对偶惯量逆 M⁻¹(ξ̂) = J⁻¹ ω + ε (m⁻¹ v_B)"""
    omega, vB = xi
    return (np.linalg.solve(J, omega), vB / m)


# ---------------------------------------------------------------------------
# 对偶四元数 (实部/对偶部都是 4 维)
# ---------------------------------------------------------------------------


def dq_from_pose(q, p):
    """位姿 (q, p) → 单位对偶四元数 q̂ = q + ε(½ p ⊗ q)"""
    pq = np.array([0.0, p[0], p[1], p[2]])
    return (np.asarray(q, dtype=float), 0.5 * quat_mult(pq, q))


def dq_pose(qh):
    """单位对偶四元数 q̂ → (q, p), 其中 p = 2 q' ⊗ q*"""
    q, qd = qh
    p = quat_mult(qd, quat_conj(q))[1:] * 2.0
    return q, p


def dq_mult(qh1, qh2):
    """对偶四元数乘法 q̂1 ⊗ q̂2 = q1q2 + ε(q1 q2' + q1' q2)"""
    q1, q1d = qh1
    q2, q2d = qh2
    return (
        quat_mult(q1, q2),
        quat_mult(q1, q2d) + quat_mult(q1d, q2),
    )


def dq_conj(qh):
    """对偶四元数共轭 q̂* = q* + ε (q')*  (逐四元数共轭)"""
    q, qd = qh
    return (quat_conj(q), quat_conj(qd))


def dq_unit_check(qh):
    """
    检查单位对偶四元数条件: ‖q‖=1 且 q·q' = 0
    返回 (‖q‖, q·q')
    """
    q, qd = qh
    return (float(np.linalg.norm(q)), float(np.dot(q, qd)))


def so3_left_jacobian(w):
    """SO(3) 左雅可比 J_l(ω) (Barfoot 公式 7.87a)"""
    theta = np.linalg.norm(w)
    if theta < 1e-12:
        return np.eye(3)
    w_hat = skew(w)
    A = (1.0 - np.cos(theta)) / (theta ** 2)
    B = (theta - np.sin(theta)) / (theta ** 3)
    return np.eye(3) + A * w_hat + B * (w_hat @ w_hat)


def so3_left_jacobian_inv(w):
    """SO(3) 左雅可比逆 J_l⁻¹(ω) (Barfoot 公式 7.87b)"""
    theta = np.linalg.norm(w)
    if theta < 1e-12:
        return np.eye(3)
    w_hat = skew(w)
    A = 1.0 / 2.0
    B = 1.0 / (theta ** 2) - (1.0 + np.cos(theta)) / (2.0 * theta * np.sin(theta))
    return np.eye(3) - A * w_hat + B * (w_hat @ w_hat)


def dq_exp(screw):
    """
    对偶指数映射 exp(δ̂), δ̂ = δ_ω + ε δ_v 为纯对偶向量 (李代数元素)
    返回单位对偶四元数 q̂ = q + ε(½ p ⊗ q), 其中
      q = exp(δ_ω),  p = J_l(δ_ω) δ_v

    注: 论文里的 q̂_Δ = exp(½ δ̂) 对应本函数调用 dq_exp((δ_ω/2, δ_v/2))
    """
    dw, dv = screw
    q = quat_exp(dw)
    p = so3_left_jacobian(dw) @ dv
    return dq_from_pose(q, p)


def dq_log(qh):
    """对偶对数映射: 单位对偶四元数 → 纯对偶向量 δ̂ = δ_ω + ε δ_v"""
    q, p = dq_pose(qh)
    dw = quat_log(q)
    dv = so3_left_jacobian_inv(dw) @ p
    return (dw, dv)
