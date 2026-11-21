# -*- coding: utf-8 -*-
"""
冲击重置映射 (Phase 1)
======================

实现论文 §II-C 的经典矩阵模型 与 §III-B 的对偶四元数模型,
两套接口输出一致, 供 Proposition 1 数值等价验证使用。

输入约定:
  * r_c   : 接触点, 机体系坐标 (3)
  * n_W   : 接触面法向, 世界系, 单位向量 (3)
  * t_W   : 摩擦冲量方向 (与滑动速度相反), 世界系, 单位向量 (3)
  * q     : 无人机姿态 (机体系→世界系), 单位四元数 (w,x,y,z)
  * e     : 恢复系数
  * mu    : 摩擦系数
  * omega : 撞前机体角速度 (3)
  * v_W   : 撞前世界系线速度 (3)
  * J     : 惯量矩阵 (3x3)
  * m     : 质量

输出约定 (两套接口一致):
  * omega_plus : 撞后机体角速度 (3)
  * v_W_plus   : 撞后世界系线速度 (3)
"""

import numpy as np

from dual_quaternion import (
    quat_inv_rotate,
    quat_rotate,
    rot_matrix,
    dual_dot,
    dual_inertia_inv,
)


def contact_tangent(v_W, omega, q, r_c, n_W):
    """
    由撞前接触点滑动速度计算摩擦冲量方向 t_W (单位向量).

    注意: 库仑摩擦阻碍滑动, 故 t 取「滑动速度的相反方向」,
    这样论文公式 λ = Λ(n + μt) / Ŵ = Λ(ŝ_n + μ ŝ_t) 才是耗散的。
    (论文 §III-B 文字把 t 写作"滑动速度方向", 但物理上摩擦须反向,
     此处按耗散方向实现, Proposition 1 的等价性不受影响。)
    """
    R = rot_matrix(q)
    v_c = v_W + R @ np.cross(omega, r_c)      # 接触点世界系速度
    v_t = v_c - np.dot(v_c, n_W) * n_W        # 切向(滑动)分量
    norm = np.linalg.norm(v_t)
    if norm < 1e-12:
        # 无切向滑动时, 摩擦方向对结果无影响, 任取与 n_W 正交的单位向量
        basis = np.array([1.0, 0.0, 0.0])
        if abs(np.dot(basis, n_W)) > 0.9:
            basis = np.array([0.0, 1.0, 0.0])
        v_t = basis - np.dot(basis, n_W) * n_W
        v_t = v_t / np.linalg.norm(v_t)
        return -v_t
    return -v_t / norm


def matrix_reset(r_c, n_W, t_W, q, e, mu, omega, v_W, J, m):
    """经典矩阵冲击模型 (论文 Eq.2, Eq.3)"""
    R = rot_matrix(q)
    n_B = quat_inv_rotate(q, n_W)

    # 接触点撞前速度 (世界系): v_c = v_W + R(ω × r_c)
    v_c = v_W + R @ np.cross(omega, r_c)

    # 有效质量 ρ (Eq.3b)
    rn = np.cross(r_c, n_B)
    rho = 1.0 / m + float(rn @ np.linalg.solve(J, rn))

    # 冲量大小与冲量向量 (Eq.3a)
    lam = -(1.0 + e) * float(np.dot(v_c, n_W)) / rho
    lam_W = lam * (n_W + mu * t_W)

    # 撞后状态 (Eq.2)
    v_W_plus = v_W + lam_W / m
    omega_plus = omega + np.linalg.solve(J, np.cross(r_c, quat_inv_rotate(q, lam_W)))

    return omega_plus, v_W_plus


def dq_reset(r_c, n_W, t_W, q, e, mu, omega, v_W, J, m):
    """对偶四元数冲击重置映射 (论文 Eq.9 ~ Eq.14)"""
    # 世界系法向/切向 → 机体系
    n_B = quat_inv_rotate(q, n_W)
    t_B = quat_inv_rotate(q, t_W)

    # 螺旋轴 (Eq.10, Plücker 线坐标): ŝ = (r_c × n) + ε n
    s_n = (np.cross(r_c, n_B), n_B)
    s_t = (np.cross(r_c, t_B), t_B)

    # 撞前对偶扭转 (机体系): ξ̂⁻ = ω + ε v_B
    v_B = quat_inv_rotate(q, v_W)
    xi_minus = (omega, v_B)

    # 冲量大小 Λ (Eq.14)
    num = -(1.0 + e) * dual_dot(xi_minus, s_n)
    den = dual_dot(dual_inertia_inv(s_n, J, m), s_n)
    cross_den = dual_dot(dual_inertia_inv(s_t, J, m), s_n)
    den += mu * cross_den
    lam = num / den

    # 总碰撞力螺旋 Ŵ = Λ(ŝ_n + μ ŝ_t) (Eq.11)
    W = (lam * (s_n[0] + mu * s_t[0]),
         lam * (s_n[1] + mu * s_t[1]))

    # 撞后扭转 ξ̂⁺ = ξ̂⁻ + M⁻¹(Ŵ) (Eq.9)
    dxi = dual_inertia_inv(W, J, m)
    xi_plus = (xi_minus[0] + dxi[0], xi_minus[1] + dxi[1])

    omega_plus = xi_plus[0]
    v_B_plus = xi_plus[1]
    v_W_plus = quat_rotate(q, v_B_plus)      # 机体系 → 世界系

    return omega_plus, v_W_plus
