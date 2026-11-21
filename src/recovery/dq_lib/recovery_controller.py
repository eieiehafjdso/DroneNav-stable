# -*- coding: utf-8 -*-
"""
碰撞恢复控制器 (Phase 2)
==========================

实现论文 §III-C:
  * 导纳增益 Γ̂ (Eq.21)
  * 刹车位移 δ̂ = Γ̂ ∘ ξ̂⁺ (Eq.22)
  * 参考位姿偏移 q̂_d = q̂_Δ ⊗ q̂(t_c), q̂_Δ = exp(½δ̂) (Eq.23a)
  * 位姿误差 q̂_e (Eq.23b)
  * 几何控制律 F̂_a (Eq.24) → 力矩 τ + 力 f (机体系)
  * 增益上界 (Eq.31, 能量预算分配系数 α)

依赖: dual_quaternion.py, numpy
"""

import numpy as np

from dual_quaternion import (
    quat_mult,
    quat_conj,
    quat_inv_rotate,
    quat_exp,
    rot_matrix,
    dq_exp,
    dq_mult,
    dq_from_pose,
    dq_pose,
)

G_VEC = np.array([0.0, 0.0, -9.81])   # 世界系重力 (z 向上)


def kinetic_energy(omega, v_B, J, m):
    """动能 ½⟨ξ̂, M(ξ̂)⟩ = ½ ωᵀJω + ½ m‖v_B‖²"""
    return 0.5 * float(omega @ (J @ omega)) + 0.5 * m * float(v_B @ v_B)


def potential_error(q_e, p_e, k_q, k_p):
    """V_pos = 2k_q(1 - q_e^w) + ½k_p‖p_e‖²  (Eq.25 位置部分)"""
    return 2.0 * k_q * (1.0 - q_e[0]) + 0.5 * k_p * float(p_e @ p_e)


def admittance_gains(omega_plus, v_B_plus, E, k_q, k_p, alpha=0.5, factor=0.5):
    """
    按 Eq.31 合成导纳增益上界, 取上界的 factor 倍 (安全裕度).
    返回标量 (gamma_omega, gamma_v), 对应各向同性对角增益.

    E = E_diss + V_pos(q̂_e⁻) 为可分配能量预算
    """
    norm_om2 = float(omega_plus @ omega_plus)
    norm_vb2 = float(v_B_plus @ v_B_plus)
    g_om = 0.0
    g_v = 0.0
    if norm_om2 > 1e-6:
        g_om = np.sqrt(4.0 * alpha * E / (k_q * norm_om2))
    if norm_vb2 > 1e-6:
        g_v = np.sqrt(2.0 * (1.0 - alpha) * E / (k_p * norm_vb2))
    # 数值保护：|ω+| 或 |v_B+| 极小时上式会爆，钳到合理上界
    G_MAX = 5.0
    g_om = min(g_om, G_MAX)
    g_v = min(g_v, G_MAX)
    return factor * g_om, factor * g_v


def dq_admittance_shift(q, p, omega_plus, v_B_plus, gamma_om, gamma_v):
    """
    统一(耦合)导纳让步 (Eq.22~23a):
      δ̂ = Γ̂ ∘ ξ̂⁺,  q̂_Δ = exp(½δ̂),  q̂_d = q̂_Δ ⊗ q̂(t_c)
    返回 (q_d, p_d)
    """
    dw = gamma_om * omega_plus
    dv = gamma_v * v_B_plus
    # 让步幅度钳位：单次恢复里期望姿态/位置跳变必须有界，否则 PX4 姿态环会被甩翻
    DW_MAX = 0.30      # rad (~17°)
    DV_MAX = 0.50      # m/s
    nw = float(np.linalg.norm(dw))
    if nw > DW_MAX:
        dw = dw * (DW_MAX / nw)
    nv = float(np.linalg.norm(dv))
    if nv > DV_MAX:
        dv = dv * (DV_MAX / nv)
    qh_delta = dq_exp((0.5 * dw, 0.5 * dv))
    qh_cur = dq_from_pose(q, p)
    qh_d = dq_mult(qh_delta, qh_cur)
    return dq_pose(qh_d)


def decoupled_admittance_shift(q, p, omega_plus, v_W_plus, gamma_om, gamma_v):
    """
    解耦导纳让步 (基线, 位置/姿态独立处理):
      p_d = p + ½ gamma_v v_W⁺ ;  q_d = exp(½ gamma_om ω⁺) ⊗ q
    (½ 因子与 DQ 侧 exp(½δ̂) 的位移尺度保持一致, 保证对比公平)
    返回 (q_d, p_d)
    """
    p_d = p + 0.5 * gamma_v * v_W_plus
    q_d = quat_mult(quat_exp(0.5 * gamma_om * omega_plus), q)
    return q_d, p_d


def geometric_control(q, p, v_B, omega, q_d, p_d, J, m, k_q, k_p, k_dw, k_dv):
    """
    几何控制律 (Eq.24), 分量形式:
      τ = ω×(Jω) - k_q q_e^v - k_dw ω
      f = -m Rᵀg + m(ω×v_B) - k_p p_e - k_dv v_B
    返回 (tau, f), 均在机体系.
    """
    # 位姿误差 (Eq.23b): q_e = q_d* ⊗ q, p_e = R_dᵀ(p - p_d)
    q_e = quat_mult(quat_conj(q_d), q)
    if q_e[0] < 0.0:
        q_e = -q_e
    q_ev = q_e[1:4]
    p_e = quat_inv_rotate(q_d, p - p_d)

    R = rot_matrix(q)

    tau = np.cross(omega, J @ omega) - k_q * q_ev - k_dw * omega
    f = -m * (R.T @ G_VEC) + m * np.cross(omega, v_B) - k_p * p_e - k_dv * v_B
    # 执行器能力钳位：力矩 <= TAU_MAX，推力 <= 2.5mg（防止数值解甩翻无人机）
    TAU_MAX = 1.0       # N*m
    F_MAX = 2.5 * m * 9.81
    tn = float(np.linalg.norm(tau))
    if tn > TAU_MAX:
        tau = tau * (TAU_MAX / tn)
    fn = float(np.linalg.norm(f))
    if fn > F_MAX:
        f = f * (F_MAX / fn)
    return tau, f
