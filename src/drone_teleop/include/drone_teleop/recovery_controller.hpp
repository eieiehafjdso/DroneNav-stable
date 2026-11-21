// 碰撞恢复控制器（论文 §III-C，Eq.21~Eq.31）
// 对应 dq_reference/recovery_controller.py
#pragma once

#include "drone_teleop/dq_math.hpp"

namespace drone_teleop {

/// 动能 ½⟨ξ̂, M(ξ̂)⟩ = ½ ωᵀJω + ½ m‖v_B‖²
inline double kineticEnergy(const Vec3& omega, const Vec3& v_body, const Mat3& J, double m)
{
  return 0.5 * omega.dot(J * omega) + 0.5 * m * v_body.squaredNorm();
}

/// V_pos = 2k_q(1 - q_e^w) + ½ k_p ‖p_e‖²
inline double potentialError(const Quat& q_e, const Vec3& p_e, double k_q, double k_p)
{
  Quat qe = q_e;
  if (qe.w() < 0.0) {
    qe.coeffs() *= -1.0;  // 取短弧
  }
  return 2.0 * k_q * (1.0 - qe.w()) + 0.5 * k_p * p_e.squaredNorm();
}

/// 按 Eq.31 合成导纳增益上界，取上界的 factor 倍作为安全裕度。
/// 返回各向同性标量 (Γ_ω, Γ_v)。
inline void admittanceGains(const Vec3& omega_plus, const Vec3& v_body_plus, double E,
                            double k_q, double k_p, double alpha, double factor,
                            double& gamma_omega, double& gamma_v)
{
  const double norm_om2 = omega_plus.squaredNorm();
  const double norm_vb2 = v_body_plus.squaredNorm();
  double g_om = 0.0;
  double g_v = 0.0;
  if (norm_om2 > 1e-12 && k_q > 1e-9) {
    g_om = std::sqrt(4.0 * alpha * std::max(E, 0.0) / (k_q * norm_om2));
  }
  if (norm_vb2 > 1e-12 && k_p > 1e-9) {
    g_v = std::sqrt(2.0 * (1.0 - alpha) * std::max(E, 0.0) / (k_p * norm_vb2));
  }
  gamma_omega = factor * g_om;
  gamma_v = factor * g_v;
}

/// 耦合导纳让步（Eq.22~23a）：δ̂ = Γ̂∘ξ̂⁺, q̂_Δ = exp(½δ̂), q̂_d = q̂_Δ ⊗ q̂(t_c)
inline void dqAdmittanceShift(const Quat& q, const Vec3& p, const Vec3& omega_plus,
                              const Vec3& v_body_plus, double gamma_omega, double gamma_v,
                              Quat& q_d, Vec3& p_d)
{
  const Dual3 delta(gamma_omega * omega_plus, gamma_v * v_body_plus);
  const Dual3 delta_half(0.5 * delta.real, 0.5 * delta.dual);
  const DQ qh_delta = dqExp(delta_half);
  const DQ qh_cur = DQ::fromPose(q, p);
  (qh_delta * qh_cur).toPose(q_d, p_d);
}

/// 解耦导纳让步（基线）：p_d = p + ½Γ_v v_W⁺, q_d = exp(½Γ_ω ω⁺) ⊗ q
inline void decoupledAdmittanceShift(const Quat& q, const Vec3& p, const Vec3& omega_plus,
                                     const Vec3& v_world_plus, double gamma_omega,
                                     double gamma_v, Quat& q_d, Vec3& p_d)
{
  p_d = p + 0.5 * gamma_v * v_world_plus;
  q_d = quatMult(quatExp(0.5 * gamma_omega * omega_plus), q);
}

/// 几何控制律（Eq.24，分量形式），输出机体系力矩 τ 与力 f。
inline void geometricControl(const Quat& q, const Vec3& p, const Vec3& v_body,
                             const Vec3& omega, const Quat& q_d, const Vec3& p_d,
                             const Mat3& J, double m, double k_q, double k_p, double k_dw,
                             double k_dv, Vec3& tau, Vec3& f_body)
{
  // 位姿误差（Eq.23b）
  Quat q_e = quatConj(q_d) * q;
  if (q_e.w() < 0.0) {
    q_e.coeffs() *= -1.0;
  }
  const Vec3 q_ev = q_e.vec();
  const Vec3 p_e = quatInvRotate(q_d, p - p_d);

  const Mat3 R = rotMatrix(q);
  const Vec3 g_world(0.0, 0.0, -kGravity);

  tau = omega.cross(J * omega) - k_q * q_ev - k_dw * omega;
  f_body = -m * (R.transpose() * g_world) + m * omega.cross(v_body) - k_p * p_e - k_dv * v_body;
}

/// 由机体系期望力投影出（欠驱）期望推力方向与期望姿态（世界系 ENU / 机体系 FLU）。
/// F_W = R f_B；b3 = F_W/|F_W|；航向由 yaw_des 决定。
inline void thrustToAttitude(const Quat& q, const Vec3& f_body, double yaw_des, Quat& q_des,
                             double& thrust)
{
  const Mat3 R = rotMatrix(q);
  Vec3 F_W = R * f_body;
  if (F_W.norm() < 1e-6) {
    F_W = Vec3(0.0, 0.0, 1e-3);
  }
  thrust = F_W.norm();
  const Vec3 b3 = F_W.normalized();

  Vec3 b1c(std::cos(yaw_des), std::sin(yaw_des), 0.0);
  Vec3 b2 = b3.cross(b1c);
  if (b2.norm() < 1e-6) {
    b1c = Vec3(1.0, 0.0, 0.0);
    b2 = b3.cross(b1c);
  }
  b2.normalize();
  const Vec3 b1 = b2.cross(b3);

  Mat3 Rd;
  Rd.col(0) = b1;
  Rd.col(1) = b2;
  Rd.col(2) = b3;
  q_des = Quat(Rd).normalized();
}

}  // namespace drone_teleop
