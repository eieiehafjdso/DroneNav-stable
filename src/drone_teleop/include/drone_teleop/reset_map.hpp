// 冲击重置映射（论文 §II-C 矩阵模型 / §III-B 对偶四元数模型）
// 对应 dq_reference/reset_map.py
#pragma once

#include "drone_teleop/dq_math.hpp"

namespace drone_teleop {

struct ImpactState {
  Vec3 omega_plus{Vec3::Zero()};  ///< 撞后机体角速度
  Vec3 v_world_plus{Vec3::Zero()}; ///< 撞后世界系线速度
};

/// 由撞前接触点滑动速度求摩擦冲量方向（世界系单位向量）。
/// 库仑摩擦阻碍滑动，故返回「滑动速度的反方向」，保持论文公式耗散。
inline Vec3 contactTangent(const Vec3& v_world, const Vec3& omega, const Quat& q,
                           const Vec3& r_c, const Vec3& n_world)
{
  const Mat3 R = rotMatrix(q);
  const Vec3 v_c = v_world + R * omega.cross(r_c);  // 接触点世界系速度
  Vec3 v_t = v_c - v_c.dot(n_world) * n_world;      // 切向滑动分量
  const double norm = v_t.norm();
  if (norm < 1e-12) {
    Vec3 basis(1.0, 0.0, 0.0);
    if (std::abs(basis.dot(n_world)) > 0.9) {
      basis = Vec3(0.0, 1.0, 0.0);
    }
    v_t = basis - basis.dot(n_world) * n_world;
    v_t.normalize();
    return -v_t;
  }
  return -v_t / norm;
}

/// 经典牛顿矩阵冲击模型（Eq.2, Eq.3）
inline ImpactState matrixReset(const Vec3& r_c, const Vec3& n_world, const Vec3& t_world,
                               const Quat& q, double e, double mu, const Vec3& omega,
                               const Vec3& v_world, const Mat3& J, double m)
{
  const Mat3 R = rotMatrix(q);
  const Vec3 n_body = quatInvRotate(q, n_world);

  // 有效质量 ρ
  const Vec3 rn = r_c.cross(n_body);
  const double rho = 1.0 / m + rn.dot(J.ldlt().solve(rn));

  // 冲量大小与冲量向量
  const double lam = -(1.0 + e) * (v_world + R * omega.cross(r_c)).dot(n_world) / rho;
  const Vec3 lam_world = lam * (n_world + mu * t_world);

  ImpactState out;
  out.v_world_plus = v_world + lam_world / m;
  out.omega_plus = omega + J.ldlt().solve(r_c.cross(quatInvRotate(q, lam_world)));
  return out;
}

/// 对偶四元数冲击重置映射（Eq.9 ~ Eq.14）
inline ImpactState dqReset(const Vec3& r_c, const Vec3& n_world, const Vec3& t_world,
                           const Quat& q, double e, double mu, const Vec3& omega,
                           const Vec3& v_world, const Mat3& J, double m)
{
  const Vec3 n_body = quatInvRotate(q, n_world);
  const Vec3 t_body = quatInvRotate(q, t_world);

  // 螺旋轴（Eq.10，Plücker 线坐标）：ŝ = (r_c × n) + ε n
  const Dual3 s_n(r_c.cross(n_body), n_body);
  const Dual3 s_t(r_c.cross(t_body), t_body);

  // 撞前对偶扭转（机体系）：ξ̂⁻ = ω + ε v_B
  const Dual3 xi_minus(omega, quatInvRotate(q, v_world));

  // 冲量大小 Λ（Eq.14）
  const double num = -(1.0 + e) * xi_minus.dot(s_n);
  const double den = dualInertiaInv(s_n, J, m).dot(s_n)
                    + mu * dualInertiaInv(s_t, J, m).dot(s_n);
  const double lam = num / den;

  // 总碰撞力螺旋 Ŵ = Λ(ŝ_n + μ ŝ_t)
  const Dual3 W(lam * (s_n.real + mu * s_t.real), lam * (s_n.dual + mu * s_t.dual));

  // ξ̂⁺ = ξ̂⁻ + M⁻¹(Ŵ)
  const Dual3 dxi = dualInertiaInv(W, J, m);
  const Dual3 xi_plus(xi_minus.real + dxi.real, xi_minus.dual + dxi.dual);

  ImpactState out;
  out.omega_plus = xi_plus.real;
  out.v_world_plus = quatRotate(q, xi_plus.dual);
  return out;
}

}  // namespace drone_teleop
