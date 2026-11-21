// Copyright 2026 DroneNav
// 对偶四元数代数（C++ 版，对应 dq_reference/dual_quaternion.py）
//
// 约定（论文 §II-A / §III-A）：
//   * 单位四元数 q = (w,x,y,z)，Hamilton 乘法，机体系 -> 世界系；世界系 z 向上(ENU 风格)
//   * 位姿对偶四元数  q̂ = q + ε q',  q' = ½ p ⊗ q  (p 为世界系位置)
//   * 对偶扭转        ξ̂ = ω + ε v_B
//   * 对偶动量        Ĥ = M(ξ̂) = Jω + ε (m v_B)
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace drone_teleop {

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;
using Mat3 = Eigen::Matrix3d;

inline constexpr double kGravity = 9.81;

// ---------------- 四元数基础 ----------------

inline Quat quatMult(const Quat& a, const Quat& b) { return a * b; }
inline Quat quatConj(const Quat& q) { return q.conjugate(); }

inline Quat quatNormalized(const Quat& q)
{
  const double n = q.norm();
  if (n < 1e-15) {
    return Quat::Identity();
  }
  Quat out = q;
  out.coeffs() /= n;
  out.normalize();
  return out;
}

inline Vec3 quatRotate(const Quat& q, const Vec3& v) { return q * v; }
inline Vec3 quatInvRotate(const Quat& q, const Vec3& v) { return q.conjugate() * v; }

/// 旋转向量 -> 单位四元数
inline Quat quatExp(const Vec3& w)
{
  const double theta = w.norm();
  if (theta < 1e-15) {
    return Quat::Identity();
  }
  return Quat(Eigen::AngleAxisd(theta, w / theta));
}

/// 四元数 -> 旋转向量（取 w>=0 的短弧）
inline Vec3 quatLog(const Quat& q_in)
{
  Quat q = quatNormalized(q_in);
  if (q.w() < 0.0) {
    q.coeffs() *= -1.0;
  }
  const Vec3 v = q.vec();
  const double vn = v.norm();
  if (vn < 1e-15) {
    return Vec3::Zero();
  }
  const double theta = 2.0 * std::atan2(vn, q.w());
  return theta * v / vn;
}

inline Quat quatScale(const Quat& q, double s)
{
  Quat out = q;
  out.coeffs() *= s;
  return out;
}

inline Mat3 rotMatrix(const Quat& q) { return q.toRotationMatrix(); }

inline Mat3 skew(const Vec3& v)
{
  Mat3 m;
  m << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return m;
}

/// SO(3) 左雅可比（Barfoot 7.87a）
inline Mat3 so3LeftJacobian(const Vec3& w)
{
  const double theta = w.norm();
  if (theta < 1e-12) {
    return Mat3::Identity();
  }
  const Mat3 w_hat = skew(w);
  const double A = (1.0 - std::cos(theta)) / (theta * theta);
  const double B = (theta - std::sin(theta)) / (theta * theta * theta);
  return Mat3::Identity() + A * w_hat + B * (w_hat * w_hat);
}

/// SO(3) 左雅可比逆（Barfoot 7.87b）
inline Mat3 so3LeftJacobianInv(const Vec3& w)
{
  const double theta = w.norm();
  if (theta < 1e-12) {
    return Mat3::Identity();
  }
  const Mat3 w_hat = skew(w);
  const double A = 0.5;
  const double B = 1.0 / (theta * theta) -
                   (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta));
  return Mat3::Identity() - A * w_hat + B * (w_hat * w_hat);
}

// ---------------- 纯对偶向量（实部/对偶部均为 3 维） ----------------

struct Dual3 {
  Vec3 real{Vec3::Zero()};
  Vec3 dual{Vec3::Zero()};

  Dual3() = default;
  Dual3(const Vec3& r, const Vec3& d) : real(r), dual(d) {}

  /// ⟨â,b̂⟩ = a_r·b_r + a_d·b_d
  double dot(const Dual3& other) const { return real.dot(other.real) + dual.dot(other.dual); }

  /// 对偶叉积 ×：a×c + ε(a×d + b×c)
  Dual3 cross(const Dual3& other) const
  {
    return Dual3(real.cross(other.real), real.cross(other.dual) + dual.cross(other.real));
  }

  /// 对偶伴随叉积 ×*：(a×c + b×d) + ε(a×d)
  Dual3 crossStar(const Dual3& other) const
  {
    return Dual3(real.cross(other.real) + dual.cross(other.dual), real.cross(other.dual));
  }
};

/// M(ξ̂) = Jω + ε (m v_B)
inline Dual3 dualInertia(const Dual3& xi, const Mat3& J, double m)
{
  return Dual3(J * xi.real, m * xi.dual);
}

/// M⁻¹(ξ̂) = J⁻¹ω + ε (v_B/m)
inline Dual3 dualInertiaInv(const Dual3& xi, const Mat3& J, double m)
{
  return Dual3(J.ldlt().solve(xi.real), xi.dual / m);
}

// ---------------- 对偶四元数 ----------------

struct DQ {
  Quat real{Quat::Identity()};
  Quat dual{0.0, 0.0, 0.0, 0.0};

  DQ() = default;
  DQ(const Quat& q, const Quat& qd) : real(q), dual(qd) {}

  /// q̂ = q + ε(½ p ⊗ q)
  static DQ fromPose(const Quat& q, const Vec3& p)
  {
    const Quat pq(0.0, p.x(), p.y(), p.z());
    return DQ(q, quatScale(pq * q, 0.5));
  }

  /// 提取 (q, p)，p = 2 q' ⊗ q*
  void toPose(Quat& q, Vec3& p) const
  {
    q = real;
    p = 2.0 * (dual * real.conjugate()).vec();
  }

  /// q̂1 ⊗ q̂2 = q1q2 + ε(q1 q2' + q1' q2)
  DQ operator*(const DQ& other) const
  {
    const Eigen::Vector4d dv = (real * other.dual).coeffs() + (dual * other.real).coeffs();
    const Quat d(dv[3], dv[0], dv[1], dv[2]);  // Eigen 系数序为 (x,y,z,w)
    return DQ(real * other.real, d);
  }

  /// 逐四元数共轭（单位对偶四元数下等价于逆）
  DQ conjugate() const { return DQ(real.conjugate(), dual.conjugate()); }
};

/// exp(δ̂)，δ̂ = δω + ε δv；q = exp(δω)，p = J_l(δω) δv
inline DQ dqExp(const Dual3& screw)
{
  const Quat q = quatExp(screw.real);
  const Vec3 p = so3LeftJacobian(screw.real) * screw.dual;
  return DQ::fromPose(q, p);
}

/// log(q̂) -> δ̂ = δω + ε δv
inline Dual3 dqLog(const DQ& qh)
{
  Quat q;
  Vec3 p;
  qh.toPose(q, p);
  const Vec3 dw = quatLog(q);
  const Vec3 dv = so3LeftJacobianInv(dw) * p;
  return Dual3(dw, dv);
}

}  // namespace drone_teleop
