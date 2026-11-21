// 对偶四元数代数 / 冲击重置映射 / 恢复控制器的单元测试
// 对应 dq_reference/test_proposition1.py 的核心断言（C++ 版）
#include <gtest/gtest.h>

#include <random>

#include "drone_teleop/dq_math.hpp"
#include "drone_teleop/frame_transforms.hpp"
#include "drone_teleop/recovery_controller.hpp"
#include "drone_teleop/reset_map.hpp"

using namespace drone_teleop;

namespace {

Quat randomQuat(std::mt19937& gen)
{
  std::normal_distribution<double> n(0.0, 1.0);
  Quat q(n(gen), n(gen), n(gen), n(gen));
  return q.normalized();
}

Vec3 randomUnit(std::mt19937& gen)
{
  std::normal_distribution<double> n(0.0, 1.0);
  Vec3 v(n(gen), n(gen), n(gen));
  if (v.norm() < 1e-9) {
    v = Vec3(1, 0, 0);
  }
  return v.normalized();
}

}  // namespace

TEST(DqMath, ExpLogRoundTrip)
{
  std::mt19937 gen(7);
  std::uniform_real_distribution<double> u(-1.0, 1.0);
  for (int i = 0; i < 500; ++i) {
    Vec3 w(u(gen), u(gen), u(gen));
    w *= 1.5;  // |w| < pi，保证 quatLog 取短弧后与输入一致
    const Quat q = quatExp(w);
    const Vec3 w2 = quatLog(q);
    EXPECT_LT((w - w2).norm(), 1e-10);
  }
}

TEST(DqMath, PoseRoundTrip)
{
  std::mt19937 gen(11);
  std::normal_distribution<double> n(0.0, 1.0);
  for (int i = 0; i < 500; ++i) {
    const Quat q = randomQuat(gen);
    const Vec3 p(n(gen), n(gen), n(gen));
    const DQ qh = DQ::fromPose(q, p);
    const Dual3 screw = dqLog(qh);
    const DQ qh2 = dqExp(screw);
    Quat q2;
    Vec3 p2;
    qh2.toPose(q2, p2);
    EXPECT_LT((p - p2).norm(), 1e-10);
    EXPECT_GT(std::abs(q.dot(q2)), 1.0 - 1e-9);
  }
}

TEST(ResetMap, Proposition1Equivalence)
{
  std::mt19937 gen(2026);
  std::normal_distribution<double> n(0.0, 1.0);
  const Mat3 J = (Mat3() << 0.02167, 0.0, 0.0, 0.0, 0.02167, 0.0, 0.0, 0.0, 0.04).finished();
  const double m = 2.0643;

  double max_err = 0.0;
  for (int i = 0; i < 5000; ++i) {
    const Quat q = randomQuat(gen);
    const Vec3 r_c = 0.25 * Vec3(n(gen), n(gen), n(gen));
    const Vec3 n_w = randomUnit(gen);
    const Vec3 v_w = Vec3(n(gen), n(gen), n(gen));
    const Vec3 omega = Vec3(n(gen), n(gen), n(gen));
    const Vec3 t_w = contactTangent(v_w, omega, q, r_c, n_w);

    const double e = 0.7;
    const double mu = 0.0;  // Proposition 1 assumes the Eq.15 decoupled-friction limit.
    const ImpactState a = matrixReset(r_c, n_w, t_w, q, e, mu, omega, v_w, J, m);
    const ImpactState b = dqReset(r_c, n_w, t_w, q, e, mu, omega, v_w, J, m);

    max_err = std::max(max_err, (a.omega_plus - b.omega_plus).norm());
    max_err = std::max(max_err, (a.v_world_plus - b.v_world_plus).norm());
  }
  EXPECT_LT(max_err, 1e-10) << "Proposition 1 数值等价失败";
}

TEST(ResetMap, FrictionlessEnergyIsDissipated)
{
  std::mt19937 gen(3);
  std::normal_distribution<double> n(0.0, 1.0);
  const Mat3 J = (Mat3() << 0.02167, 0.0, 0.0, 0.0, 0.02167, 0.0, 0.0, 0.0, 0.04).finished();
  const double m = 2.0643;

  for (int i = 0; i < 500; ++i) {
    const Quat q = randomQuat(gen);
    const Vec3 r_c = 0.25 * Vec3(n(gen), n(gen), n(gen));
    const Vec3 n_w = randomUnit(gen);
    Vec3 v_w = Vec3(n(gen), n(gen), n(gen));
    const Vec3 omega = Vec3(n(gen), n(gen), n(gen));
    const Vec3 v_c0 = v_w + rotMatrix(q) * omega.cross(r_c);
    if (v_c0.dot(n_w) > -0.1) {
      v_w -= (v_c0.dot(n_w) + 0.5) * n_w;
    }
    const Vec3 t_w = contactTangent(v_w, omega, q, r_c, n_w);
    const ImpactState out = dqReset(r_c, n_w, t_w, q, 0.7, 0.0, omega, v_w, J, m);

    const double e0 = kineticEnergy(omega, quatInvRotate(q, v_w), J, m);
    const double e1 = kineticEnergy(out.omega_plus, quatInvRotate(q, out.v_world_plus), J, m);
    EXPECT_LE(e1, e0 + 1e-9) << "无摩擦碰撞后动能不应增加";
  }
}

TEST(ResetMap, NormalRestitution)
{
  std::mt19937 gen(9);
  std::normal_distribution<double> n(0.0, 1.0);
  const Mat3 J = (Mat3() << 0.02167, 0.0, 0.0, 0.0, 0.02167, 0.0, 0.0, 0.0, 0.04).finished();
  const double m = 2.0643;
  const double e = 0.7;
  const double mu = 0.0;  // 无摩擦时法向恢复系数才严格成立（有摩擦时模型会有耦合偏差）

  for (int i = 0; i < 300; ++i) {
    const Quat q = randomQuat(gen);
    const Vec3 r_c = 0.25 * Vec3(n(gen), n(gen), n(gen));
    const Vec3 n_w = randomUnit(gen);
    Vec3 v_w = Vec3(n(gen), n(gen), n(gen));
    const Vec3 omega = Vec3(n(gen), n(gen), n(gen));
    const Vec3 v_c0 = v_w + rotMatrix(q) * omega.cross(r_c);
    if (v_c0.dot(n_w) > -0.1) {
      v_w -= (v_c0.dot(n_w) + 0.5) * n_w;
    }
    const Vec3 t_w = contactTangent(v_w, omega, q, r_c, n_w);
    const ImpactState out = dqReset(r_c, n_w, t_w, q, e, mu, omega, v_w, J, m);

    const Vec3 v_c_minus = v_w + rotMatrix(q) * omega.cross(r_c);
    const Vec3 v_c_plus = out.v_world_plus + rotMatrix(q) * out.omega_plus.cross(r_c);
    EXPECT_NEAR(v_c_plus.dot(n_w), -e * v_c_minus.dot(n_w), 1e-9);
  }
}

TEST(Recovery, AdmittanceShiftFiniteAndMovesAlongVelocity)
{
  const Quat q = Quat::Identity();
  const Vec3 p(0.0, 0.0, 2.0);
  const Vec3 v_body(0.0, 0.0, 3.0);  // 向上冲
  const Vec3 omega(0.2, 0.0, 0.0);

  Quat q_d;
  Vec3 p_d;
  dqAdmittanceShift(q, p, omega, v_body, 0.05, 0.3, q_d, p_d);
  EXPECT_TRUE(q_d.coeffs().allFinite());
  EXPECT_TRUE(p_d.allFinite());
  // 让步方向与撞后速度同向（沿 +z 让出空间）
  EXPECT_GT(p_d.z(), p.z());

  Quat q_d2;
  Vec3 p_d2;
  decoupledAdmittanceShift(q, p, omega, v_body, 0.05, 0.3, q_d2, p_d2);
  EXPECT_GT(p_d2.z(), p.z());
}

TEST(Recovery, GeometricControlCancelsGravityAtHover)
{
  const Mat3 J = (Mat3() << 0.02167, 0.0, 0.0, 0.0, 0.02167, 0.0, 0.0, 0.0, 0.04).finished();
  const double m = 2.0643;
  const Quat q = Quat::Identity();
  const Vec3 p(0.0, 0.0, 1.0);
  Vec3 tau;
  Vec3 f_body;
  geometricControl(q, p, Vec3::Zero(), Vec3::Zero(), q, p, J, m, 2.0, 2.0, 0.6, 1.2, tau, f_body);
  // 悬停时机体系力应约为 +mg (FLU z 向上)
  EXPECT_NEAR(f_body.z(), m * kGravity, 1e-9);
  EXPECT_NEAR(tau.norm(), 0.0, 1e-12);
}

TEST(FrameTransforms, QuaternionRoundTrip)
{
  std::mt19937 gen(5);
  for (int i = 0; i < 200; ++i) {
    const Quat q = randomQuat(gen);
    const Quat q2 = qEnuFluToNedFrd(qNedFrdToEnuFlu(q));
    EXPECT_GT(std::abs(q.dot(q2)), 1.0 - 1e-12);
  }
}

TEST(FrameTransforms, GravityDirection)
{
  // PX4 悬停姿态(水平)在 ENU/FLU 下也应是水平；重力在 ENU 是 -z
  const Quat q_ned_frd = Quat::Identity();
  const Quat q_enu_flu = qNedFrdToEnuFlu(q_ned_frd);
  const Vec3 g_body = quatInvRotate(q_enu_flu, Vec3(0.0, 0.0, -kGravity));
  EXPECT_NEAR(g_body.z(), -kGravity, 1e-9);
}
