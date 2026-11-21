// Copyright 2026 DroneNav
// 坐标系转换：PX4 (NED / FRD) <-> 论文控制器 (ENU / FLU)
#pragma once

#include "drone_teleop/dq_math.hpp"

namespace drone_teleop {

inline const Mat3& Cne()
{
  static const Mat3 m = (Mat3() << 0.0, 1.0, 0.0,
                         1.0, 0.0, 0.0,
                         0.0, 0.0, -1.0).finished();
  return m;
}

inline Mat3 Cff() { return Vec3(1.0, -1.0, -1.0).asDiagonal(); }

inline Vec3 nedToEnu(const Vec3& v) { return Cne() * v; }
inline Vec3 enuToNed(const Vec3& v) { return Cne() * v; }  // Cne 自逆

inline Vec3 frdToFlu(const Vec3& v) { return Cff() * v; }
inline Vec3 fluToFrd(const Vec3& v) { return Cff() * v; }

/// 姿态 NED/FRD -> ENU/FLU：R_e = Cne * R_n * Cff
inline Quat qNedFrdToEnuFlu(const Quat& q_ned_frd)
{
  const Mat3 R = Cne() * q_ned_frd.toRotationMatrix() * Cff();
  return Quat(R).normalized();
}

/// 姿态 ENU/FLU -> NED/FRD（逆变换）
inline Quat qEnuFluToNedFrd(const Quat& q_enu_flu)
{
  const Mat3 R = Cne() * q_enu_flu.toRotationMatrix() * Cff();
  return Quat(R).normalized();
}

}  // namespace drone_teleop
