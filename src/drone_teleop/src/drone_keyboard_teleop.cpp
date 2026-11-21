// Copyright 2026 DroneNav
//
// Keyboard input adapter for PX4 Offboard position control.
// PX4 owns flight control; this node only publishes TrajectorySetpoint.

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>

namespace {
enum class FlightMode { kIdle, kManual, kTakeoff, kLand };
}

class KeyboardTeleop : public rclcpp::Node
{
public:
  KeyboardTeleop() : rclcpp::Node("keyboard_teleop")
  {
    max_speed_ = declare_parameter("max_speed", 1.5);
    max_vert_ = declare_parameter("max_vert", 0.8);
    yaw_rate_ = declare_parameter("yaw_rate", 0.8);
    takeoff_height_ = declare_parameter("takeoff_height", 1.2);
    land_speed_ = declare_parameter("land_speed", 0.5);
    force_arm_ = declare_parameter("force_arm", false);

    rmw_qos_profile_t qos = rmw_qos_profile_sensor_data;
    auto sub_qos = rclcpp::QoS(rclcpp::QoSInitialization(qos.history, qos.depth), qos);

    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      "/fmu/out/vehicle_odometry", sub_qos,
      [this](const px4_msgs::msg::VehicleOdometry::SharedPtr m) {
        if (m->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) return;
        position_ = {m->position[0], m->position[1], m->position[2]};
        velocity_odom_ = {m->velocity[0], m->velocity[1], m->velocity[2]};
        if (!odom_valid_) {
          target_position_ = position_;
          target_yaw_ = yawFromQuaternion(m->q);
          current_yaw_ = target_yaw_;
        }
        current_yaw_ = yawFromQuaternion(m->q);
        odom_valid_ = true;
        odom_stamp_ = now();
      });

    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      "/fmu/out/vehicle_status_v1", sub_qos,
      [this](const px4_msgs::msg::VehicleStatus::SharedPtr m) {
        armed_ = m->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
        nav_state_ = m->nav_state;
      });

    land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      "/fmu/out/vehicle_land_detected", sub_qos,
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr m) {
        if (mode_ == FlightMode::kLand && m->landed) {
          sendDisarm();
          mode_ = FlightMode::kIdle;
          target_position_ = position_;
          pending_arm_offboard_ = false;
          pending_takeoff_arm_ = false;
          printMsg("PX4 检测落地，上锁");
        }
      });

    recovery_attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitudeSetpoint>(
      "/recovery/setpoint", sub_qos,
      [this](const px4_msgs::msg::VehicleAttitudeSetpoint::SharedPtr m) {
        recovery_attitude_ = *m;
        recovery_attitude_stamp_ = now();
      });

    recovery_thrust_sub_ = create_subscription<px4_msgs::msg::VehicleThrustSetpoint>(
      "/recovery/thrust_setpoint", sub_qos,
      [this](const px4_msgs::msg::VehicleThrustSetpoint::SharedPtr m) {
        recovery_thrust_ = *m;
        recovery_thrust_stamp_ = now();
      });

    recovery_torque_sub_ = create_subscription<px4_msgs::msg::VehicleTorqueSetpoint>(
      "/recovery/torque_setpoint", sub_qos,
      [this](const px4_msgs::msg::VehicleTorqueSetpoint::SharedPtr m) {
        recovery_torque_ = *m;
        recovery_torque_stamp_ = now();
      });

    recovery_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/recovery/status", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr m) {
        if (m->data.rfind("RECOVER started", 0) == 0) {
          recovery_takeover_active_ = true;
          recovery_status_stamp_ = now();
          recovery_cooldown_until_ = now();
          velocity_.fill(0.0);
          yaw_rate_cmd_ = 0.0;
          mode_ = FlightMode::kManual;
          target_position_ = position_;
          target_yaw_ = current_yaw_;
        } else if (m->data.rfind("RECOVER done", 0) == 0) {
          recovery_takeover_active_ = false;
          recovery_status_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          recovery_cooldown_until_ = now() + rclcpp::Duration::from_seconds(2.0);
          recovery_attitude_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          recovery_thrust_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          recovery_torque_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
          velocity_.fill(0.0);
          yaw_rate_cmd_ = 0.0;
          mode_ = FlightMode::kManual;
          target_position_ = position_;
          target_yaw_ = current_yaw_;
          printMsg("碰撞恢复完成，悬停冷却 2 s");
        }
      });

    reset_sub_ = create_subscription<std_msgs::msg::String>(
      "/mujoco/reset", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr) {
        mode_ = FlightMode::kIdle;
        pending_arm_offboard_ = false;
        pending_takeoff_arm_ = false;
        takeoff_ground_valid_ = false;
        recovery_takeover_active_ = false;
        recovery_cooldown_until_ = now();
        recovery_status_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        velocity_.fill(0.0);
        yaw_rate_cmd_ = 0.0;
        target_position_ = position_;
        target_yaw_ = current_yaw_;
        printMsg("仿真已 Reset");
      });

    rmw_qos_profile_t pub_qos = rmw_qos_profile_sensor_data;
    heartbeat_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      "/fmu/in/offboard_control_mode",
      rclcpp::QoS(rclcpp::QoSInitialization(pub_qos.history, pub_qos.depth), pub_qos));
    trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint",
      rclcpp::QoS(rclcpp::QoSInitialization(pub_qos.history, pub_qos.depth), pub_qos));
    attitude_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      "/fmu/in/vehicle_attitude_setpoint_v1",
      rclcpp::QoS(rclcpp::QoSInitialization(pub_qos.history, pub_qos.depth), pub_qos));
    thrust_pub_ = create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
      "/fmu/in/vehicle_thrust_setpoint",
      rclcpp::QoS(rclcpp::QoSInitialization(pub_qos.history, pub_qos.depth), pub_qos));
    torque_pub_ = create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
      "/fmu/in/vehicle_torque_setpoint",
      rclcpp::QoS(rclcpp::QoSInitialization(pub_qos.history, pub_qos.depth), pub_qos));
    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", rclcpp::QoS(1));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/drone_teleop/keyboard_status", 5);

    terminal_ok_ = setupTerminal();
    if (terminal_ok_) printBanner();
    else RCLCPP_WARN(get_logger(), "stdin 不是终端，键盘输入不可用");

    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               [this]() { tick(); });
  }

  ~KeyboardTeleop() override { restoreTerminal(); }

private:
  bool setupTerminal()
  {
    tty_fd_ = STDIN_FILENO;
    if (tcgetattr(tty_fd_, &orig_) < 0) {
      tty_fd_ = open("/dev/tty", O_RDWR | O_NOCTTY);
      if (tty_fd_ < 0 || tcgetattr(tty_fd_, &orig_) < 0) {
        tty_fd_ = -1;
        return false;
      }
    }
    termios raw = orig_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    return tcsetattr(tty_fd_, TCSANOW, &raw) == 0;
  }

  void restoreTerminal()
  {
    if (!terminal_ok_ || tty_fd_ < 0) return;
    tcsetattr(tty_fd_, TCSANOW, &orig_);
    if (tty_fd_ != STDIN_FILENO) close(tty_fd_);
    tty_fd_ = -1;
    terminal_ok_ = false;
  }

  void pollKeys()
  {
    if (!terminal_ok_) return;
    char c;
    while (tty_fd_ >= 0 && read(tty_fd_, &c, 1) == 1) handleKey(c);
  }

  void handleKey(char c)
  {
    if (c == 'o') {
      velocity_.fill(0.0);
      yaw_rate_cmd_ = 0.0;
      mode_ = FlightMode::kIdle;
      target_position_ = position_;
      target_yaw_ = current_yaw_;
      pending_arm_offboard_ = false;
      pending_takeoff_arm_ = false;
      sendOffboard();
      printMsg("切换 OFFBOARD（未解锁）；按 t 解锁并起飞");
      return;
    }
    if (c == 'x') {
      pending_arm_offboard_ = false;
      pending_takeoff_arm_ = false;
      mode_ = FlightMode::kIdle;
      velocity_.fill(0.0);
      yaw_rate_cmd_ = 0.0;
      target_position_ = position_;
      target_yaw_ = current_yaw_;
      sendDisarm();
      printMsg("上锁");
      return;
    }
    if (c == 27 || c == 3) {
      std::printf("\r\033[K  已退出键盘遥控\n");
      restoreTerminal();
      rclcpp::shutdown();
      return;
    }
    if (recovery_takeover_active_ || directRecoveryFresh() || attitudeRecoveryFresh() ||
        now() < recovery_cooldown_until_) {
      velocity_.fill(0.0);
      yaw_rate_cmd_ = 0.0;
      target_position_ = position_;
      target_yaw_ = current_yaw_;
      printMsg("碰撞恢复接管中，当前按键已忽略");
      return;
    }
    if (c == 't') {
      mode_ = FlightMode::kTakeoff;
      velocity_.fill(0.0);
      yaw_rate_cmd_ = 0.0;
      target_position_[0] = position_[0];
      target_position_[1] = position_[1];
      takeoff_ground_z_ = position_[2];
      takeoff_ground_valid_ = true;
      target_position_[2] = takeoff_ground_z_ - takeoff_height_;

      if (nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        pending_arm_offboard_ = true;
        pending_takeoff_arm_ = true;
        sendOffboard();
        printMsg("切换 OFFBOARD 并解锁起飞到 %.1f m", takeoff_height_);
      } else if (!armed_) {
        pending_takeoff_arm_ = true;
        sendArm();
        printMsg("解锁并起飞到 %.1f m", takeoff_height_);
      } else {
        printMsg("起飞到 %.1f m", takeoff_height_);
      }
      key_stamp_ = now();
      return;
    }
    if (recovery_takeover_active_ || directRecoveryFresh() || attitudeRecoveryFresh() ||
        now() < recovery_cooldown_until_) {
      velocity_.fill(0.0);
      yaw_rate_cmd_ = 0.0;
      target_position_ = position_;
      target_yaw_ = current_yaw_;
      printMsg("碰撞恢复接管中，当前按键已忽略");
      return;
    }
    if (!armed_) {
      printMsg("无人机未解锁；按 t 解锁并起飞");
      return;
    }

    key_stamp_ = now();
    switch (c) {
      case 'w':
        mode_ = FlightMode::kManual;
        velocity_[0] = max_speed_;
        break;
      case 's':
        mode_ = FlightMode::kManual;
        velocity_[0] = -max_speed_;
        break;
      case 'a':
        mode_ = FlightMode::kManual;
        velocity_[1] = -max_speed_;
        break;
      case 'd':
        mode_ = FlightMode::kManual;
        velocity_[1] = max_speed_;
        break;
      case 'r':
        mode_ = FlightMode::kManual;
        velocity_[2] = -max_vert_;
        break;
      case 'f':
        mode_ = FlightMode::kManual;
        velocity_[2] = max_vert_;
        break;
      case 'q':
        mode_ = FlightMode::kManual;
        yaw_rate_cmd_ = -yaw_rate_;
        break;
      case 'e':
        mode_ = FlightMode::kManual;
        yaw_rate_cmd_ = yaw_rate_;
        break;
      case ' ':
        mode_ = FlightMode::kManual;
        velocity_.fill(0.0);
        yaw_rate_cmd_ = 0.0;
        target_position_ = position_;
        target_yaw_ = current_yaw_;
        printMsg("悬停");
        break;
      case 'l':
        if (mode_ != FlightMode::kLand) {
          land_ground_z_ = takeoff_ground_valid_ ? takeoff_ground_z_ : position_[2];
          sendLand();
          last_land_command_time_ = now();
        }
        mode_ = FlightMode::kLand;
        velocity_.fill(0.0);
        yaw_rate_cmd_ = 0.0;
        pending_takeoff_arm_ = false;
        printMsg("降落");
        break;
      default:
        break;
    }
  }

  uint64_t nowUs() const
  {
    return static_cast<uint64_t>(now().nanoseconds() / 1000);
  }

  void sendCommand(uint16_t command, float p1 = 0.0f, float p2 = 0.0f)
  {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.timestamp = nowUs();
    cmd.command = command;
    cmd.param1 = p1;
    cmd.param2 = p2;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    command_pub_->publish(cmd);
  }

  void sendOffboard()
  {
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    last_command_time_ = now();
  }
  void sendLand()
  {
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  }
  void sendArm()
  {
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f,
                force_arm_ ? 21196.0f : 0.0f);
    last_arm_command_time_ = now();
  }
  void sendDisarm()
  {
    sendCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
  }

  bool directRecoveryFresh() const
  {
    return (now() - recovery_thrust_stamp_).seconds() < 0.2 &&
           (now() - recovery_torque_stamp_).seconds() < 0.2;
  }
  bool attitudeRecoveryFresh() const
  {
    return (now() - recovery_attitude_stamp_).seconds() < 0.2;
  }

  void publishHeartbeat()
  {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.timestamp = nowUs();
    if (directRecoveryFresh()) msg.direct_actuator = true;
    else if (attitudeRecoveryFresh()) msg.attitude = true;
    else msg.position = true;
    heartbeat_pub_->publish(msg);
  }

  void publishRecovery()
  {
    if (directRecoveryFresh()) {
      auto thrust = recovery_thrust_;
      thrust.timestamp = nowUs();
      thrust.timestamp_sample = nowUs();
      thrust_pub_->publish(thrust);
      auto torque = recovery_torque_;
      torque.timestamp = nowUs();
      torque.timestamp_sample = nowUs();
      torque_pub_->publish(torque);
    } else if (attitudeRecoveryFresh()) {
      auto attitude = recovery_attitude_;
      attitude.timestamp = nowUs();
      attitude_pub_->publish(attitude);
    }
  }

  void publishTrajectory()
  {
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = nowUs();
    msg.position = {static_cast<float>(target_position_[0]),
                    static_cast<float>(target_position_[1]),
                    static_cast<float>(target_position_[2])};
    msg.velocity = {NAN, NAN, NAN};
    msg.acceleration = {NAN, NAN, NAN};
    msg.jerk = {NAN, NAN, NAN};
    msg.yaw = static_cast<float>(target_yaw_);
    msg.yawspeed = NAN;
    trajectory_pub_->publish(msg);
  }

  double yawFromQuaternion(const std::array<float, 4>& q) const
  {
    return std::atan2(2.0 * (q[0] * q[3] + q[1] * q[2]),
                      1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
  }

  void printBanner()
  {
    std::printf("\033[2J\033[H");
    std::printf("+--------------------------------------------------------------+\n");
    std::printf("|          DroneNav 键盘遥控（PX4 Offboard）                   |\n");
    std::printf("+--------------------------------------------------------------+\n");
    std::printf("| w/s 前后  a/d 左右  r/f 升降  q/e 转向                     |\n");
    std::printf("| 空格 悬停  t 解锁起飞  l 降落  o OFFBOARD  x 上锁  Esc退出   |\n");
    std::printf("+--------------------------------------------------------------+\n");
    std::fflush(stdout);
  }

  void printMsg(const char* fmt, ...)
  {
    static rclcpp::Time last_recovery_ignore_message_{0, 0, RCL_ROS_TIME};
    char buf[256];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (std::strstr(buf, "碰撞恢复接管中") != nullptr &&
        now() - last_recovery_ignore_message_ < rclcpp::Duration::from_seconds(1.0)) {
      return;
    }
    if (std::strstr(buf, "碰撞恢复接管中") != nullptr) {
      last_recovery_ignore_message_ = now();
    }
    std::printf("\r\033[K  > %s\n", buf);
    std::fflush(stdout);
  }

  void tick()
  {
    pollKeys();
    publishHeartbeat();

    if (!odom_valid_ || (now() - odom_stamp_).seconds() > 0.5) {
      return;
    }

    if (recovery_takeover_active_ &&
        recovery_status_stamp_.nanoseconds() > 0 &&
        (now() - recovery_status_stamp_).seconds() > 1.0 &&
        !directRecoveryFresh() && !attitudeRecoveryFresh()) {
      recovery_takeover_active_ = false;
      recovery_cooldown_until_ = now();
    }

    if (directRecoveryFresh() || attitudeRecoveryFresh()) {
      recovery_takeover_active_ = true;
      recovery_cooldown_until_ = now();
      velocity_.fill(0.0);
      yaw_rate_cmd_ = 0.0;
      target_position_ = position_;
      target_yaw_ = current_yaw_;
      publishRecovery();
      return;
    }

    if (recovery_takeover_active_ || now() < recovery_cooldown_until_) {
      velocity_.fill(0.0);
      yaw_rate_cmd_ = 0.0;
      target_position_ = position_;
      target_yaw_ = current_yaw_;
      publishTrajectory();
      return;
    }

    if (pending_arm_offboard_) {
      if (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
        if (pending_takeoff_arm_ && !armed_) {
          if ((now() - last_arm_command_time_).seconds() > 1.0) {
            sendArm();
          }
        } else if (!pending_takeoff_arm_ || armed_) {
          pending_arm_offboard_ = false;
          if (armed_) pending_takeoff_arm_ = false;
        }
      } else if ((now() - last_command_time_).seconds() > 1.0) {
        sendOffboard();
      }
    }

    constexpr double dt = 0.02;
    if ((now() - key_stamp_).seconds() > 0.35) {
      velocity_.fill(0.0);
      yaw_rate_cmd_ = 0.0;
    }

    if (mode_ == FlightMode::kManual) {
      // velocity_[0] is FRD forward and velocity_[1] is FRD right.
      // PX4 TrajectorySetpoint.position is NED: map heading-relative motion
      // to North/East before integrating the target.
      const double forward = velocity_[0];
      const double right = velocity_[1];
      target_position_[0] += (std::cos(current_yaw_) * forward -
                              std::sin(current_yaw_) * right) * dt;
      target_position_[1] += (std::sin(current_yaw_) * forward +
                              std::cos(current_yaw_) * right) * dt;
      target_position_[2] += velocity_[2] * dt;
      target_yaw_ += yaw_rate_cmd_ * dt;
      target_yaw_ = std::remainder(target_yaw_, 2.0 * M_PI);
    } else if (mode_ == FlightMode::kTakeoff &&
               position_[2] <= target_position_[2] + 0.15) {
      mode_ = FlightMode::kManual;
      velocity_.fill(0.0);
      printMsg("起飞完成");
    } else if (mode_ == FlightMode::kLand) {
      if ((now() - last_land_command_time_).seconds() > 2.0 &&
          nav_state_ != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LAND) {
        sendLand();
        last_land_command_time_ = now();
      }
      target_position_[2] += land_speed_ * dt;
      if (position_[2] >= land_ground_z_ - 0.05 &&
          std::fabs(velocity_odom_[2]) < 0.12 &&
          std::fabs(velocity_odom_[0]) < 0.25 &&
          std::fabs(velocity_odom_[1]) < 0.25) {
        sendDisarm();
        mode_ = FlightMode::kIdle;
        target_position_ = position_;
        pending_arm_offboard_ = false;
        pending_takeoff_arm_ = false;
        printMsg("已到地面，上锁");
      }
    }

    publishTrajectory();
  }

  double max_speed_{1.5};
  double max_vert_{0.8};
  double yaw_rate_{0.8};
  double takeoff_height_{1.2};
  double land_speed_{0.5};
  double takeoff_ground_z_{0.0};
  double land_ground_z_{0.0};
  double target_yaw_{0.0};
  double yaw_rate_cmd_{0.0};
  bool force_arm_{false};
  bool odom_valid_{false};
  bool armed_{false};
  bool takeoff_ground_valid_{false};
  bool pending_arm_offboard_{false};
  bool pending_takeoff_arm_{false};
  bool recovery_takeover_active_{false};
  uint8_t nav_state_{0};

  std::array<float, 3> position_{0, 0, 0};
  std::array<float, 3> target_position_{0, 0, 0};
  std::array<float, 3> velocity_{0, 0, 0};
  std::array<float, 3> velocity_odom_{0, 0, 0};
  double current_yaw_{0.0};

  rclcpp::Time odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time key_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_arm_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_land_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time recovery_attitude_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time recovery_thrust_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time recovery_torque_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time recovery_cooldown_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Time recovery_status_stamp_{0, 0, RCL_ROS_TIME};

  px4_msgs::msg::VehicleAttitudeSetpoint recovery_attitude_{};
  px4_msgs::msg::VehicleThrustSetpoint recovery_thrust_{};
  px4_msgs::msg::VehicleTorqueSetpoint recovery_torque_{};
  FlightMode mode_{FlightMode::kIdle};
  termios orig_{};
  int tty_fd_{-1};
  bool terminal_ok_{false};

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr recovery_attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr recovery_thrust_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr recovery_torque_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr recovery_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reset_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr attitude_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardTeleop>());
  rclcpp::shutdown();
  return 0;
}
