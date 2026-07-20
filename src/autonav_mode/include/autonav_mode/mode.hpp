#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rover/speed_rate.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>

static const std::string kModeName = "AutoNav";

// Indoor safety envelope (R3/R5 of docs/rover_autonav_requirements.md).
// Below RO_YAW_RATE_LIM=1.57 on the FC.
static constexpr float kMaxSpeed = 0.8f;      // [m/s]
static constexpr float kMaxYawRate = 1.0f;    // [rad/s]
// cmd_vel older than this -> zero setpoint (R5.3 watchdog)
static constexpr double kCmdVelTimeout = 0.5;  // [s]

class AutoNavMode : public px4_ros2::ModeBase {
 public:
  explicit AutoNavMode(rclcpp::Node& node) : ModeBase(node, kModeName), _node(node)
  {
    _rover_setpoint = std::make_shared<px4_ros2::RoverSpeedRateSetpointType>(*this);

    _cmd_vel_sub = node.create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(1),
        [this](geometry_msgs::msg::Twist::UniquePtr msg) {
          _speed = std::clamp(static_cast<float>(msg->linear.x), -kMaxSpeed, kMaxSpeed);
          _yaw_rate = std::clamp(static_cast<float>(msg->angular.z), -kMaxYawRate, kMaxYawRate);
          _last_cmd_time = _node.get_clock()->now();
        });
  }

  void onActivate() override
  {
    // never reuse a stale command from a previous activation
    _speed = 0.f;
    _yaw_rate = 0.f;
    _last_cmd_time = rclcpp::Time(0, 0, _node.get_clock()->get_clock_type());
    RCLCPP_INFO(_node.get_logger(), "AutoNav activated — holding zero until /cmd_vel arrives");
  }

  void onDeactivate() override
  {
    RCLCPP_INFO(_node.get_logger(), "AutoNav deactivated");
  }

  void updateSetpoint(float /*dt_s*/) override
  {
    const bool cmd_fresh = _last_cmd_time.nanoseconds() > 0 &&
        (_node.get_clock()->now() - _last_cmd_time).seconds() < kCmdVelTimeout;

    if (cmd_fresh) {
      _rover_setpoint->update(_speed, _yaw_rate);
    } else {
      _rover_setpoint->update(0.f, 0.f);
    }
  }

 private:
  rclcpp::Node& _node;
  std::shared_ptr<px4_ros2::RoverSpeedRateSetpointType> _rover_setpoint;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;

  float _speed{0.f};
  float _yaw_rate{0.f};
  rclcpp::Time _last_cmd_time{0, 0, RCL_ROS_TIME};
};
