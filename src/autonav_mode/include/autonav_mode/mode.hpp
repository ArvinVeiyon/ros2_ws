#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rover/speed_rate.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

static const std::string kModeName = "AutoNav";

// Indoor safety envelope (R3/R5 of docs/rover_autonav_requirements.md).
// Below RO_YAW_RATE_LIM=1.57 on the FC.
static constexpr float kMaxSpeed = 0.8f;      // [m/s]
static constexpr float kMaxYawRate = 1.0f;    // [rad/s]
// cmd_vel older than this -> zero setpoint (R5.3 watchdog)
static constexpr double kCmdVelTimeout = 0.5;  // [s]

// Reflex collision-stop defaults (overridable via ROS params, "collision.*").
// This is the last line of defence: it sits inside the executor — the single
// funnel to the motors — so it applies no matter who publishes /cmd_vel
// (a test script, Nav2, a joystick). It only ever REMOVES forward motion;
// reverse and yaw are always left free so the vehicle can back off / turn away.
static constexpr double kStopDistance = 0.6;     // [m] block forward if closer than this
static constexpr double kClearDistance = 0.75;   // [m] release only once farther than this (hysteresis)
static constexpr double kSectorHalfAngle = 0.35; // [rad] +/- forward sector (~20 deg)
static constexpr double kScanTimeout = 0.5;      // [s] /scan older than this -> perception stale

class AutoNavMode : public px4_ros2::ModeBase {
 public:
  explicit AutoNavMode(rclcpp::Node& node) : ModeBase(node, kModeName), _node(node)
  {
    _rover_setpoint = std::make_shared<px4_ros2::RoverSpeedRateSetpointType>(*this);

    // --- reflex collision-stop parameters ---
    _collision_enabled = node.declare_parameter<bool>("collision.enabled", true);
    _stop_distance = node.declare_parameter<double>("collision.stop_distance", kStopDistance);
    _clear_distance = node.declare_parameter<double>("collision.clear_distance", kClearDistance);
    _sector_half = node.declare_parameter<double>("collision.sector_half_angle", kSectorHalfAngle);
    _scan_timeout = node.declare_parameter<double>("collision.scan_timeout", kScanTimeout);
    // When /scan is stale/absent: true = fail-safe (block forward, no blind driving),
    // false = permit forward with no perception (drivetrain-only bench runs).
    _require_scan = node.declare_parameter<bool>("collision.require_scan", true);

    _cmd_vel_sub = node.create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::QoS(1),
        [this](geometry_msgs::msg::Twist::UniquePtr msg) {
          _speed = std::clamp(static_cast<float>(msg->linear.x), -kMaxSpeed, kMaxSpeed);
          _yaw_rate = std::clamp(static_cast<float>(msg->angular.z), -kMaxYawRate, kMaxYawRate);
          _last_cmd_time = _node.get_clock()->now();
        });

    // Sensor data QoS (best-effort) matches typical LaserScan publishers.
    _scan_sub = node.create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::UniquePtr msg) { onScan(*msg); });

    RCLCPP_INFO(_node.get_logger(),
        "AutoNav collision-stop %s: stop<%.2fm clear>%.2fm sector=+/-%.0fdeg "
        "scan_timeout=%.2fs require_scan=%s",
        _collision_enabled ? "ON" : "OFF", _stop_distance, _clear_distance,
        _sector_half * 180.0 / M_PI, _scan_timeout, _require_scan ? "yes" : "no");

    // Passive diagnostic: reports the collision-stop decision continuously,
    // even while disarmed/inactive, so the brake can be validated on stands
    // (wheels up, NOT armed, no EKF bridge -> no limit-cycle hazard). Edge-
    // triggered: only logs on clear<->BLOCK transitions, so it is not spammy.
    _diag_timer = node.create_wall_timer(std::chrono::milliseconds(200),
        [this]() { diagTick(); });
  }

  void onActivate() override
  {
    // never reuse a stale command from a previous activation
    _speed = 0.f;
    _yaw_rate = 0.f;
    _last_cmd_time = rclcpp::Time(0, 0, _node.get_clock()->get_clock_type());
    _blocked = false;
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

    float speed = cmd_fresh ? _speed : 0.f;
    float yaw_rate = cmd_fresh ? _yaw_rate : 0.f;

    // Reflex collision-stop: only ever cancels FORWARD motion. Reverse/yaw pass through.
    if (_collision_enabled && speed > 0.f && forwardBlocked()) {
      RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
          "collision-stop: forward blocked (front=%.2fm)", _front_min_range);
      speed = 0.f;
    }

    _rover_setpoint->update(speed, yaw_rate);
  }

 private:
  // Update the nearest valid return inside the forward sector.
  void onScan(const sensor_msgs::msg::LaserScan& scan)
  {
    float min_r = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float ang = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
      if (ang < -_sector_half || ang > _sector_half) {
        continue;
      }
      const float r = scan.ranges[i];
      if (!std::isfinite(r) || r <= 0.f || r < scan.range_min || r > scan.range_max) {
        continue;  // 0 / inf / nan / out-of-spec are not valid obstacles
      }
      min_r = std::min(min_r, r);
    }
    _front_min_range = min_r;  // inf => nothing detected in sector
    _last_scan_time = _node.get_clock()->now();
  }

  // Hysteretic forward-block decision. Fail-safe when perception is stale.
  bool forwardBlocked()
  {
    const bool scan_fresh = _last_scan_time.nanoseconds() > 0 &&
        (_node.get_clock()->now() - _last_scan_time).seconds() < _scan_timeout;
    if (!scan_fresh) {
      // No trustworthy perception: fail-safe blocks forward unless explicitly permitted.
      return _require_scan;
    }
    if (_front_min_range < _stop_distance) {
      _blocked = true;
    } else if (_front_min_range > _clear_distance) {
      _blocked = false;
    }  // between stop and clear: hold previous state
    return _blocked;
  }

  // Passive, stateless view of the block decision for on-stands validation.
  // Uses the raw stop_distance (no hysteresis) so the flip point is a clean
  // ~stop_distance in both directions, easy to read while waving a wall.
  void diagTick()
  {
    const bool scan_fresh = _last_scan_time.nanoseconds() > 0 &&
        (_node.get_clock()->now() - _last_scan_time).seconds() < _scan_timeout;
    const bool would_block = scan_fresh ? (_front_min_range < _stop_distance)
                                        : _require_scan;
    if (!_diag_inited || would_block != _diag_last) {
      RCLCPP_INFO(_node.get_logger(),
          "collision-diag: %s  (scan_fresh=%s front=%.2fm)",
          would_block ? "BLOCK forward" : "clear",
          scan_fresh ? "yes" : "no", _front_min_range);
      _diag_last = would_block;
      _diag_inited = true;
    }
  }

  rclcpp::Node& _node;
  std::shared_ptr<px4_ros2::RoverSpeedRateSetpointType> _rover_setpoint;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr _scan_sub;

  float _speed{0.f};
  float _yaw_rate{0.f};
  rclcpp::Time _last_cmd_time{0, 0, RCL_ROS_TIME};

  // collision-stop config
  bool _collision_enabled{true};
  bool _require_scan{true};
  double _stop_distance{kStopDistance};
  double _clear_distance{kClearDistance};
  double _sector_half{kSectorHalfAngle};
  double _scan_timeout{kScanTimeout};

  // collision-stop state
  float _front_min_range{std::numeric_limits<float>::infinity()};
  rclcpp::Time _last_scan_time{0, 0, RCL_ROS_TIME};
  bool _blocked{false};

  // passive on-stands diagnostic
  rclcpp::TimerBase::SharedPtr _diag_timer;
  bool _diag_last{false};
  bool _diag_inited{false};
};
