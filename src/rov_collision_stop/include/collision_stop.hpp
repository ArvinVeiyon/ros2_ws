#pragma once
/****************************************************************************
 * rov_collision_stop — TFmini-based throttle gate for Rover (smooth & fast)
 *
 * INPUTS:
 *  - /fmu/in/distance_sensor          (px4_msgs/DistanceSensor)
 *  - /fmu/out/manual_control_setpoint (px4_msgs/ManualControlSetpoint)
 *
 * OUTPUTS:
 *  - /fmu/in/rover_throttle_setpoint  (px4_msgs/RoverThrottleSetpoint)
 *  - /fmu/in/rover_steering_setpoint  (px4_msgs/RoverSteeringSetpoint)
 *
 * CHANGES (as requested):
 *  - Reverse is ALWAYS allowed (even when blocked / very close).
 *  - Slow-zone uses a FLOOR so forward throttle never collapses to zero
 *    inside the slow zone. New param: forward_min_scale (default 0.35).
 *  - “Hard stop” now applies to forward only; reverse still works.
 *
 * Tuning (indoor defaults):
 *  - stop_threshold_m = 0.50, clear_threshold_m = 0.70, slow_zone_m = 0.90
 *  - forward_min_scale = 0.35  (floor of forward throttle factor in slow zone)
 *  - hard_stop_m = 0.35        (forward instantly zero below this)
 *  - control_hz = 50
 *  - accel_limit_per_s = 1.5, decel_limit_per_s = 3.0
 *  - tfmini_timeout_ms = 0  (disabled)
 *  - brake_hold_ms = 0      (disabled)
 ****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/rover_throttle_setpoint.hpp>
#include <px4_msgs/msg/rover_steering_setpoint.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>
#include <algorithm>
#include <cmath>
#include <chrono>

class CollisionStopNode : public rclcpp::Node
{
public:
  CollisionStopNode() : Node("rov_collision_stop")
  {
    // --- Params (defaults match indoor) ---
    enabled_            = declare_parameter<bool>("enabled", true);
    stop_threshold_m_   = declare_parameter<double>("stop_threshold_m", 0.50);
    clear_threshold_m_  = declare_parameter<double>("clear_threshold_m", 0.70);
    slow_zone_m_        = declare_parameter<double>("slow_zone_m", 0.90);  // ≥ clear to enable slow-down
    forward_min_scale_  = std::clamp(declare_parameter<double>("forward_min_scale", 0.35), 0.0, 1.0);
    hard_stop_m_        = declare_parameter<double>("hard_stop_m", 0.35);  // forward-only “slam to zero”
    control_hz_         = std::max<int>(10, declare_parameter<int>("control_hz", 50));
    tfmini_timeout_ms_  = std::max<int>(0, declare_parameter<int>("tfmini_timeout_ms", 0));  // 0 = disabled
    brake_hold_ms_      = std::max<int>(0, declare_parameter<int>("brake_hold_ms", 0));      // 0 = disabled

    // Slew limits (per second). Set to 0 to disable smoothing.
    accel_limit_per_s_  = std::max<double>(0.0, declare_parameter<double>("accel_limit_per_s", 1.5)); // up ramp
    decel_limit_per_s_  = std::max<double>(0.0, declare_parameter<double>("decel_limit_per_s", 3.0)); // down ramp

    if (clear_threshold_m_ <= stop_threshold_m_) {
      clear_threshold_m_ = stop_threshold_m_ + 0.10;
      RCLCPP_WARN(get_logger(), "Adjusted clear_threshold_m to %.2f (> stop %.2f).",
                  clear_threshold_m_, stop_threshold_m_);
    }
    if (slow_zone_m_ < clear_threshold_m_) {
      slow_zone_m_ = clear_threshold_m_; // disables slow-down
    }
    if (hard_stop_m_ > stop_threshold_m_) {
      hard_stop_m_ = stop_threshold_m_;
    }

    auto be = rclcpp::QoS(10).best_effort();

    // --- Pubs ---
    thr_pub_ = create_publisher<px4_msgs::msg::RoverThrottleSetpoint>("/fmu/in/rover_throttle_setpoint", be);
    str_pub_ = create_publisher<px4_msgs::msg::RoverSteeringSetpoint>("/fmu/in/rover_steering_setpoint", be);

    // --- Subs ---
    manual_sub_ = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
      "/fmu/out/manual_control_setpoint", be,
      std::bind(&CollisionStopNode::onManual, this, std::placeholders::_1));

    dist_sub_ = create_subscription<px4_msgs::msg::DistanceSensor>(
      "/fmu/in/distance_sensor", be,
      std::bind(&CollisionStopNode::onDistance, this, std::placeholders::_1));

    // --- Control loop ---
    ctrl_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / control_hz_),
      std::bind(&CollisionStopNode::controlTick, this));

    // --- Param updates at runtime ---
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& ps){
        for (const auto &p : ps) {
          const auto &n = p.get_name();
          if      (n == "enabled")            enabled_ = p.as_bool();
          else if (n == "stop_threshold_m")   stop_threshold_m_ = p.as_double();
          else if (n == "clear_threshold_m")  clear_threshold_m_ = p.as_double();
          else if (n == "slow_zone_m")        slow_zone_m_ = p.as_double();
          else if (n == "forward_min_scale")  forward_min_scale_ = std::clamp(p.as_double(), 0.0, 1.0);
          else if (n == "hard_stop_m")        hard_stop_m_ = p.as_double();
          else if (n == "control_hz")         control_hz_ = std::max<int>(10, p.as_int());
          else if (n == "tfmini_timeout_ms")  tfmini_timeout_ms_ = std::max<int>(0, p.as_int());
          else if (n == "brake_hold_ms")      brake_hold_ms_ = std::max<int>(0, p.as_int());
          else if (n == "accel_limit_per_s")  accel_limit_per_s_ = std::max<double>(0.0, p.as_double());
          else if (n == "decel_limit_per_s")  decel_limit_per_s_ = std::max<double>(0.0, p.as_double());
        }
        if (clear_threshold_m_ <= stop_threshold_m_) {
          clear_threshold_m_ = stop_threshold_m_ + 0.10;
          RCLCPP_WARN(get_logger(), "Adjusted clear_threshold_m to %.2f to keep hysteresis.", clear_threshold_m_);
        }
        if (slow_zone_m_ < clear_threshold_m_)  slow_zone_m_ = clear_threshold_m_;
        if (hard_stop_m_ > stop_threshold_m_)   hard_stop_m_ = stop_threshold_m_;

        ctrl_timer_->cancel();
        ctrl_timer_ = create_wall_timer(
          std::chrono::milliseconds(1000 / control_hz_),
          std::bind(&CollisionStopNode::controlTick, this));

        rcl_interfaces::msg::SetParametersResult r; r.successful = true; return r;
      });

    RCLCPP_INFO(get_logger(),
      "rov_collision_stop started (enabled=%s) stop=%.2fm clear=%.2fm slow=%.2fm floor=%.2f hard_stop=%.2fm ctrl=%dHz timeout=%dms hold=%dms accel=%.2f/s decel=%.2f/s",
      enabled_ ? "true" : "false", stop_threshold_m_, clear_threshold_m_, slow_zone_m_, forward_min_scale_,
      hard_stop_m_, control_hz_, tfmini_timeout_ms_, brake_hold_ms_, accel_limit_per_s_, decel_limit_per_s_);
  }

private:
  // Distance callback → update distance state
  void onDistance(const px4_msgs::msg::DistanceSensor::SharedPtr msg)
  {
    last_distance_m_  = msg->current_distance;
    last_distance_ts_ = now();

    // Update block state with hysteresis
    if (last_distance_m_ > msg->min_distance && last_distance_m_ < stop_threshold_m_) {
      obstacle_blocked_ = true;
      last_block_ts_ = now();
    } else if (last_distance_m_ > clear_threshold_m_ && last_distance_m_ < msg->max_distance) {
      obstacle_blocked_ = false;
      last_clear_ts_ = now();
    }

    // Track very-close zone for forward-only hard stop (handled in gateThrottle)
    very_close_ = (last_distance_m_ > msg->min_distance && last_distance_m_ < hard_stop_m_);
  }

  // Manual input → passthrough steering, throttle w/ gating + smoothing
  void onManual(const px4_msgs::msg::ManualControlSetpoint::SharedPtr m)
  {
    last_manual_ = *m;
    last_manual_ts_ = now();

    if (!enabled_) return;

    publishSteering(m->roll);

    float desired = gateThrottle(m->throttle);  // reverse always allowed; forward gated
    desired = applySlew(desired);               // smooth steps
    publishThrottle(desired);
  }

  // Enforcement loop
  void controlTick()
  {
    if (!enabled_) return;

    if (isTimedOut()) {
      last_cmd_throttle_ = 0.0f;
      publishThrottle(0.0f);
      return;
    }

    if (last_manual_ts_.nanoseconds() > 0) {
      publishSteering(last_manual_.roll);
      float desired = gateThrottle(last_manual_.throttle);
      desired = applySlew(desired);
      publishThrottle(desired);
    }
  }

  // -------- Helpers --------
  // Throttle gating:
  //  - Reverse (thr<0) always allowed (bypass all stops).
  //  - Forward: hard stop if very_close_, stop if blocked/hold, slow in slow zone with floor.
  float gateThrottle(float rc_throttle) const
  {
    float thr = std::clamp(rc_throttle, -1.0f, 1.0f);

    // Reverse is always allowed
    if (thr < 0.0f) return thr;

    // Optional timeout stop
    if (tfmini_timeout_ms_ > 0 && isTimedOut()) return 0.0f;

    // Forward-only hard stop when extremely close
    if (very_close_) return 0.0f;

    // Blocked or brake-hold → forward zero
    if (obstacle_blocked_ || isHoldingBrake()) return 0.0f;

    // Slow-down in slow zone with a floor (forward_min_scale_)
    if (slow_zone_m_ > clear_threshold_m_ && last_distance_m_ > 0.0) {
      if (last_distance_m_ <= slow_zone_m_ && last_distance_m_ >= clear_threshold_m_) {
        const double span = slow_zone_m_ - clear_threshold_m_;
        const double ratio = (last_distance_m_ - clear_threshold_m_) / (span > 1e-6 ? span : 1.0);
        // scale ramps from floor at clear_threshold -> 1.0 at slow_zone
        const double scale = forward_min_scale_ + (1.0 - forward_min_scale_) * std::clamp(ratio, 0.0, 1.0);
        thr = thr * static_cast<float>(std::clamp(scale, forward_min_scale_, 1.0));
      }
    }
    return thr;
  }

  // Slew limiter (per second)
  float applySlew(float desired)
  {
    const rclcpp::Time tnow = now();
    const double dt = last_cmd_stamp_.nanoseconds() > 0
                      ? (tnow - last_cmd_stamp_).nanoseconds() / 1e9
                      : 0.0;
    last_cmd_stamp_ = tnow;

    if (dt <= 0.0 || (accel_limit_per_s_ <= 0.0 && decel_limit_per_s_ <= 0.0)) {
      last_cmd_throttle_ = desired;
      return desired;
    }

    const float delta = desired - last_cmd_throttle_;
    double limit = 0.0;
    if (delta > 0.0f)      limit =  accel_limit_per_s_ * dt;  // accelerating forward
    else if (delta < 0.0f) limit = -decel_limit_per_s_ * dt;  // decelerating

    float next = last_cmd_throttle_;
    if (limit != 0.0) {
      if (delta > 0.0f) next = last_cmd_throttle_ + static_cast<float>(std::min<double>(delta,  limit));
      else              next = last_cmd_throttle_ + static_cast<float>(std::max<double>(delta,  limit));
    } else {
      next = desired;
    }

    next = std::clamp(next, -1.0f, 1.0f);
    last_cmd_throttle_ = next;
    return next;
  }

  // Timeout helpers
  bool isTimedOut() const
  {
    if (tfmini_timeout_ms_ <= 0) return false;            // disabled
    if (last_distance_ts_.nanoseconds() == 0) return true; // never received
    const int64_t dt_ns = (now() - last_distance_ts_).nanoseconds();
    return (dt_ns / 1000000) > tfmini_timeout_ms_;
  }

  bool isHoldingBrake() const
  {
    if (brake_hold_ms_ <= 0) return false;
    if (obstacle_blocked_)   return true;
    if (last_clear_ts_.nanoseconds() == 0) return false;
    const int64_t dt_ns = (now() - last_clear_ts_).nanoseconds();
    return (dt_ns / 1000000) < brake_hold_ms_;
  }

  // Publish helpers
  void publishThrottle(float thr_x)
  {
    px4_msgs::msg::RoverThrottleSetpoint t{};
    t.timestamp = now().nanoseconds();
    t.throttle_body_x = std::clamp(thr_x, -1.0f, 1.0f);
    t.throttle_body_y = 0.0f;
    thr_pub_->publish(t);
  }

  void publishSteering(float roll)
  {
    px4_msgs::msg::RoverSteeringSetpoint s{};
    s.timestamp = now().nanoseconds();
    s.normalized_steering_setpoint = std::clamp(roll, -1.0f, 1.0f);
    str_pub_->publish(s);
  }

  rclcpp::Time now() const { return get_clock()->now(); }

private:
  // Params/state
  bool   enabled_{true};
  double stop_threshold_m_{0.50};
  double clear_threshold_m_{0.70};
  double slow_zone_m_{0.90};
  double forward_min_scale_{0.35};
  double hard_stop_m_{0.35};
  int    control_hz_{50};
  int    tfmini_timeout_ms_{0};
  int    brake_hold_ms_{0};
  double accel_limit_per_s_{1.5};
  double decel_limit_per_s_{3.0};

  // Distance / RC cache
  double       last_distance_m_{-1.0};
  bool         very_close_{false};       // inside hard_stop_m_
  rclcpp::Time last_distance_ts_{};
  px4_msgs::msg::ManualControlSetpoint last_manual_{};
  rclcpp::Time last_manual_ts_{};

  // Obstacle state + transitions
  bool         obstacle_blocked_{false};
  rclcpp::Time last_block_ts_{};
  rclcpp::Time last_clear_ts_{};

  // Smoothing state
  float        last_cmd_throttle_{0.0f};
  rclcpp::Time last_cmd_stamp_{};

  // ROS I/O
  rclcpp::Publisher<px4_msgs::msg::RoverThrottleSetpoint>::SharedPtr thr_pub_;
  rclcpp::Publisher<px4_msgs::msg::RoverSteeringSetpoint>::SharedPtr str_pub_;
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_sub_;
  rclcpp::Subscription<px4_msgs::msg::DistanceSensor>::SharedPtr dist_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_;
};
