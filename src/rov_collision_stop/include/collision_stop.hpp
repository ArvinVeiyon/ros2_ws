#pragma once
/****************************************************************************
 * rov_collision_stop — Pre-brake in slow zone + hard stop very close
 *
 * INPUTS:
 *  - /fmu/out/manual_control_setpoint (px4_msgs/ManualControlSetpoint)
 *  - TFmini via /dev/ttyAMA2 @115200 (direct UART, no ROS hop)
 *
 * OUTPUTS:
 *  - /fmu/in/rover_throttle_setpoint  (px4_msgs/RoverThrottleSetpoint)
 *  - /fmu/in/rover_steering_setpoint  (px4_msgs/RoverSteeringSetpoint)
 *  - /rov/distance_m                  (std_msgs/Float32) debug distance (m)
 *
 * Behavior:
 *  - Reverse ALWAYS allowed (instant; optional zero-cross snap).
 *  - Forward passes unless blocked (< stop_threshold_m), with clear hysteresis.
 *  - PRE-BRAKE: on first entry into slow zone [stop .. slow_zone), send a small
 *    reverse pulse (once), then allow forward again.
 *  - HARD STOP: inside very_close band (< hard_stop_m), send a stronger reverse
 *    pulse, then hold 0 until clear.
 *
 * Minimal defaults (safe indoors):
 *  - stop_threshold_m=0.30, clear_threshold_m=0.45
 *  - slow_zone_m=0.80, hard_stop_m=0.20
 *  - pre_brake: level=0.10 (10%), ms=120, cooldown=400
 *  - hard stop pulse: level=0.22 (22%), ms=250
 *  - control_hz=100, slew OFF (crisp)
 ****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/rover_throttle_setpoint.hpp>
#include <px4_msgs/msg/rover_steering_setpoint.hpp>
#include <std_msgs/msg/float32.hpp>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstring>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

class CollisionStopNode : public rclcpp::Node
{
public:
  CollisionStopNode() : Node("rov_collision_stop")
  {
    // --- Parameters ---
    enabled_            = declare_parameter<bool>("enabled", true);

    stop_threshold_m_   = declare_parameter<double>("stop_threshold_m", 0.30);
    clear_threshold_m_  = declare_parameter<double>("clear_threshold_m", 0.45);

    // Slow zone entry point (> stop, < slow_zone)
    slow_zone_m_        = declare_parameter<double>("slow_zone_m", 0.80);

    // Very-close hard-stop band (< hard_stop)
    hard_stop_m_        = declare_parameter<double>("hard_stop_m", 0.20);

    control_hz_         = std::max<int>(10, declare_parameter<int>("control_hz", 100));
    tfmini_timeout_ms_  = std::max<int>(0, declare_parameter<int>("tfmini_timeout_ms", 0)); // 0=disabled

    // Slew (defaults OFF for crisp)
    accel_limit_per_s_  = std::max<double>(0.0, declare_parameter<double>("accel_limit_per_s", 0.0));
    decel_limit_per_s_  = std::max<double>(0.0, declare_parameter<double>("decel_limit_per_s", 0.0));
    reverse_bypass_slew_= declare_parameter<bool>("reverse_bypass_slew", true);
    zero_cross_snap_    = declare_parameter<bool>("zero_cross_snap", true);

    // Pre-brake in slow zone (single gentle pulse on entry)
    pre_brake_enable_      = declare_parameter<bool>("pre_brake_enable", true);
    pre_brake_level_       = std::clamp(declare_parameter<double>("pre_brake_level", 0.10), 0.0, 1.0);
    pre_brake_ms_          = std::max<int>(0, declare_parameter<int>("pre_brake_ms", 120));
    pre_brake_cooldown_ms_ = std::max<int>(0, declare_parameter<int>("pre_brake_cooldown_ms", 400));

    // Blocked/hard-stop reverse pulses
    brake_pulse_enable_      = declare_parameter<bool>("brake_pulse_enable", true);
    brake_pulse_level_       = std::clamp(declare_parameter<double>("brake_pulse_level", 0.22), 0.0, 1.0);
    brake_pulse_ms_          = std::max<int>(0, declare_parameter<int>("brake_pulse_ms", 250));
    brake_pulse_level_close_ = std::clamp(declare_parameter<double>("brake_pulse_level_close", 0.22), 0.0, 1.0);
    brake_pulse_ms_close_    = std::max<int>(0, declare_parameter<int>("brake_pulse_ms_close", 250));

    // Sanity relations
    if (clear_threshold_m_ <= stop_threshold_m_) {
      clear_threshold_m_ = stop_threshold_m_ + 0.10;
      RCLCPP_WARN(get_logger(), "Adjusted clear_threshold_m to %.2f (> stop %.2f).",
                  clear_threshold_m_, stop_threshold_m_);
    }
    if (slow_zone_m_ < clear_threshold_m_) slow_zone_m_ = clear_threshold_m_;
    if (hard_stop_m_ > stop_threshold_m_)  hard_stop_m_ = stop_threshold_m_;

    auto be = rclcpp::QoS(10).best_effort();

    // --- Publishers ---
    thr_pub_ = create_publisher<px4_msgs::msg::RoverThrottleSetpoint>("/fmu/in/rover_throttle_setpoint", be);
    str_pub_ = create_publisher<px4_msgs::msg::RoverSteeringSetpoint>("/fmu/in/rover_steering_setpoint", be);
    dist_debug_pub_ = create_publisher<std_msgs::msg::Float32>("/rov/distance_m", be);

    // --- Subscriber (manual input) ---
    manual_sub_ = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
      "/fmu/out/manual_control_setpoint", be,
      std::bind(&CollisionStopNode::onManual, this, std::placeholders::_1));

    // --- Control loop ---
    ctrl_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / control_hz_),
      std::bind(&CollisionStopNode::controlTick, this));

    // --- Param updates ---
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& ps){
        for (const auto &p : ps) {
          const auto &n = p.get_name();
          if      (n == "enabled")            enabled_ = p.as_bool();
          else if (n == "stop_threshold_m")   stop_threshold_m_ = p.as_double();
          else if (n == "clear_threshold_m")  clear_threshold_m_ = p.as_double();
          else if (n == "slow_zone_m")        slow_zone_m_ = p.as_double();
          else if (n == "hard_stop_m")        hard_stop_m_ = p.as_double();
          else if (n == "control_hz")         control_hz_ = std::max<int>(10, p.as_int());
          else if (n == "tfmini_timeout_ms")  tfmini_timeout_ms_ = std::max<int>(0, p.as_int());
          else if (n == "accel_limit_per_s")  accel_limit_per_s_ = std::max<double>(0.0, p.as_double());
          else if (n == "decel_limit_per_s")  decel_limit_per_s_ = std::max<double>(0.0, p.as_double());
          else if (n == "reverse_bypass_slew") reverse_bypass_slew_ = p.as_bool();
          else if (n == "zero_cross_snap")     zero_cross_snap_     = p.as_bool();

          else if (n == "pre_brake_enable")        pre_brake_enable_ = p.as_bool();
          else if (n == "pre_brake_level")         pre_brake_level_  = std::clamp(p.as_double(), 0.0, 1.0);
          else if (n == "pre_brake_ms")            pre_brake_ms_     = std::max<int>(0, p.as_int());
          else if (n == "pre_brake_cooldown_ms")   pre_brake_cooldown_ms_ = std::max<int>(0, p.as_int());

          else if (n == "brake_pulse_enable")      brake_pulse_enable_ = p.as_bool();
          else if (n == "brake_pulse_level")       brake_pulse_level_  = std::clamp(p.as_double(), 0.0, 1.0);
          else if (n == "brake_pulse_ms")          brake_pulse_ms_     = std::max<int>(0, p.as_int());
          else if (n == "brake_pulse_level_close") brake_pulse_level_close_ = std::clamp(p.as_double(), 0.0, 1.0);
          else if (n == "brake_pulse_ms_close")    brake_pulse_ms_close_    = std::max<int>(0, p.as_int());
        }
        if (clear_threshold_m_ <= stop_threshold_m_)  clear_threshold_m_ = stop_threshold_m_ + 0.10;
        if (slow_zone_m_ < clear_threshold_m_)        slow_zone_m_ = clear_threshold_m_;
        if (hard_stop_m_ > stop_threshold_m_)         hard_stop_m_ = stop_threshold_m_;

        ctrl_timer_->cancel();
        ctrl_timer_ = create_wall_timer(
          std::chrono::milliseconds(1000 / control_hz_),
          std::bind(&CollisionStopNode::controlTick, this));

        rcl_interfaces::msg::SetParametersResult r; r.successful = true; return r;
      });

    openTfmini("/dev/ttyAMA2", B115200);

    RCLCPP_INFO(get_logger(),
      "started (stop=%.2fm clear=%.2fm slow=%.2fm hard=%.2fm, pre_brake=%s L=%.2f %dms cd=%dms, "
      "block_pulse=%s L=%.2f %dms close L=%.2f %dms, ctrl=%dHz)",
      stop_threshold_m_, clear_threshold_m_, slow_zone_m_, hard_stop_m_,
      pre_brake_enable_ ? "on":"off", pre_brake_level_, pre_brake_ms_, pre_brake_cooldown_ms_,
      brake_pulse_enable_ ? "on":"off", brake_pulse_level_, brake_pulse_ms_,
      brake_pulse_level_close_, brake_pulse_ms_close_, control_hz_);
  }

  ~CollisionStopNode() override { if (tf_fd_ >= 0) ::close(tf_fd_); }

private:
  // --- UART helpers ---
  void openTfmini(const char* dev, speed_t baud)
  {
    tf_fd_ = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (tf_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "TFmini: open %s failed", dev);
      tf_fd_ok_ = false;
      return;
    }
    struct termios tty{};
    if (tcgetattr(tf_fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "TFmini: tcgetattr failed");
      ::close(tf_fd_);
      tf_fd_ = -1;
      tf_fd_ok_ = false;
      return;
    }
    cfmakeraw(&tty);
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    if (tcsetattr(tf_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "TFmini: tcsetattr failed");
      ::close(tf_fd_);
      tf_fd_ = -1;
      tf_fd_ok_ = false;
      return;
    }
    tf_fd_ok_ = true;
    RCLCPP_INFO(get_logger(), "TFmini: %s @115200 opened", dev);
  }

  void readTfmini()
  {
    if (!tf_fd_ok_) return;

    int avail = 0;
    if (ioctl(tf_fd_, FIONREAD, &avail) != 0) {
      avail = 0;
    }
    if (avail <= 0) {
      return;
    }
    if (avail > 128) {
      avail = 128;
    }

    uint8_t buf[128];
    int n = ::read(tf_fd_, buf, avail);
    if (n <= 0) return;

    for (int i = 0; i < n; ++i) {
      // Shift buffer (size FRAME_LEN=9), append new byte
      std::memmove(frame_buf_, frame_buf_ + 1, FRAME_LEN - 1);
      frame_buf_[FRAME_LEN - 1] = buf[i];

      // Look for TFmini frame header 0x59 0x59
      if (frame_buf_[0] == 0x59 && frame_buf_[1] == 0x59) {
        uint16_t sum = 0;
        for (int k = 0; k < 8; ++k) {
          sum += frame_buf_[k];
        }
        if ((sum & 0xFF) != frame_buf_[8]) {
          continue;
        }

        uint16_t dist_cm = static_cast<uint16_t>(frame_buf_[2]) |
                           (static_cast<uint16_t>(frame_buf_[3]) << 8);
        const double dist_m = static_cast<double>(dist_cm) / 100.0;

        const rclcpp::Time tnow = now();
        last_distance_m_  = dist_m;
        last_distance_ts_ = tnow;

        // Update zones with hysteresis & rising edges
        const bool was_blocked   = obstacle_blocked_;
        const bool was_slow      = in_slow_zone_;
        const bool was_veryclose = very_close_;

        // Blocked if < stop; clear if >= clear
        if (last_distance_m_ > 0.02 && last_distance_m_ < stop_threshold_m_) {
          obstacle_blocked_ = true;
        } else if (last_distance_m_ >= clear_threshold_m_) {
          obstacle_blocked_ = false;
        }
        if (obstacle_blocked_ && !was_blocked) {
          last_block_ts_ = tnow;
        }
        if (!obstacle_blocked_ && was_blocked) {
          last_clear_ts_ = tnow;
        }

        // Slow zone (between stop and slow)
        in_slow_zone_ = (last_distance_m_ >= stop_threshold_m_ && last_distance_m_ < slow_zone_m_);
        if (in_slow_zone_ && !was_slow) {
          last_slow_entry_ts_ = tnow;
          slow_pulsed_ = false;
        }
        if (!in_slow_zone_ && was_slow) {
          last_slow_exit_ts_  = tnow;
          slow_pulsed_ = false;
        }

        // Very close
        very_close_ = (last_distance_m_ > 0.02 && last_distance_m_ < hard_stop_m_);
        if (very_close_ && !was_veryclose) {
          last_close_ts_ = tnow;
        }

        // Debug
        if (dist_debug_pub_) {
          std_msgs::msg::Float32 dbg;
          dbg.data = static_cast<float>(last_distance_m_);
          dist_debug_pub_->publish(dbg);
        }
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 500,
          "TFmini: %.2fm | slow=%d blocked=%d vclose=%d",
          last_distance_m_, in_slow_zone_, obstacle_blocked_, very_close_);
      }
    }
  }

  // --- Callbacks ---
  void onManual(const px4_msgs::msg::ManualControlSetpoint::SharedPtr m)
  {
    last_manual_ = *m;
    last_manual_ts_ = now();
    if (!enabled_) return;

    publishSteering(m->roll);

    float desired = gateThrottle(m->throttle);  // reverse allowed; forward gated + pulses
    desired = applySlew(desired);               // optional smoothing
    publishThrottle(desired);
  }

  void controlTick()
  {
    if (!enabled_) return;

    readTfmini();

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

  // --- Gating / pulses ---
  float gateThrottle(float rc_throttle)
  {
    float thr = std::clamp(rc_throttle, -1.0f, 1.0f);

    // Reverse always allowed
    if (thr < 0.0f) return thr;

    // Optional sensor timeout cut
    if (tfmini_timeout_ms_ > 0 && isTimedOut()) return 0.0f;

    const rclcpp::Time tnow = now();

    // HARD STOP zone first: stronger pulse while inside close band
    if (very_close_) {
      if (brake_pulse_enable_ && last_close_ts_.nanoseconds() > 0) {
        const int64_t ms = (tnow - last_close_ts_).nanoseconds() / 1000000;
        if (ms >= 0 && ms < brake_pulse_ms_close_) {
          return -static_cast<float>(brake_pulse_level_close_);
        }
      }
      return 0.0f;  // after pulse, hold zero
    }

    // BLOCKED zone: normal brake pulse on entry
    if (obstacle_blocked_) {
      if (brake_pulse_enable_ && last_block_ts_.nanoseconds() > 0) {
        const int64_t ms = (tnow - last_block_ts_).nanoseconds() / 1000000;
        if (ms >= 0 && ms < brake_pulse_ms_) {
          return -static_cast<float>(brake_pulse_level_);
        }
      }
      return 0.0f;
    }

    // PRE-BRAKE in SLOW zone: a single gentle reverse pulse on entry, then allow forward
    if (pre_brake_enable_ && in_slow_zone_ && last_slow_entry_ts_.nanoseconds() > 0) {
      const int64_t ms = (tnow - last_slow_entry_ts_).nanoseconds() / 1000000;
      // Only once per entry; ignore if still in cooldown after a recent exit/entry bounce
      const bool in_cooldown = (last_slow_exit_ts_.nanoseconds() > 0) &&
                               ((tnow - last_slow_exit_ts_).nanoseconds() / 1000000 < pre_brake_cooldown_ms_);
      if (!slow_pulsed_ && !in_cooldown && ms >= 0 && ms < pre_brake_ms_) {
        slow_pulsed_ = true; // latch so we only pulse once per entry
        return -static_cast<float>(pre_brake_level_);
      }
    }

    // CLEAR region → pass-through forward
    return thr;
  }

  // Slew (reverse bypass + zero-cross snap)
  float applySlew(float desired)
  {
    const rclcpp::Time tnow = now();
    const double dt = (last_cmd_stamp_.nanoseconds() > 0)
                        ? (tnow - last_cmd_stamp_).nanoseconds() / 1e9
                        : 0.0;
    last_cmd_stamp_ = tnow;

    // Reverse immediate
    if (desired < 0.0f && reverse_bypass_slew_) {
      if (zero_cross_snap_ && last_cmd_throttle_ > 0.0f) {
        last_cmd_throttle_ = 0.0f;
      }
      last_cmd_throttle_ = std::clamp(desired, -1.0f, 1.0f);
      return last_cmd_throttle_;
    }

    if (dt <= 0.0 || (accel_limit_per_s_ <= 0.0 && decel_limit_per_s_ <= 0.0)) {
      last_cmd_throttle_ = std::clamp(desired, -1.0f, 1.0f);
      return last_cmd_throttle_;
    }

    const float delta = desired - last_cmd_throttle_;
    if (delta == 0.0f) return last_cmd_throttle_;
    double limit = (delta > 0.0f) ? (accel_limit_per_s_ * dt) : (-decel_limit_per_s_ * dt);
    float next = last_cmd_throttle_ + (delta > 0.0f
                    ? static_cast<float>(std::min<double>(delta,  limit))
                    : static_cast<float>(std::max<double>(delta,  limit)));
    last_cmd_throttle_ = std::clamp(next, -1.0f, 1.0f);
    return last_cmd_throttle_;
  }

  bool isTimedOut() const
  {
    if (tfmini_timeout_ms_ <= 0) return false;
    if (last_distance_ts_.nanoseconds() == 0) return true;
    const int64_t dt_ns = (now() - last_distance_ts_).nanoseconds();
    return (dt_ns / 1000000) > tfmini_timeout_ms_;
  }

  // Publish
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
  // Params
  bool   enabled_{true};
  double stop_threshold_m_{0.30};
  double clear_threshold_m_{0.45};
  double slow_zone_m_{0.80};
  double hard_stop_m_{0.20};
  int    control_hz_{100};
  int    tfmini_timeout_ms_{0};

  double accel_limit_per_s_{0.0};
  double decel_limit_per_s_{0.0};
  bool   reverse_bypass_slew_{true};
  bool   zero_cross_snap_{true};

  bool   pre_brake_enable_{true};
  double pre_brake_level_{0.10};
  int    pre_brake_ms_{120};
  int    pre_brake_cooldown_ms_{400};

  bool   brake_pulse_enable_{true};
  double brake_pulse_level_{0.22};
  int    brake_pulse_ms_{250};
  double brake_pulse_level_close_{0.22};
  int    brake_pulse_ms_close_{250};

  // State
  double       last_distance_m_{-1.0};
  rclcpp::Time last_distance_ts_{};
  bool         obstacle_blocked_{false};
  bool         in_slow_zone_{false};
  bool         very_close_{false};

  rclcpp::Time last_block_ts_{};
  rclcpp::Time last_clear_ts_{};
  rclcpp::Time last_slow_entry_ts_{};
  rclcpp::Time last_slow_exit_ts_{};
  bool         slow_pulsed_{false};
  rclcpp::Time last_close_ts_{};

  px4_msgs::msg::ManualControlSetpoint last_manual_{};
  rclcpp::Time last_manual_ts_{};

  float        last_cmd_throttle_{0.0f};
  rclcpp::Time last_cmd_stamp_{};

  // ROS I/O
  rclcpp::Publisher<px4_msgs::msg::RoverThrottleSetpoint>::SharedPtr thr_pub_;
  rclcpp::Publisher<px4_msgs::msg::RoverSteeringSetpoint>::SharedPtr str_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_debug_pub_;
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr manual_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_;

  // UART
  int  tf_fd_{-1};
  bool tf_fd_ok_{false};
  static constexpr int FRAME_LEN = 9;
  uint8_t frame_buf_[FRAME_LEN]{};
};
