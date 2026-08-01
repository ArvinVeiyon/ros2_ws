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
// (a test script, Nav2, a joystick). It REMOVES forward motion and, when
// blocked, also CAPS yaw; reverse always passes through so the vehicle can
// back off.
//
// Distances below are CLEARANCE AT THE FRONT BUMPER, not raw /scan range.
// /scan originates at camera_link, which sits kFrontOverhang behind the front
// plate tip, so the raw range is converted before any comparison. Getting this
// wrong is what put the rover into a wall on 2026-07-28: 0.60 m of raw range
// was only 0.255 m of actual bumper clearance.
static constexpr double kStopDistance = 0.35;    // [m] block forward if bumper closer than this
static constexpr double kClearDistance = 0.50;   // [m] release only once farther than this (hysteresis)
static constexpr double kSectorHalfAngle = 0.35; // [rad] +/- forward sector (~20 deg), outer bound only
// Half the width of the corridor the body sweeps. The top plate is 0.450 m wide
// (the wheels sit INBOARD of it, so the 0.31 m track is the wrong number), giving
// a half-width of 0.225 m; 0.275 adds ~5 cm of margin per side for heading error.
// This, not the sector angle, is what decides whether an obstacle is in the path.
// CORRECTED 2026-08-01: was 0.25, derived from a plate width of 0.405 m that
// docs/rover_autonav_requirements.md:75 supersedes ("07-21 said 0.405 - wrong").
// 0.25 still covered the body, so nothing in the path was missed, but it left
// 2.5 cm of heading-error margin per side rather than the 5 cm claimed here.
static constexpr double kCorridorHalfWidth = 0.275;  // [m]
static constexpr double kScanTimeout = 0.5;      // [s] /scan older than this -> perception stale
// MEASURED 2026-07-28, rover parked square against a flat wall with zero gap:
// the forward-sector min range read 0.337 m over 178 consecutive scans with no
// spread (min == max == 0.337). That is the scan origin -> bumper distance.
// Agrees with the 0.345 m base_link -> front-plate-tip figure in
// docs/rover_autonav_requirements.md to within 8 mm. Re-measure the same way if
// the camera is ever remounted — this constant is what makes the distances below
// mean bumper clearance rather than lens clearance.
static constexpr double kFrontOverhang = 0.337;  // [m] scan origin -> front bumper
// Vehicle footprint in base_link, used to reject the rover's own bodywork from a
// height-aware scan. The top plate is 0.730 x 0.450 m with the rotation centre
// 0.345 m back from the tip => x in [-0.385, +0.345], |y| <= 0.225.
// Full derivation and the consumer list: docs/rover_geometry.md.
static constexpr double kFootprintFront = 0.345;      // [m] base_link -> front plate tip
static constexpr double kFootprintHalfWidth = 0.225;  // [m] half of the 0.450 m plate
// Grown slightly before the reject test: the plate's own edges lie EXACTLY on the
// footprint boundary, so a bare strict inequality lets the edge itself leak
// through as an obstacle (observed 2026-08-01 at bearing 35 deg / range 0.392 m,
// which is precisely where the side edge sits). This also absorbs static-TF and
// measurement error, which is ~1 cm on front_overhang. Cost: a real obstacle
// within 20 mm of the bodywork is ignored -- acceptable, since the reflex already
// blocks at 350 mm of bumper clearance and could never legitimately be there.
static constexpr double kFootprintMargin = 0.02;      // [m]
// While blocked, yaw is capped rather than freed: enough authority to turn away,
// not enough to lunge. An asymmetric skid-steer spin TRANSLATES, which is how an
// ungated yaw leg reached the wall despite the forward brake working correctly.
static constexpr double kBlockedYawRate = 0.3;   // [rad/s] max |yaw| while blocked

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
    _corridor_half_width = node.declare_parameter<double>("collision.corridor_half_width", kCorridorHalfWidth);
    _scan_timeout = node.declare_parameter<double>("collision.scan_timeout", kScanTimeout);
    _front_overhang = node.declare_parameter<double>("collision.front_overhang", kFrontOverhang);
    _footprint_front = node.declare_parameter<double>("collision.footprint_front", kFootprintFront);
    _footprint_half_width = node.declare_parameter<double>("collision.footprint_half_width", kFootprintHalfWidth);
    _footprint_margin = node.declare_parameter<double>("collision.footprint_margin", kFootprintMargin);
    _blocked_yaw_rate = node.declare_parameter<double>("collision.blocked_yaw_rate", kBlockedYawRate);
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
        "AutoNav collision-stop %s: bumper stop<%.2fm clear>%.2fm (overhang %.3fm => forward "
        "%.2fm/%.2fm) corridor=+/-%.2fm sector<=+/-%.0fdeg scan_timeout=%.2fs require_scan=%s "
        "footprint x<%.3f&|y|<%.3f (+%.3f margin) rejected blocked_yaw<=%.2frad/s",
        _collision_enabled ? "ON" : "OFF", _stop_distance, _clear_distance, _front_overhang,
        _stop_distance + _front_overhang, _clear_distance + _front_overhang,
        _corridor_half_width, _sector_half * 180.0 / M_PI, _scan_timeout,
        _require_scan ? "yes" : "no", _footprint_front, _footprint_half_width, _footprint_margin,
        _blocked_yaw_rate);

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

    // Reflex collision-stop. Evaluated every tick (not only when driving forward) so
    // the hysteresis state tracks reality continuously and yaw can be capped too.
    // Reverse always passes through so the vehicle can back off.
    if (_collision_enabled && forwardBlocked()) {
      if (speed > 0.f) {
        RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
            "collision-stop: forward blocked (bumper=%.2fm raw=%.2fm)",
            frontClearance(), _front_min_range);
        speed = 0.f;
      }
      // Cap, don't cancel: a skid-steer must still be able to rotate away, but an
      // asymmetric spin translates, so unlimited yaw next to an obstacle is a lunge.
      const float cap = static_cast<float>(_blocked_yaw_rate);
      if (std::abs(yaw_rate) > cap) {
        RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
            "collision-stop: yaw capped %.2f -> %.2f rad/s (bumper=%.2fm)",
            yaw_rate, std::copysign(cap, yaw_rate), frontClearance());
        yaw_rate = std::copysign(cap, yaw_rate);
      }
    }

    _rover_setpoint->update(speed, yaw_rate);
  }

 private:
  // Nearest obstacle inside the CORRIDOR THE BODY WILL SWEEP.
  //
  // This used to test an angular sector, which is the wrong shape. The rover is
  // a fixed 0.450 m wide, but a cone narrows as it approaches the sensor, so an
  // obstacle could sit inside the body width and still fall outside the cone.
  // That happens whenever the forward distance is less than
  //     half_width / tan(sector_half) = 0.225 / tan(20 deg) = 0.618 m
  // i.e. a box just in front of a front wheel was invisible to the brake while
  // being exactly the thing the brake exists to stop for.
  //
  // So each ray is resolved into forward (x) and lateral (y) components and kept
  // only if it lies within the corridor. Clearance is then measured along x --
  // the direction of travel -- not along the slant range r. For an off-centre
  // obstacle r > x, so the old test also OVER-reported clearance. Using x is
  // both correct and conservative, and it leaves the measured front_overhang
  // calibration valid, since that was taken square-on to a wall where x == r.
  void onScan(const sensor_msgs::msg::LaserScan& scan)
  {
    float min_x = std::numeric_limits<float>::infinity();
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float ang = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
      if (ang < -_sector_half || ang > _sector_half) {
        continue;  // outer bound only, cheap reject
      }
      const float r = scan.ranges[i];
      if (!std::isfinite(r) || r <= 0.f || r < scan.range_min || r > scan.range_max) {
        continue;  // 0 / inf / nan / out-of-spec are not valid obstacles
      }
      const float x = r * std::cos(ang);   // forward, along travel
      const float y = r * std::sin(ang);   // lateral
      if (x <= 0.f || std::fabs(y) > _corridor_half_width) {
        continue;  // beside the rover, not in its path
      }
      // Returns from INSIDE the vehicle footprint are the vehicle. The depth
      // camera sees a 45 mm sliver of the rover's own top plate at 0.300-0.345 m
      // (docs/rover_geometry.md S4); nothing external can be inside the bumper.
      //
      // This is a POLYGON test, not a radial one, and both terms matter. The
      // corridor is +/-0.275 m but the body is only +/-0.225 m half-width, so a
      // return at x=0.32, y=0.26 is BESIDE the rover -- a real obstacle -- and a
      // bare `x < front` cut would silently discard it.
      //
      // Doing it here rather than with a blanket scan range_min is what lets
      // range_min drop to the sensor floor: a constant 0.40 m cut also erased
      // genuine obstacles between the 0.337 m bumper and 0.40 m, and a dropped
      // ray is indistinguishable from empty space, so the reflex read those as
      // INFINITE clearance. That was a ~6 cm fail-open strip at the bumper.
      if (x < _footprint_front + _footprint_margin &&
          std::fabs(y) < _footprint_half_width + _footprint_margin) {
        continue;  // the rover's own bodywork
      }
      min_x = std::min(min_x, x);
    }
    _front_min_range = min_x;  // inf => nothing in the corridor
    _last_scan_time = _node.get_clock()->now();
  }

  // Clearance at the front BUMPER, which is what the thresholds mean. Infinity
  // (nothing seen in the sector) stays infinity.
  float frontClearance() const
  {
    return _front_min_range - static_cast<float>(_front_overhang);
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
    const float clearance = frontClearance();
    if (clearance < _stop_distance) {
      _blocked = true;
    } else if (clearance > _clear_distance) {
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
    const bool would_block = scan_fresh ? (frontClearance() < _stop_distance)
                                        : _require_scan;
    if (!_diag_inited || would_block != _diag_last) {
      RCLCPP_INFO(_node.get_logger(),
          "collision-diag: %s  (scan_fresh=%s bumper=%.2fm raw=%.2fm)",
          would_block ? "BLOCK forward" : "clear",
          scan_fresh ? "yes" : "no", frontClearance(), _front_min_range);
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
  double _corridor_half_width{kCorridorHalfWidth};
  double _footprint_front{kFootprintFront};
  double _footprint_half_width{kFootprintHalfWidth};
  double _footprint_margin{kFootprintMargin};
  double _scan_timeout{kScanTimeout};
  double _front_overhang{kFrontOverhang};
  double _blocked_yaw_rate{kBlockedYawRate};

  // collision-stop state
  float _front_min_range{std::numeric_limits<float>::infinity()};
  rclcpp::Time _last_scan_time{0, 0, RCL_ROS_TIME};
  bool _blocked{false};

  // passive on-stands diagnostic
  rclcpp::TimerBase::SharedPtr _diag_timer;
  bool _diag_last{false};
  bool _diag_inited{false};
};
