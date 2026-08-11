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
// PERCEPTION-HEALTH GATE. Fraction of the WHOLE scan that must carry a valid
// range before the clearance test is allowed to run at all.
//
// This exists because of the 2026-08-10 00:30 wall contact. A fresh scan that
// returns ZERO valid rays in the corridor yields min_x = inf => clearance = inf
// => "farther than clear_distance" => UNBLOCKED. The rover ratcheted into a wall
// through alternating glimpses, each blocking correctly and each released by the
// blind frame that followed:
//     BLOCK 0.35 -> clear inf -> BLOCK 0.11 -> clear 0.38 -> BLOCK 0.26 -> clear inf -> BLOCK 0.08
// scan_fresh was "yes" the whole way. The stale-scan fail-safe below covers a
// scan that stops ARRIVING; it did not cover one that arrives EMPTY. "Cannot
// see" was being read as "nothing there" — the same fail-open class as the
// dropped-ray bug already fixed at the footprint, missed here.
//
// It cannot be gated on "0 rays in the corridor": a genuinely empty room gives
// that too. The discriminator is validity across the FULL scan, which does not
// depend on where the rover is pointed. MEASURED 2026-08-10 on this camera:
// healthy = 560/640 = 87.5% (corridor 222/276), stable to <0.5% run to run;
// blind ~= 0%. 0.35 sits ~2.5x below healthy and far above blind.
//
// FAILS SAFE BY CONSTRUCTION: anything that destroys returns — a starved camera,
// a surface with no IR return, an obstacle inside the 0.308 m depth minimum —
// drives the fraction DOWN and therefore blocks. The cost is that a genuinely
// open space larger than range_max also reads as low-validity and blocks; that
// is the correct direction to be wrong, and indoors it does not arise.
static constexpr double kMinValidFraction = 0.35;  // [0-1] of the whole scan
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
// Grown before the reject test, for two reasons.
//
// 1. The plate's own edges lie EXACTLY on the footprint boundary, so a bare strict
//    inequality lets the edge itself leak through as an obstacle (observed at
//    bearing 35 deg / range 0.392 m, precisely where the side edge sits). It also
//    absorbs static-TF and measurement error, ~1 cm on front_overhang.
//
// 2. MIXED PIXELS AT THE PLATE EDGE. The depth camera interpolates across the
//    discontinuity between the deck edge and the distant floor, producing points
//    that belong to NEITHER surface. Confirmed visually 2026-08-01: flush with the
//    plate edge, nothing physically there. They are NOT removable by filtering:
//      - the SDK's DispOutliersFilter and FalsePositiveFilter are NOT SUPPORTED by
//        this device; TemporalFilter and SpatialAdvancedFilter were measured and
//        changed nothing (38.6 +/-7.3 baseline vs 41.0 +/-14.3 and 32.1 +/-9.8);
//      - a temporal-persistence test does not discriminate either, because these
//        beams are present in EVERY frame and merely jitter in range.
//    They are, however, BOUNDED. Measured extent past the 0.345 m edge:
//        x 0.35-0.40  71 pts   |   x 0.40-0.50  ZERO   |   x 0.50+ unrelated
//    so 0.06 clears the skirt with a 10 cm clean gap beyond it.
//
// COST of 0.06: blind from 0.345 to 0.405 m, i.e. the first 60 mm past the bumper.
// Operationally irrelevant for this reflex, which blocks at 0.35 m of bumper
// clearance (x = 0.687) and so has already stopped long before anything reaches
// 0.405 m. It matters only if the vehicle is placed inside its own stop distance.
static constexpr double kFootprintMargin = 0.06;      // [m]
// While blocked, yaw is capped rather than freed: enough authority to turn away,
// not enough to lunge. An asymmetric skid-steer spin TRANSLATES, which is how an
// ungated yaw leg reached the wall despite the forward brake working correctly.
static constexpr double kBlockedYawRate = 0.3;   // [rad/s] max |yaw| while blocked

class AutoNavMode : public px4_ros2::ModeBase {
 public:
  // activateEvenWhileDisarmed: without it px4_ros2 sets is_active only when
  // (nav_state matches AND armed) -- see ModeBase::vehicleStatusUpdated -- so a
  // DISARMED mode switch moves nav_state to 23 while updateSetpoint() is never
  // called at all. The mode looks selected and is not running. That cost a test
  // run on 2026-08-11: the acting path could not be observed disarmed, because
  // there was no acting path executing to observe.
  //
  // With it, the full control loop (including the collision reflex) runs while
  // disarmed, so the brake can be validated against a REAL setpoint with the
  // wheels physically unable to turn. Disarmed setpoints reach no actuator.
  explicit AutoNavMode(rclcpp::Node& node)
      : ModeBase(node, px4_ros2::ModeBase::Settings{kModeName}
                           .activateEvenWhileDisarmed(true)),
        _node(node)
  {
    _rover_setpoint = std::make_shared<px4_ros2::RoverSpeedRateSetpointType>(*this);

    // --- reflex collision-stop parameters ---
    _collision_enabled = node.declare_parameter<bool>("collision.enabled", true);
    _stop_distance = node.declare_parameter<double>("collision.stop_distance", kStopDistance);
    _clear_distance = node.declare_parameter<double>("collision.clear_distance", kClearDistance);
    _sector_half = node.declare_parameter<double>("collision.sector_half_angle", kSectorHalfAngle);
    _corridor_half_width = node.declare_parameter<double>("collision.corridor_half_width", kCorridorHalfWidth);
    _scan_timeout = node.declare_parameter<double>("collision.scan_timeout", kScanTimeout);
    _min_valid_fraction = node.declare_parameter<double>("collision.min_valid_fraction", kMinValidFraction);
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

    // Which scan feeds the brake. Default stays /scan (depthimage_to_laserscan, a
    // single horizontal row) so behaviour is unchanged unless someone opts in.
    //
    // /scan_3d is the height-aware alternative from rover-scan-3d.service. It is
    // faster (29.2 Hz / 99 ms worst gap vs 23.5 / 233) and sees the class of
    // obstacle a horizontal row cannot -- table tops, low boxes, overhangs.
    // Measured 2026-08-08, rover parked ~3 cm off a rack: the two agree to 6 mm
    // (/scan 0.028 m bumper clearance, /scan_3d 0.034 m), so /scan_3d is not
    // worse on a plain obstacle dead ahead -- but note it reads marginally LESS
    // conservative, and that test cannot show the height-awareness that is the
    // whole point. ⛔ Do NOT flip this until /scan_3d is shown to catch a low or
    // overhanging object that /scan misses; see docs/rover_autonav_collision_stop.md.
    //
    // ⚠️ /scan_3d contains the rover's own top plate BY DESIGN. The footprint
    // rejection in onScan() is what makes that safe -- do not bypass it.
    _scan_topic = node.declare_parameter<std::string>("collision.scan_topic", "/scan");

    // Sensor data QoS (best-effort) matches typical LaserScan publishers.
    _scan_sub = node.create_subscription<sensor_msgs::msg::LaserScan>(
        _scan_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::UniquePtr msg) { onScan(*msg); });

    RCLCPP_INFO(_node.get_logger(),
        "AutoNav collision-stop %s on %s: bumper stop<%.2fm clear>%.2fm (overhang %.3fm => forward "
        "%.2fm/%.2fm) corridor=+/-%.2fm sector<=+/-%.0fdeg scan_timeout=%.2fs require_scan=%s "
        "min_valid=%.0f%% footprint x<%.3f&|y|<%.3f (+%.3f margin) rejected blocked_yaw<=%.2frad/s",
        _collision_enabled ? "ON" : "OFF", _scan_topic.c_str(),
        _stop_distance, _clear_distance, _front_overhang,
        _stop_distance + _front_overhang, _clear_distance + _front_overhang,
        _corridor_half_width, _sector_half * 180.0 / M_PI, _scan_timeout,
        _require_scan ? "yes" : "no", _min_valid_fraction * 100.0,
        _footprint_front, _footprint_half_width, _footprint_margin,
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
    size_t valid_total = 0;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float ang = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
      const float r = scan.ranges[i];
      const bool valid = std::isfinite(r) && r > 0.f && r >= scan.range_min && r <= scan.range_max;
      // Health is counted over the WHOLE scan, BEFORE the sector reject, because
      // it must answer "can the camera see?" independently of where the rover is
      // pointed. Counting only the corridor would conflate an empty room with a
      // blind one — the exact confusion that put the rover into a wall.
      if (valid) {
        ++valid_total;
      }
      if (ang < -_sector_half || ang > _sector_half) {
        continue;  // outer bound only, cheap reject
      }
      if (!valid) {
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
    _front_min_range = min_x;  // inf => nothing in the corridor OR nothing visible; see the gate
    _scan_valid_fraction = scan.ranges.empty()
        ? 0.f
        : static_cast<float>(valid_total) / static_cast<float>(scan.ranges.size());
    _last_scan_time = _node.get_clock()->now();
  }

  // Can the camera see at all? A scan that ARRIVES is not a scan that SEES.
  bool perceptionHealthy() const
  {
    return _scan_valid_fraction >= static_cast<float>(_min_valid_fraction);
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
    if (!perceptionHealthy()) {
      // Fresh but BLIND. Treated exactly like stale, and deliberately placed
      // BEFORE the clearance test so a blind frame can never reach the release
      // branch below and clear a block earned by the last frame that could see.
      RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 1000,
          "collision-stop: perception BLIND (valid %.0f%% < %.0f%%) — forward blocked",
          _scan_valid_fraction * 100.f, _min_valid_fraction * 100.0);
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

  enum class DiagState { Clear, Block, BlockStale, ClearStale, BlockBlind, ClearBlind };

  static const char* diagLabel(DiagState s)
  {
    switch (s) {
      case DiagState::Block:      return "BLOCK forward";
      case DiagState::BlockStale: return "BLOCK forward (stale)";
      case DiagState::ClearStale: return "clear (stale, permitted)";
      case DiagState::BlockBlind: return "BLOCK forward (BLIND)";
      case DiagState::ClearBlind: return "clear (blind, permitted)";
      case DiagState::Clear:      break;
    }
    return "clear";
  }

  // Passive, stateless view of the block decision for on-stands validation.
  // Uses the raw stop_distance (no hysteresis) so the flip point is a clean
  // ~stop_distance in both directions, easy to read while waving a wall.
  void diagTick()
  {
    const bool scan_fresh = _last_scan_time.nanoseconds() > 0 &&
        (_node.get_clock()->now() - _last_scan_time).seconds() < _scan_timeout;
    // Three states, not two: a BLIND report must be distinguishable from a
    // clear one in the log, otherwise the ratchet that caused the 08-10 contact
    // is invisible during on-stands validation — it printed "clear" both times.
    DiagState state;
    if (!scan_fresh) {
      state = _require_scan ? DiagState::BlockStale : DiagState::ClearStale;
    } else if (!perceptionHealthy()) {
      state = _require_scan ? DiagState::BlockBlind : DiagState::ClearBlind;
    } else {
      state = (frontClearance() < _stop_distance) ? DiagState::Block : DiagState::Clear;
    }
    if (!_diag_inited || state != _diag_last) {
      RCLCPP_INFO(_node.get_logger(),
          "collision-diag: %s  (scan_fresh=%s valid=%.0f%% bumper=%.2fm raw=%.2fm)",
          diagLabel(state), scan_fresh ? "yes" : "no", _scan_valid_fraction * 100.f,
          frontClearance(), _front_min_range);
      _diag_last = state;
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
  std::string _scan_topic{"/scan"};
  double _stop_distance{kStopDistance};
  double _clear_distance{kClearDistance};
  double _sector_half{kSectorHalfAngle};
  double _corridor_half_width{kCorridorHalfWidth};
  double _footprint_front{kFootprintFront};
  double _footprint_half_width{kFootprintHalfWidth};
  double _footprint_margin{kFootprintMargin};
  double _scan_timeout{kScanTimeout};
  double _min_valid_fraction{kMinValidFraction};
  double _front_overhang{kFrontOverhang};
  double _blocked_yaw_rate{kBlockedYawRate};

  // collision-stop state
  float _front_min_range{std::numeric_limits<float>::infinity()};
  // Starts at 0 => BLIND until the first scan proves otherwise. Never initialise
  // this optimistically: the window between activation and the first scan would
  // then be a fail-open hole of exactly the kind this gate exists to close.
  float _scan_valid_fraction{0.f};
  rclcpp::Time _last_scan_time{0, 0, RCL_ROS_TIME};
  bool _blocked{false};

  // passive on-stands diagnostic
  rclcpp::TimerBase::SharedPtr _diag_timer;
  DiagState _diag_last{DiagState::Clear};
  bool _diag_inited{false};
};
