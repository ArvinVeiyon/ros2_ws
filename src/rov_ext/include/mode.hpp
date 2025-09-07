#pragma once
/****************************************************************************
 * rov_ext — Rover External Control (PX4 + ROS 2)
 *
 * WHAT THIS NODE DOES
 * - Registers one External component with PX4 (self-registration only).
 * - Publishes a "control config heartbeat" on /fmu/in/config_control_setpoints:
 *     * AUTO + ALLOCATION enabled
 *     * all other controller flags disabled (no MC loops)
 * - Publishes rover setpoints (throttle/steering) when `enabled=true`.
 * - Optionally publishes a GotoSetpoint “hold” (0 speed) keepalive.
 * - Mirrors PX4’s active External slot `source_id` by subscribing to
 *   /fmu/out/vehicle_control_mode so the heartbeat always matches FMU.
 *
 * WHY SELF-REGISTRATION ONLY?
 * - Avoids double registration (ModeBase + self), which leads to multiple
 *   External slots (e.g., External 1 & 2) and QGC confusion.
 * - You can flip to ModeBase registration later if you want, but this base
 *   keeps a single registrar to stay deterministic.
 *
 * KEY TOPICS
 *   IN (published by this node):
 *     /fmu/in/config_control_setpoints   (px4_msgs/VehicleControlMode)
 *     /fmu/in/rover_throttle_setpoint    (px4_msgs/RoverThrottleSetpoint)
 *     /fmu/in/rover_steering_setpoint    (px4_msgs/RoverSteeringSetpoint)
 *     /fmu/in/goto_setpoint              (px4_msgs/GotoSetpoint) [optional]
 *     /fmu/in/register_ext_component_request (px4_msgs/RegisterExtComponentRequest)
 *     /fmu/in/vehicle_command            (px4_msgs/VehicleCommand)
 *
 *   OUT (subscribed by this node):
 *     /fmu/out/vehicle_control_mode      (mirror source_id + status)
 *     /fmu/out/register_ext_component_reply (registration ack)
 *     /fmu/out/arming_check_request      (debug-only)
 *
 * CONTROL LOOP RATES (defaults; configurable via ROS params)
 *   heartbeat_hz = 1   (recommend 2–5 for robustness)
 *   setpoint_hz  = 20  (rover setpoints when enabled=true)
 *
 * SAFE OPERATION CHECKLIST
 * - Ensure exactly ONE registrar is active (this file does that by default).
 * - Keep a continuous signal in External mode:
 *     * set `enabled=true` (even with zero throttle/steer)
 *     * or `use_goto_keepalive=true` to send a 0-speed hold
 * - Consider setting heartbeat_hz >= 2 to avoid “control signal lost”.
 ****************************************************************************/

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>

#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/register_ext_component_request.hpp>
#include <px4_msgs/msg/register_ext_component_reply.hpp>
#include <px4_msgs/msg/arming_check_request.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/rover_throttle_setpoint.hpp>
#include <px4_msgs/msg/rover_steering_setpoint.hpp>
#include <px4_msgs/msg/goto_setpoint.hpp>

#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <cstring>
#include <string>
#include <chrono>
#include <limits>

using namespace std::chrono_literals;

// Name shown by ModeBase in logs; the QGC external name uses `component_name` param below.
static const std::string kModeName = "Rover_External_Control";

static inline float clampf(float v, float lo, float hi) {
  return std::min(std::max(v, lo), hi);
}

class FlightModeTest : public px4_ros2::ModeBase
{
public:
  explicit FlightModeTest(rclcpp::Node & node)
  : ModeBase(node, kModeName), node_(&node)
  {
    // Be tolerant to minor message definition drift across versions
    this->setSkipMessageCompatibilityCheck();

    // ---------- Parameters ----------
    external_slot_   = declare_param<int>("external_slot", 1);
    source_id_       = (external_slot_ == 2) ? 24u : 23u; // (External1=23, External2=24, External3=25...)
    heartbeat_hz_    = std::max<int>(1, declare_param<int>("heartbeat_hz", 1));
    setpoint_hz_     = std::max<int>(1, declare_param<int>("setpoint_hz", 20));

    do_register_     = declare_param<bool>("do_register", true);     // self-registration only
    reg_arm_check_   = declare_param<bool>("register_arming_check", false);
    reg_mode_        = declare_param<bool>("register_mode", false);
    reg_mode_exec_   = declare_param<bool>("register_mode_executor", false);

    comp_name_       = declare_param<std::string>("component_name", "rov_ext");

    enabled_         = declare_param<bool>("enabled", false);
    use_goto_keepalive_ = declare_param<bool>("use_goto_keepalive", false);

    // Rover command inputs (normalized in [-1, +1])
    throttle_x_      = clampf((float)declare_param<double>("throttle_body_x", 0.0), -1.f, 1.f);
    throttle_y_      = clampf((float)declare_param<double>("throttle_body_y", 0.0), -1.f, 1.f);
    steering_        = clampf((float)declare_param<double>("steering_angle",   0.0), -1.f, 1.f);

    // Live parameter updates
    param_cb_ = node_->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &ps) {
        for (const auto &p : ps) {
          const auto &n = p.get_name();
          if (n == "external_slot")       { external_slot_ = p.as_int(); }
          else if (n == "heartbeat_hz")   { heartbeat_hz_ = std::max<int>(1, (int)p.as_int()); if (cfg_timer_)  cfg_timer_->reset(); }
          else if (n == "setpoint_hz")    { setpoint_hz_  = std::max<int>(1, (int)p.as_int()); if (tick_timer_) tick_timer_->reset(); }
          else if (n == "do_register")    { do_register_  = p.as_bool(); }
          else if (n == "register_arming_check")  { reg_arm_check_ = p.as_bool(); }
          else if (n == "register_mode")          { reg_mode_ = p.as_bool(); }
          else if (n == "register_mode_executor") { reg_mode_exec_ = p.as_bool(); }
          else if (n == "component_name") { comp_name_ = p.as_string(); }
          else if (n == "enabled")        { enabled_ = p.as_bool(); }
          else if (n == "use_goto_keepalive") { use_goto_keepalive_ = p.as_bool(); }
          else if (n == "throttle_body_x"){ throttle_x_ = clampf(p.as_double(), -1.0, 1.0); }
          else if (n == "throttle_body_y"){ throttle_y_ = clampf(p.as_double(), -1.0, 1.0); }
          else if (n == "steering_angle") { steering_   = clampf(p.as_double(), -1.0, 1.0); }
        }
        rcl_interfaces::msg::SetParametersResult r; r.successful = true; return r;
      });

    // ---------- Publishers / Subscribers ----------
    auto be = rclcpp::QoS(10).best_effort();

    cfg_pub_  = node_->create_publisher<px4_msgs::msg::VehicleControlMode>    ("/fmu/in/config_control_setpoints", be);
    thr_pub_  = node_->create_publisher<px4_msgs::msg::RoverThrottleSetpoint> ("/fmu/in/rover_throttle_setpoint",  be);
    str_pub_  = node_->create_publisher<px4_msgs::msg::RoverSteeringSetpoint> ("/fmu/in/rover_steering_setpoint",  be);
    goto_pub_ = node_->create_publisher<px4_msgs::msg::GotoSetpoint>          ("/fmu/in/goto_setpoint",            be);

    reg_pub_  = node_->create_publisher<px4_msgs::msg::RegisterExtComponentRequest>("/fmu/in/register_ext_component_request", 10);
    reg_sub_  = node_->create_subscription<px4_msgs::msg::RegisterExtComponentReply>(
      "/fmu/out/register_ext_component_reply", rclcpp::QoS(10).best_effort(),
      [this](px4_msgs::msg::RegisterExtComponentReply::SharedPtr){
        RCLCPP_INFO(node_->get_logger(),"Got RegisterExtComponentReply");
      });

    armchk_sub_ = node_->create_subscription<px4_msgs::msg::ArmingCheckRequest>(
      "/fmu/out/arming_check_request", rclcpp::QoS(10).best_effort(),
      [this](px4_msgs::msg::ArmingCheckRequest::SharedPtr m){
        if (!printed_armchk_) {
          printed_armchk_ = true;
          RCLCPP_DEBUG(node_->get_logger(),"Arming check request (id=%u, only printed once)", m->request_id);
        }
      });

    cmd_pub_  = node_->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

    vcm_sub_ = node_->create_subscription<px4_msgs::msg::VehicleControlMode>(
      "/fmu/out/vehicle_control_mode", rclcpp::QoS(10).best_effort(),
      [this](const px4_msgs::msg::VehicleControlMode::SharedPtr m)
      {
        if (m->source_id != source_id_) {
          source_id_ = m->source_id; // adopt FMU slot
          if (!printed_source_warn_) {
            printed_source_warn_ = true;
            RCLCPP_WARN(node_->get_logger(),
                        "FMU active slot source_id=%u; mirroring to match.",
                        source_id_);
          }
        }
      });

    // ---------- Timers ----------
    tick_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(1000/std::max(1,setpoint_hz_)),
      std::bind(&FlightModeTest::tick,this));

    cfg_timer_  = node_->create_wall_timer(
      std::chrono::milliseconds(1000/std::max(1,heartbeat_hz_)),
      std::bind(&FlightModeTest::publishConfig,this));

    lpos_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

    _rates_sp_ = std::make_shared<px4_ros2::RatesSetpointType>(*this);
    _att_sp_   = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
  }

  void onActivate() override
  {
    burst_left_ = 10;
    burst_timer_ = node_->create_wall_timer(50ms, std::bind(&FlightModeTest::burstTick, this));

    if (do_register_) {
      sendRegistration();
    } else {
      RCLCPP_WARN(node_->get_logger(), "Skipping registration (param do_register=false)");
    }
  }

  void onDeactivate() override {}
  void updateSetpoint(float) override {}

private:
  template<typename T> T declare_param(const std::string &name, const T &def){
    return node_->declare_parameter<T>(name, def);
  }

  void sendRegistration()
  {
    px4_msgs::msg::RegisterExtComponentRequest req{};
    req.timestamp = 0;
    std::fill(req.name.begin(), req.name.end(), 0);
    const size_t n = std::min(comp_name_.size(), req.name.size()-1);
    std::memcpy(req.name.data(), comp_name_.data(), n);
    req.register_arming_check  = reg_arm_check_;
    req.register_mode          = reg_mode_;
    req.register_mode_executor = reg_mode_exec_;
    reg_pub_->publish(req);
  }

  void publishConfig()
  {
    px4_msgs::msg::VehicleControlMode m{};
    m.timestamp = 0;

    m.flag_armed = false;
    m.flag_multicopter_position_control_enabled = false;

    m.flag_control_manual_enabled       = false;
    m.flag_control_auto_enabled         = true;
    m.flag_control_offboard_enabled     = false;

    m.flag_control_position_enabled     = false;
    m.flag_control_velocity_enabled     = false;
    m.flag_control_altitude_enabled     = false;
    m.flag_control_climb_rate_enabled   = false;
    m.flag_control_acceleration_enabled = false;
    m.flag_control_attitude_enabled     = false;
    m.flag_control_rates_enabled        = false;

    m.flag_control_allocation_enabled   = true;
    m.flag_control_termination_enabled  = false;

    m.source_id = source_id_;
    cfg_pub_->publish(m);
  }

  void publishGotoKeepAlive()
  {
    if (!use_goto_keepalive_) return;
    const auto ned = lpos_->positionNed();
    px4_msgs::msg::GotoSetpoint g{};
    g.timestamp = 0;
    g.position[0]=ned.x(); g.position[1]=ned.y(); g.position[2]=ned.z();
    g.flag_control_heading=false; g.heading=std::numeric_limits<float>::quiet_NaN();
    g.flag_set_max_horizontal_speed=true; g.max_horizontal_speed=0.0f;
    g.flag_set_max_vertical_speed=false;  g.max_vertical_speed  =0.0f;
    g.flag_set_max_heading_rate=true;     g.max_heading_rate    =0.7854f;
    goto_pub_->publish(g);
  }

  void publishRoverSetpoints()
  {
    if (!enabled_) return;

    // Throttle
    px4_msgs::msg::RoverThrottleSetpoint t{};
    t.timestamp = 0;
    t.throttle_body_x = throttle_x_;
    t.throttle_body_y = throttle_y_;
    thr_pub_->publish(t);

    // Steering (new PX4 field name)
    px4_msgs::msg::RoverSteeringSetpoint s{};
    s.timestamp = 0;
    s.normalized_steering_setpoint = steering_;
    str_pub_->publish(s);
  }

  void burstTick()
  {
    if (burst_left_-- > 0) { publishConfig(); if (enabled_) publishRoverSetpoints(); }
    else { burst_timer_->cancel(); }
  }

  void tick()
  {
    publishRoverSetpoints();
    publishGotoKeepAlive();
  }

private:
  rclcpp::Node* node_{nullptr};

  int external_slot_{1};
  unsigned source_id_{23};
  int heartbeat_hz_{1};
  int setpoint_hz_{20};

  bool do_register_{true};
  bool reg_arm_check_{false};
  bool reg_mode_{false};
  bool reg_mode_exec_{false};
  std::string comp_name_{"rov_ext"};

  bool enabled_{false};
  bool use_goto_keepalive_{false};

  float throttle_x_{0.f}, throttle_y_{0.f}, steering_{0.f};

  rclcpp::Publisher<px4_msgs::msg::VehicleControlMode>::SharedPtr          cfg_pub_;
  rclcpp::Publisher<px4_msgs::msg::RoverThrottleSetpoint>::SharedPtr       thr_pub_;
  rclcpp::Publisher<px4_msgs::msg::RoverSteeringSetpoint>::SharedPtr       str_pub_;
  rclcpp::Publisher<px4_msgs::msg::GotoSetpoint>::SharedPtr                goto_pub_;
  rclcpp::Publisher<px4_msgs::msg::RegisterExtComponentRequest>::SharedPtr reg_pub_;
  rclcpp::Subscription<px4_msgs::msg::RegisterExtComponentReply>::SharedPtr reg_sub_;
  rclcpp::Subscription<px4_msgs::msg::ArmingCheckRequest>::SharedPtr       armchk_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr              cmd_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vcm_sub_;
  bool printed_source_warn_{false};

  std::shared_ptr<px4_ros2::OdometryLocalPosition> lpos_;
  rclcpp::TimerBase::SharedPtr tick_timer_, cfg_timer_, burst_timer_;
  int burst_left_{0};
  int64_t last_cfg_log_ns_{0};
  bool printed_armchk_{false};

  std::shared_ptr<px4_ros2::RatesSetpointType>    _rates_sp_;
  std::shared_ptr<px4_ros2::AttitudeSetpointType> _att_sp_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};
