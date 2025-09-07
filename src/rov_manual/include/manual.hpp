#pragma once
/****************************************************************************
 * Rover Manual Control Node (rov_manual)
 * - Subscribes: /fmu/out/manual_control_setpoint     (px4_msgs/ManualControlSetpoint)
 * - Publishes:  /fmu/in/rover_throttle_setpoint      (px4_msgs/RoverThrottleSetpoint)
 *               /fmu/in/rover_steering_setpoint      (px4_msgs/RoverSteeringSetpoint)
 *
 * Notes (PX4 v1.15+ messages on your Pi):
 * - RoverSteeringSetpoint uses `normalized_steering_setpoint` (NOT
 *   `normalized_steering_angle`). The old `normalized_speed_diff` field
 *   is also removed. This file matches the new schema.
 *
 * Behavior:
 * - Auto-enabled on startup (enabled=true). You can still toggle at runtime:
 *     ros2 param set /rov_manual enabled false
 *     ros2 param set /rov_manual enabled true
 ****************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/rover_throttle_setpoint.hpp>
#include <px4_msgs/msg/rover_steering_setpoint.hpp>
#include <cmath>

class RoverManualNode : public rclcpp::Node
{
public:
  RoverManualNode()
  : Node("rov_manual")
  {
    // Auto-enable at startup; still user-overridable via param
    enabled_ = declare_parameter<bool>("enabled", true);
    RCLCPP_INFO(get_logger(),
                "rov_manual started (auto-enabled=%s). Publishing RC → rover setpoints.",
                enabled_ ? "true" : "false");

    // XRCE-DDS paths on PX4 generally expect BEST_EFFORT for /fmu/in
    auto qos = rclcpp::QoS(10).best_effort();

    thr_pub_ = create_publisher<px4_msgs::msg::RoverThrottleSetpoint>(
        "/fmu/in/rover_throttle_setpoint", qos);
    str_pub_ = create_publisher<px4_msgs::msg::RoverSteeringSetpoint>(
        "/fmu/in/rover_steering_setpoint", qos);

    sub_ = create_subscription<px4_msgs::msg::ManualControlSetpoint>(
        "/fmu/out/manual_control_setpoint", qos,
        std::bind(&RoverManualNode::handleManualInput, this, std::placeholders::_1));

    // Allow runtime enable/disable via parameter
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        for (const auto &p : params) {
          if (p.get_name() == "enabled") {
            enabled_ = p.as_bool();
            RCLCPP_INFO(get_logger(), "rov_manual: enabled set to %s",
                        enabled_ ? "true" : "false");
          }
        }
        rcl_interfaces::msg::SetParametersResult r; r.successful = true; return r;
      });
  }

private:
  void handleManualInput(const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
  {
    if (!enabled_) return;

    // Map joystick/RC sticks → rover throttle + steering
    // Throttle forward/back
    px4_msgs::msg::RoverThrottleSetpoint thr{};
    thr.timestamp = now().nanoseconds();
    thr.throttle_body_x = msg->throttle;   // normalized [-1, +1]
    thr.throttle_body_y = 0.0f;            // Ackermann: Y unused
    thr_pub_->publish(thr);

    // Steering left/right (NEW FIELD NAME)
    px4_msgs::msg::RoverSteeringSetpoint str{};
    str.timestamp = now().nanoseconds();
    str.normalized_steering_setpoint = msg->roll;  // normalized [-1, +1]
    str_pub_->publish(str);

    RCLCPP_DEBUG(get_logger(), "RC→Rover: throttle=%.2f, steering=%.2f",
                 thr.throttle_body_x, str.normalized_steering_setpoint);
  }

  bool enabled_{true};

  rclcpp::Publisher<px4_msgs::msg::RoverThrottleSetpoint>::SharedPtr thr_pub_;
  rclcpp::Publisher<px4_msgs::msg::RoverSteeringSetpoint>::SharedPtr str_pub_;
  rclcpp::Subscription<px4_msgs::msg::ManualControlSetpoint>::SharedPtr sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};
