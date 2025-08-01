#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/distance_sensor.hpp>

// PX4-ROS2 Interface Library components
#include <px4_ros2/components/mode.hpp>                    // ModeBase
#include <px4_ros2/components/manual_control_input.hpp>    // ManualControlInput
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>  // RatesSetpointType

class CollisionManualMode : public px4_ros2::ModeBase {
public:
  // Store a reference to the ROS node for logging
  explicit CollisionManualMode(rclcpp::Node &node)
    : ModeBase(node, Settings{"Collision Manual Mode"}),
      _node(node),
      _manual_input(std::make_shared<px4_ros2::ManualControlInput>(*this)),
      _rates_setpoint(std::make_shared<px4_ros2::RatesSetpointType>(*this))
  {
  // disable the built-in PX4 <-> ROS2 msg version check
      setSkipMessageCompatibilityCheck();
  // Subscribe to TFmini distance sensor on /fmu/in/distance_sensor
    _node.create_subscription<px4_msgs::msg::DistanceSensor>(
      "/fmu/in/distance_sensor", 10,
      [this](px4_msgs::msg::DistanceSensor::UniquePtr msg) {
        _latest_distance = msg->current_distance;
      });

    RCLCPP_INFO(_node.get_logger(), "CollisionManualMode initialized");
  }

  void onActivate() override {
    RCLCPP_INFO(_node.get_logger(), "Collision Manual Mode Activated");
  }

  void onDeactivate() override {
    RCLCPP_INFO(_node.get_logger(), "Collision Manual Mode Deactivated");
  }

  // Must exactly match ModeBase signature
  void updateSetpoint(float /*dt_s*/) override {
    // PX4 throttle() is negative‐downwards
    float thrust = -_manual_input->throttle();
    Eigen::Vector3f rates{
      _manual_input->roll()   * 150.0f * static_cast<float>(M_PI) / 180.0f,
      -_manual_input->pitch() * 150.0f * static_cast<float>(M_PI) / 180.0f,
      _manual_input->yaw()    * 100.0f * static_cast<float>(M_PI) / 180.0f
    };

    // Block forward thrust if obstacle closer than 0.2 m
    if (_latest_distance > 0.0f && _latest_distance < 0.2f) {
      RCLCPP_WARN(
        _node.get_logger(),
        "Obstacle at %.2f m — blocking thrust",
        _latest_distance);
      thrust = 0.0f;
    }

    // Publish rates + thrust setpoint
    _rates_setpoint->update(rates, Eigen::Vector3f{0.0f, 0.0f, thrust});
  }

private:
  rclcpp::Node &_node;
  std::shared_ptr<px4_ros2::ManualControlInput> _manual_input;
  std::shared_ptr<px4_ros2::RatesSetpointType>  _rates_setpoint;
  float _latest_distance{-1.0f};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("collision_manual_mode");
  auto mode = std::make_shared<CollisionManualMode>(*node);

  if (!mode->doRegister()) {
    RCLCPP_ERROR(node->get_logger(), "CollisionManualMode registration failed");
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
