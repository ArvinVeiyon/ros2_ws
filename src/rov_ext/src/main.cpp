#include "rclcpp/rclcpp.hpp"
#include <px4_ros2/components/node_with_mode.hpp>
#include <string>

#include "../include/mode.hpp"

// Wrap our ModeBase class so rclcpp::spin works
using MyNodeWithMode = px4_ros2::NodeWithMode<FlightModeTest>;
static const std::string kNodeName = "rov_ext";
static const bool kEnableDebugOutput = true;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNodeWithMode>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}
