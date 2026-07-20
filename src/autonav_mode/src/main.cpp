#include <autonav_mode/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>

#include <rclcpp/rclcpp.hpp>

using AutoNavNode = px4_ros2::NodeWithMode<AutoNavMode>;

static const std::string kNodeName = "autonav_mode";
static const bool kEnableDebugOutput = true;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoNavNode>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}
