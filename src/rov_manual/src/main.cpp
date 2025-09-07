#include "rclcpp/rclcpp.hpp"
#include "../include/manual.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoverManualNode>());
  rclcpp::shutdown();
  return 0;
}
