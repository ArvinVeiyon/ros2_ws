#include <rclcpp/rclcpp.hpp>
#include "../include/collision_stop.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CollisionStopNode>());
  rclcpp::shutdown();
  return 0;
}
