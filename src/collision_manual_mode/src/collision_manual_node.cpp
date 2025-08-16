// File: collision_manual_mode.cpp

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

// Non-blocking keyboard read
static int getch_nonblocking() {
  termios oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int ch = -1;
  if (read(STDIN_FILENO, &ch, 1) < 0) {
    ch = -1;
  }
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

class KeyboardManualControl : public rclcpp::Node {
public:
  KeyboardManualControl()
  : Node("keyboard_manual_control"), throttle_(0.0f)
  {
    // Publisher for manual stick input
    stick_pub_ = create_publisher<px4_msgs::msg::ManualControlSetpoint>(
      "/fmu/in/manual_control_input", rclcpp::SensorDataQoS());
    // Publisher for arm/disarm command
    cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      "/fmu/in/vehicle_command", rclcpp::SensorDataQoS());

    timer_ = create_wall_timer(100ms, std::bind(&KeyboardManualControl::on_timer, this));

    RCLCPP_INFO(get_logger(),
      "Keyboard control: 'w'/'s'=throttle up/down, 'a'/'d'=yaw, 'z'=ARM, 'x'=DISARM, 'q'=quit");
  }

private:
  void on_timer() {
    int c = getch_nonblocking();
    float yaw = 0.0f;

    // Throttle control
    if (c == 'w' || c == 'W') throttle_ = std::min(throttle_ + 0.1f, 1.0f);
    if (c == 's' || c == 'S') throttle_ = std::max(throttle_ - 0.1f, 0.0f);
    // Yaw control
    if (c == 'a' || c == 'A') yaw = -0.5f;
    if (c == 'd' || c == 'D') yaw = 0.5f;
    // Arm/Disarm
    if (c == 'z' || c == 'Z') send_arm(true);
    if (c == 'x' || c == 'X') send_arm(false);
    // Quit
    if (c == 'q' || c == 'Q') rclcpp::shutdown();

    // Publish stick setpoint if any control key
    if (c != -1) {
      px4_msgs::msg::ManualControlSetpoint msg;
      msg.timestamp = now().nanoseconds() / 1000;
      msg.timestamp_sample = msg.timestamp;
      msg.valid = true;
      msg.data_source = px4_msgs::msg::ManualControlSetpoint::SOURCE_RC;
      msg.roll = 0.0f;
      msg.pitch = 0.0f;
      msg.yaw = yaw;
      msg.throttle = throttle_;
      msg.flaps = 0.0f;
      msg.aux1 = msg.aux2 = msg.aux3 = msg.aux4 = msg.aux5 = msg.aux6 = 0.0f;
      msg.sticks_moving = (yaw != 0.0f || throttle_ > 0.0f);
      msg.buttons = 0;

      stick_pub_->publish(msg);
    }
  }

  void send_arm(bool arm) {
    px4_msgs::msg::VehicleCommand cmd;
    cmd.timestamp        = now().nanoseconds() / 1000;
    cmd.param1           = arm ? 1.0f : 0.0f;
    cmd.command          = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.target_system    = 1;
    cmd.target_component = 1;
    cmd.source_system    = 1;
    cmd.source_component = 1;
    cmd.from_external    = true;
    cmd_pub_->publish(cmd);
    RCLCPP_INFO(get_logger(), "%s command sent", arm ? "ARM" : "DISARM");
  }

  rclcpp::Publisher<px4_msgs::msg::ManualControlSetpoint>::SharedPtr stick_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr       cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  float throttle_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KeyboardManualControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
