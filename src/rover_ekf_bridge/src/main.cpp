#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

using namespace std::chrono_literals;

// Feeds wheel odometry velocity to EKF2 as an external-vision measurement so
// that local position/velocity become valid indoors (no GPS). Velocity only:
// wheel-integrated position drifts without bound, so we never send it as a
// position fix — EKF2 dead-reckons from the velocity aiding instead.
class OdomToEkf : public rclcpp::Node
{
public:
  OdomToEkf()
  : Node("rover_ekf_bridge")
  {
    _velocity_variance = declare_parameter<double>("velocity_variance", 0.05);
    _velocity_z_variance = declare_parameter<double>("velocity_z_variance", 0.05);
    _publish_rate_hz = declare_parameter<double>("publish_rate_hz", 50.0);
    _odom_timeout = declare_parameter<double>("odom_timeout", 0.5);

    _interface = std::make_unique<px4_ros2::LocalPositionMeasurementInterface>(
      *this, px4_ros2::PoseFrame::Unknown, px4_ros2::VelocityFrame::BodyFRD);

    _min_interval = rclcpp::Duration::from_seconds(1.0 / _publish_rate_hz);

    _odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::UniquePtr msg) {odomCallback(std::move(msg));});

    _watchdog = create_wall_timer(1s, [this]() {checkOdom();});

    RCLCPP_INFO(
      get_logger(), "rover_ekf_bridge up: /odom -> EKF2 (velocity only, BodyFRD) "
      "at %.0f Hz, variance %.3f", _publish_rate_hz, _velocity_variance);
  }

private:
  void odomCallback(nav_msgs::msg::Odometry::UniquePtr msg)
  {
    const rclcpp::Time now = get_clock()->now();
    _last_odom_time = now;

    if (_last_publish_time.nanoseconds() > 0 && (now - _last_publish_time) < _min_interval) {
      return;
    }
    _last_publish_time = now;

    // /odom twist is body frame FLU (x forward, y left); EKF2 wants FRD, so y flips.
    const float vx = static_cast<float>(msg->twist.twist.linear.x);
    const float vy = static_cast<float>(-msg->twist.twist.linear.y);

    if (!std::isfinite(vx) || !std::isfinite(vy)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "non-finite velocity in /odom, dropping");
      return;
    }

    px4_ros2::LocalPositionMeasurement measurement{};
    measurement.timestamp_sample = msg->header.stamp;
    measurement.velocity_xy = Eigen::Vector2f{vx, vy};
    measurement.velocity_xy_variance = Eigen::Vector2f{
      static_cast<float>(_velocity_variance), static_cast<float>(_velocity_variance)};
    // EKF2 drops the whole EV sample unless the velocity vector is all-finite
    // (ev_vel_control.cpp: ev._sample.vel.isAllFinite()), so z must be sent.
    // A wheeled rover on the ground has no vertical velocity.
    measurement.velocity_z = 0.0f;
    measurement.velocity_z_variance = static_cast<float>(_velocity_z_variance);

    try {
      _interface->update(measurement);
      ++_published;
    } catch (const px4_ros2::NavigationInterfaceInvalidArgument & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "rejected by interface: %s", e.what());
    }
  }

  void checkOdom()
  {
    if (_last_odom_time.nanoseconds() == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no /odom received yet");
      return;
    }

    if ((get_clock()->now() - _last_odom_time).seconds() > _odom_timeout) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "/odom stale — EKF2 aiding has stopped");
    }
  }

  std::unique_ptr<px4_ros2::LocalPositionMeasurementInterface> _interface;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
  rclcpp::TimerBase::SharedPtr _watchdog;

  double _velocity_variance{};
  double _velocity_z_variance{};
  double _publish_rate_hz{};
  double _odom_timeout{};
  rclcpp::Duration _min_interval{0, 0};
  rclcpp::Time _last_publish_time{0, 0, RCL_ROS_TIME};
  rclcpp::Time _last_odom_time{0, 0, RCL_ROS_TIME};
  uint64_t _published{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToEkf>());
  rclcpp::shutdown();
  return 0;
}
