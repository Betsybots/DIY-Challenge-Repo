#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class WallDistanceNavigator : public rclcpp::Node
{
public:
  WallDistanceNavigator()
  : Node("wall_distance_navigator")
  {
    pointcloud_topic_ = this->declare_parameter<std::string>("pointcloud_topic", "/lidar_points");
    cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    side_half_angle_deg_ = this->declare_parameter<double>("side_half_angle_deg", 25.0);
    min_valid_range_m_ = this->declare_parameter<double>("min_valid_range_m", 0.05);
    max_valid_range_m_ = this->declare_parameter<double>("max_valid_range_m", 30.0);
    linear_velocity_mps_ = this->declare_parameter<double>("linear_velocity_mps", 0.2);
    kp_ = this->declare_parameter<double>("kp", 1.2);
    ki_ = this->declare_parameter<double>("ki", 0.0);
    kd_ = this->declare_parameter<double>("kd", 0.08);
    max_angular_velocity_rps_ = this->declare_parameter<double>("max_angular_velocity_rps", 1.2);
    integral_limit_ = this->declare_parameter<double>("integral_limit", 2.0);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&WallDistanceNavigator::pointcloudCallback, this, std::placeholders::_1));

    prev_time_ = this->now();

    RCLCPP_INFO(
      this->get_logger(),
      "Listening on %s and publishing PID cmd_vel to %s",
      pointcloud_topic_.c_str(),
      cmd_vel_topic_.c_str());
  }

private:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kHalfPi = 1.57079632679489661923;

  static double deg2rad(const double deg)
  {
    return deg * kPi / 180.0;
  }

  static bool inAngleWindow(const double angle, const double center, const double half_width)
  {
    double diff = std::atan2(std::sin(angle - center), std::cos(angle - center));
    return std::fabs(diff) <= half_width;
  }

  geometry_msgs::msg::Twist computePidCmdVel(const float left_distance, const float right_distance)
  {
    geometry_msgs::msg::Twist cmd;

    const bool left_valid = std::isfinite(left_distance);
    const bool right_valid = std::isfinite(right_distance);
    if (!(left_valid && right_valid)) {
      integral_error_ = 0.0;
      prev_error_ = 0.0;
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Missing side wall points (left_valid=%d, right_valid=%d); stopping robot",
        left_valid,
        right_valid);
      return cmd;
    }

    const rclcpp::Time now = this->now();
    double dt = (now - prev_time_).seconds();
    if (dt <= 1e-3) {
      dt = 1e-3;
    }
    prev_time_ = now;

    // Positive error means robot is closer to right wall and should turn left.
    const double error = static_cast<double>(left_distance - right_distance);
    integral_error_ += error * dt;
    integral_error_ = std::clamp(integral_error_, -integral_limit_, integral_limit_);
    const double derivative = (error - prev_error_) / dt;
    prev_error_ = error;

    double angular_z = kp_ * error + ki_ * integral_error_ + kd_ * derivative;
    angular_z = std::clamp(angular_z, -max_angular_velocity_rps_, max_angular_velocity_rps_);

    cmd.linear.x = linear_velocity_mps_;
    cmd.angular.z = angular_z;
    return cmd;
  }

  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const double side_half_angle = deg2rad(side_half_angle_deg_);

    float left_min = std::numeric_limits<float>::infinity();
    float right_min = std::numeric_limits<float>::infinity();

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      const float x = *iter_x;
      const float y = *iter_y;

      if (!std::isfinite(x) || !std::isfinite(y)) {
        continue;
      }

      const float range = std::hypot(x, y);
      if (range < min_valid_range_m_ || range > max_valid_range_m_) {
        continue;
      }

      const double angle = std::atan2(y, x);

      if (inAngleWindow(angle, kHalfPi, side_half_angle)) {
        left_min = std::min(left_min, range);
      }
      if (inAngleWindow(angle, -kHalfPi, side_half_angle)) {
        right_min = std::min(right_min, range);
      }
    }

    const bool left_valid = std::isfinite(left_min);
    const bool right_valid = std::isfinite(right_min);
    geometry_msgs::msg::Twist cmd = computePidCmdVel(left_min, right_min);

    cmd_vel_pub_->publish(cmd);

    RCLCPP_DEBUG(
      this->get_logger(),
      "Side distances [left, right]=[%.3f, %.3f] -> cmd_vel [vx=%.3f, wz=%.3f]",
      left_valid ? left_min : -1.0F,
      right_valid ? right_min : -1.0F,
      cmd.linear.x,
      cmd.angular.z);
  }

  std::string pointcloud_topic_;
  std::string cmd_vel_topic_;
  double side_half_angle_deg_;
  double min_valid_range_m_;
  double max_valid_range_m_;
  double linear_velocity_mps_;
  double kp_;
  double ki_;
  double kd_;
  double max_angular_velocity_rps_;
  double integral_limit_;
  double integral_error_ {0.0};
  double prev_error_ {0.0};
  rclcpp::Time prev_time_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallDistanceNavigator>());
  rclcpp::shutdown();
  return 0;
}
