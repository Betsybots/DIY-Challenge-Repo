#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstddef>
#include <functional>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

constexpr double kInchesToMeters = 0.0254;
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace

class MotionPlanExecutor : public rclcpp::Node
{
public:
  MotionPlanExecutor()
  : Node("motion_plan_executor")
  {
    plan_file_ = this->declare_parameter<std::string>("plan_file", "");
    pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/ground_truth_pose");
    cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "/cmd_vel_smoothed");
    straight_yaw_enabled_ = this->declare_parameter<bool>("straight_yaw_enabled", true);
    straight_yaw_kp_ = this->declare_parameter<double>("straight_yaw_kp", 1.5);
    straight_yaw_ki_ = this->declare_parameter<double>("straight_yaw_ki", 0.0);
    straight_yaw_kd_ = this->declare_parameter<double>("straight_yaw_kd", 0.05);
    straight_yaw_integral_limit_ = this->declare_parameter<double>(
      "straight_yaw_integral_limit", 0.5);
    straight_yaw_max_angular_velocity_ = this->declare_parameter<double>(
      "straight_yaw_max_angular_velocity", 0.5);

    if (plan_file_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "Parameter 'plan_file' is required.");
      throw std::runtime_error("Missing required parameter: plan_file");
    }

    load_plan(plan_file_);

    if (
      !std::isfinite(straight_yaw_kp_) || straight_yaw_kp_ < 0.0 ||
      !std::isfinite(straight_yaw_ki_) || straight_yaw_ki_ < 0.0 ||
      !std::isfinite(straight_yaw_kd_) || straight_yaw_kd_ < 0.0 ||
      !std::isfinite(straight_yaw_integral_limit_) || straight_yaw_integral_limit_ < 0.0 ||
      !std::isfinite(straight_yaw_max_angular_velocity_) ||
      straight_yaw_max_angular_velocity_ <= 0.0)
    {
      throw std::runtime_error("Straight yaw PID parameters are invalid.");
    }

    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_,
      rclcpp::QoS(10),
      std::bind(&MotionPlanExecutor::pose_callback, this, std::placeholders::_1));

    green_light_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/green_light",
      rclcpp::QoS(10),
      std::bind(&MotionPlanExecutor::green_light_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, rclcpp::QoS(10));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&MotionPlanExecutor::control_loop, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded %zu commands from %s",
      commands_.size(),
      plan_file_.c_str());
  }

private:
  enum class CommandType
  {
    STRAIGHT,
    TURN
  };

  struct Command
  {
    CommandType type{CommandType::STRAIGHT};
    double target{0.0};
    double speed{0.0};
    double turn_radius{0.0};
  };

  void load_plan(const std::string & yaml_path)
  {
    if (!std::filesystem::exists(yaml_path)) {
      throw std::runtime_error("Plan file does not exist: " + yaml_path);
    }

    YAML::Node root = YAML::LoadFile(yaml_path);

    // Optional yaw-hold PID overrides from the plan file.
    if (const YAML::Node pid = root["straight_yaw_pid"]) {
      if (pid["enabled"]) {straight_yaw_enabled_ = pid["enabled"].as<bool>();}
      if (pid["kp"]) {straight_yaw_kp_ = pid["kp"].as<double>();}
      if (pid["ki"]) {straight_yaw_ki_ = pid["ki"].as<double>();}
      if (pid["kd"]) {straight_yaw_kd_ = pid["kd"].as<double>();}
      if (pid["integral_limit"]) {
        straight_yaw_integral_limit_ = pid["integral_limit"].as<double>();
      }
      if (pid["max_angular_velocity"]) {
        straight_yaw_max_angular_velocity_ = pid["max_angular_velocity"].as<double>();
      }
    }

    YAML::Node motions = root;

    if (root["motions"]) {
      motions = root["motions"];
    }

    if (!motions.IsSequence()) {
      throw std::runtime_error(
              "YAML format error: expected a sequence at root or under 'motions'.");
    }

    const auto read_speed = [](const YAML::Node & entry) {
        if (entry["velocity"]) {
          return entry["velocity"].as<double>();
        }
        if (entry["linear_velocity"]) {
          return entry["linear_velocity"].as<double>();
        }
        throw std::runtime_error("Motion entry requires 'velocity'.");
      };

    for (std::size_t i = 0; i < motions.size(); ++i) {
      const YAML::Node entry = motions[i];
      if (!entry.IsMap()) {
        throw std::runtime_error("YAML format error: each motion entry must be a map.");
      }

      std::string type;
      if (entry["type"]) {
        type = entry["type"].as<std::string>();
      } else if (entry["command"]) {
        type = entry["command"].as<std::string>();
      } else if (entry["action"]) {
        type = entry["action"].as<std::string>();
      } else {
        throw std::runtime_error("YAML format error: entry missing type/command/action.");
      }

      if (type == "STRAIGHT") {
        if (!entry["distance"]) {
          throw std::runtime_error(
                  "STRAIGHT entry requires 'distance' and 'velocity'.");
        }
        const double distance_inches = entry["distance"].as<double>();
        const double speed = read_speed(entry);
        if (!std::isfinite(distance_inches) || !std::isfinite(speed) || speed <= 0.0) {
          throw std::runtime_error("STRAIGHT distance and velocity must be finite and positive.");
        }
        commands_.push_back(Command{
          CommandType::STRAIGHT, distance_inches * kInchesToMeters,
          speed, 0.0});
      } else if (type == "TURN") {
        if (!entry["turn_radius"] || !entry["velocity"] || !entry["stop_angle"]) {
          throw std::runtime_error(
                  "TURN entry requires 'turn_radius', 'velocity', and 'stop_angle'.");
        }
        const double turn_radius_inches = entry["turn_radius"].as<double>();
        const double velocity = entry["velocity"].as<double>();
        const double stop_angle_degrees = entry["stop_angle"].as<double>();
        if (!std::isfinite(turn_radius_inches) || !std::isfinite(velocity) ||
          !std::isfinite(stop_angle_degrees) || turn_radius_inches <= 0.0 ||
          velocity <= 0.0 || stop_angle_degrees == 0.0)
        {
          throw std::runtime_error(
              "TURN entry requires positive 'turn_radius' and 'velocity' and non-zero 'stop_angle'.");
        }
        const double turn_radius = turn_radius_inches * kInchesToMeters;
        const double stop_angle = stop_angle_degrees * kDegreesToRadians;
        commands_.push_back(Command{
          CommandType::TURN, stop_angle, velocity, turn_radius});
      } else {
        throw std::runtime_error("Unsupported command type: " + type);
      }
    }

    if (commands_.empty()) {
      throw std::runtime_error("Plan contains no commands.");
    }
  }

  void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_pose_ = *msg;
    has_pose_ = true;

    if (track_turn_yaw_) {
      const double current_yaw = yaw_from_quaternion(msg->pose.pose.orientation);
      accumulated_turn_angle_ += normalize_angle(current_yaw - last_turn_yaw_);
      last_turn_yaw_ = current_yaw;
    }
  }

  void green_light_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    // Latched: the first true starts the plan; later false is ignored.
    if (msg->data && !started_) {
      started_ = true;
      RCLCPP_INFO(this->get_logger(), "Green light received — starting motion plan.");
    }
  }

  void control_loop()
  {
    if (!started_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for true on /green_light before starting the motion plan");
      publish_stop();
      return;
    }

    if (!has_pose_) {
      publish_stop();
      return;
    }

    if (current_index_ >= commands_.size()) {
      if (!done_logged_) {
        RCLCPP_INFO(this->get_logger(), "Motion plan completed.");
        done_logged_ = true;
      }
      publish_stop();
      return;
    }

    const Command & cmd = commands_[current_index_];

    if (!segment_started_) {
      segment_started_ = true;
      start_x_ = last_pose_.pose.pose.position.x;
      start_y_ = last_pose_.pose.pose.position.y;
      start_yaw_ = yaw_from_quaternion(last_pose_.pose.pose.orientation);
      last_turn_yaw_ = start_yaw_;
      accumulated_turn_angle_ = 0.0;
      track_turn_yaw_ = (cmd.type == CommandType::TURN);
      straight_yaw_integral_ = 0.0;
      previous_straight_yaw_error_ = 0.0;
      last_control_time_ = std::chrono::steady_clock::now();

      RCLCPP_INFO(
        this->get_logger(),
        "Starting command %zu/%zu: %s target=%.3f speed=%.3f",
        current_index_ + 1,
        commands_.size(),
        (cmd.type == CommandType::STRAIGHT ? "STRAIGHT" : "TURN"),
        cmd.target,
        cmd.speed);
    }

    geometry_msgs::msg::Twist twist;

    if (cmd.type == CommandType::STRAIGHT) {
      const double dx = last_pose_.pose.pose.position.x - start_x_;
      const double dy = last_pose_.pose.pose.position.y - start_y_;
      const double traveled = std::hypot(dx, dy);
      const double target_distance = std::abs(cmd.target);

      if (traveled >= target_distance) {
        advance_command();
        publish_stop();
        return;
      }

      const double speed = std::copysign(std::abs(cmd.speed), cmd.target);
      twist.linear.x = speed;
      twist.angular.z = 0.0;

      if (straight_yaw_enabled_) {
        const auto now = std::chrono::steady_clock::now();
        const double dt = std::clamp(
          std::chrono::duration<double>(now - last_control_time_).count(), 1e-3, 0.2);
        last_control_time_ = now;

        const double current_yaw = yaw_from_quaternion(last_pose_.pose.pose.orientation);
        const double yaw_error = normalize_angle(start_yaw_ - current_yaw);
        straight_yaw_integral_ = std::clamp(
          straight_yaw_integral_ + yaw_error * dt,
          -straight_yaw_integral_limit_, straight_yaw_integral_limit_);
        const double yaw_error_rate = (yaw_error - previous_straight_yaw_error_) / dt;
        previous_straight_yaw_error_ = yaw_error;

        twist.angular.z = std::clamp(
          straight_yaw_kp_ * yaw_error +
          straight_yaw_ki_ * straight_yaw_integral_ +
          straight_yaw_kd_ * yaw_error_rate,
          -straight_yaw_max_angular_velocity_, straight_yaw_max_angular_velocity_);

        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "STRAIGHT yaw error=%.2f deg, correction=%.3f rad/s",
          yaw_error * 180.0 / 3.14159265358979323846,
          twist.angular.z);
      }
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "TURN progress: %.1f / %.1f deg",
        accumulated_turn_angle_ * 180.0 / 3.14159265358979323846,
        cmd.target * 180.0 / 3.14159265358979323846);
      const bool turn_complete = cmd.target > 0.0 ?
        accumulated_turn_angle_ >= cmd.target : accumulated_turn_angle_ <= cmd.target;
      if (turn_complete) {
        advance_command();
        publish_stop();
        return;
      }

      twist.linear.x = cmd.speed;
      twist.angular.z = std::copysign(
        cmd.speed / cmd.turn_radius, cmd.target);
    }

    cmd_pub_->publish(twist);
  }

  void advance_command()
  {
    ++current_index_;
    segment_started_ = false;
    track_turn_yaw_ = false;
  }

  void publish_stop()
  {
    geometry_msgs::msg::Twist stop;
    stop.linear.x = 0.0;
    stop.angular.z = 0.0;
    cmd_pub_->publish(stop);
  }

  std::string plan_file_;
  std::string pose_topic_;
  std::string cmd_topic_;

  std::vector<Command> commands_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr green_light_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::Odometry last_pose_;
  bool has_pose_{false};
  bool started_{false};

  std::size_t current_index_{0};
  bool segment_started_{false};
  bool done_logged_{false};

  double start_x_{0.0};
  double start_y_{0.0};
  double start_yaw_{0.0};
  double last_turn_yaw_{0.0};
  double accumulated_turn_angle_{0.0};
  bool track_turn_yaw_{false};
  bool straight_yaw_enabled_{true};
  double straight_yaw_kp_{1.5};
  double straight_yaw_ki_{0.0};
  double straight_yaw_kd_{0.05};
  double straight_yaw_integral_limit_{0.5};
  double straight_yaw_max_angular_velocity_{0.5};
  double straight_yaw_integral_{0.0};
  double previous_straight_yaw_error_{0.0};
  std::chrono::steady_clock::time_point last_control_time_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionPlanExecutor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
