#include <cmath>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace
{

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
    cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "/cmd_vel");

    if (plan_file_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "Parameter 'plan_file' is required.");
      throw std::runtime_error("Missing required parameter: plan_file");
    }

    load_plan(plan_file_);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_,
      rclcpp::QoS(10),
      std::bind(&MotionPlanExecutor::pose_callback, this, std::placeholders::_1));

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
    CommandType type;
    double target;
    double speed;
  };

  void load_plan(const std::string & yaml_path)
  {
    if (!std::filesystem::exists(yaml_path)) {
      throw std::runtime_error("Plan file does not exist: " + yaml_path);
    }

    YAML::Node root = YAML::LoadFile(yaml_path);
    YAML::Node motions = root;

    if (root["motions"]) {
      motions = root["motions"];
    }

    if (!motions.IsSequence()) {
      throw std::runtime_error(
              "YAML format error: expected a sequence at root or under 'motions'.");
    }

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
        if (!entry["distance"] || !entry["linear_velocity"]) {
          throw std::runtime_error(
                  "STRAIGHT entry requires 'distance' and 'linear_velocity'.");
        }
        commands_.push_back(
          Command{CommandType::STRAIGHT, entry["distance"].as<double>(),
            entry["linear_velocity"].as<double>()});
      } else if (type == "TURN") {
        if (!entry["angle"] || !entry["angular_velocity"]) {
          throw std::runtime_error(
                  "TURN entry requires 'angle' and 'angular_velocity'.");
        }
        commands_.push_back(
          Command{CommandType::TURN, entry["angle"].as<double>(),
            entry["angular_velocity"].as<double>()});
      } else {
        throw std::runtime_error("Unsupported command type: " + type);
      }
    }

    if (commands_.empty()) {
      throw std::runtime_error("Plan contains no commands.");
    }
  }

  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_pose_ = *msg;
    has_pose_ = true;
  }

  void control_loop()
  {
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
      start_x_ = last_pose_.pose.position.x;
      start_y_ = last_pose_.pose.position.y;
      start_yaw_ = yaw_from_quaternion(last_pose_.pose.orientation);

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
      const double dx = last_pose_.pose.position.x - start_x_;
      const double dy = last_pose_.pose.position.y - start_y_;
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
    } else {
      const double current_yaw = yaw_from_quaternion(last_pose_.pose.orientation);
      const double delta_yaw = normalize_angle(current_yaw - start_yaw_);
      const double turned = std::abs(delta_yaw);
      const double target_angle = std::abs(cmd.target);

      if (turned >= target_angle) {
        advance_command();
        publish_stop();
        return;
      }

      const double speed = std::copysign(std::abs(cmd.speed), cmd.target);
      twist.linear.x = 0.0;
      twist.angular.z = speed;
    }

    cmd_pub_->publish(twist);
  }

  void advance_command()
  {
    ++current_index_;
    segment_started_ = false;
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

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped last_pose_;
  bool has_pose_{false};

  std::size_t current_index_{0};
  bool segment_started_{false};
  bool done_logged_{false};

  double start_x_{0.0};
  double start_y_{0.0};
  double start_yaw_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionPlanExecutor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
