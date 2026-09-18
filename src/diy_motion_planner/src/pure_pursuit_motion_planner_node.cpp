// Pure-pursuit path follower for the DIY Challenge robot (C++ port of
// motion_planner/pure_pursuit_motion_planner_node.py).

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class PurePursuitMotionPlanner : public rclcpp::Node
{
public:
  PurePursuitMotionPlanner()
  : Node("pure_pursuit_motion_planner_node")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    path_topic_ = declare_parameter("path_topic", std::string("/a_star/path"));
    cmd_vel_topic_ = declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    lookahead_distance_ = declare_parameter("lookahead_distance", 0.2);
    linear_velocity_ = declare_parameter("linear_velocity", 0.3);
    max_angular_velocity_ = declare_parameter("max_angular_velocity", 1.0);
    minimum_angular_velocity_ = declare_parameter("minimum_angular_velocity", 0.12);
    rotate_in_place_threshold_ = declare_parameter("rotate_in_place_threshold", 1.0);
    minimum_turning_velocity_ = declare_parameter("minimum_turning_velocity", 0.05);
    goal_tolerance_ = declare_parameter("goal_tolerance", 0.15);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10,
      std::bind(&PurePursuitMotionPlanner::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    next_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);
    lookahead_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/pd/lookahead_marker", 10);
    // See the PD controller's matching publisher for why this exists — lets
    // an external waypoint sequencer react to "goal reached" without polling.
    goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>("/pd/goal_reached", 10);

    timer_ = create_wall_timer(
      100ms, std::bind(&PurePursuitMotionPlanner::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Pure pursuit ready: %s -> %s",
      path_topic_.c_str(), cmd_vel_topic_.c_str());
  }

  void stopRobot()
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    global_plan_ = path;
    RCLCPP_INFO(get_logger(), "Received path with %zu poses in frame \"%s\"",
      path->poses.size(), path->header.frame_id.c_str());
  }

  void controlLoop()
  {
    if (!global_plan_ || global_plan_->poses.empty()) {
      return;
    }

    geometry_msgs::msg::TransformStamped odom_to_base;
    geometry_msgs::msg::TransformStamped odom_to_path;
    try {
      odom_to_base = tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero);
      odom_to_path = tf_buffer_->lookupTransform(
        odom_frame_, global_plan_->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN(get_logger(), "Could not transform path or robot: %s", exception.what());
      return;
    }

    const double robot_x = odom_to_base.transform.translation.x;
    const double robot_y = odom_to_base.transform.translation.y;

    const geometry_msgs::msg::PoseStamped final_pose = transformPose(
      global_plan_->poses.back(), odom_to_path);

    if (std::hypot(final_pose.pose.position.x - robot_x,
      final_pose.pose.position.y - robot_y) <= goal_tolerance_)
    {
      std_msgs::msg::Bool reached;
      reached.data = true;
      goal_reached_pub_->publish(reached);
      stopRobot();
      global_plan_.reset();
      return;
    }

    const geometry_msgs::msg::PoseStamped target = selectLookaheadPose(
      robot_x, robot_y, odom_to_path);
    next_pose_pub_->publish(target);
    publishLookaheadMarker(target);

    const auto & rotation = odom_to_base.transform.rotation;
    const double robot_yaw = std::atan2(
      2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));

    const double delta_x = target.pose.position.x - robot_x;
    const double delta_y = target.pose.position.y - robot_y;
    const double distance = std::hypot(delta_x, delta_y);
    const double target_heading = std::atan2(delta_y, delta_x);
    const double heading_error = normalizeAngle(target_heading - robot_yaw);

    if (std::abs(heading_error) > rotate_in_place_threshold_) {
      geometry_msgs::msg::Twist command;
      command.angular.z = std::copysign(max_angular_velocity_, heading_error);
      cmd_pub_->publish(command);
      return;
    }

    const double curvature = 2.0 * std::sin(heading_error) / std::max(distance, 1e-6);
    const double turning_scale = std::max(
      minimum_turning_velocity_ / linear_velocity_,
      1.0 - std::abs(heading_error) / M_PI);
    const double linear_velocity = linear_velocity_ * turning_scale;

    double angular_velocity = std::clamp(
      linear_velocity * curvature, -max_angular_velocity_, max_angular_velocity_);

    const double effective_minimum_angular_velocity = std::clamp(
      minimum_angular_velocity_, 0.0, max_angular_velocity_);

    if (std::abs(angular_velocity) > 1e-6 &&
      std::abs(angular_velocity) < effective_minimum_angular_velocity)
    {
      angular_velocity = std::copysign(effective_minimum_angular_velocity, angular_velocity);
    }

    geometry_msgs::msg::Twist command;
    command.linear.x = linear_velocity;
    command.angular.z = angular_velocity;
    cmd_pub_->publish(command);
  }

  geometry_msgs::msg::PoseStamped selectLookaheadPose(
    double robot_x, double robot_y, const geometry_msgs::msg::TransformStamped & transform)
  {
    const auto & poses = global_plan_->poses;

    std::size_t closest_index = 0;
    double minimum_distance_squared = std::numeric_limits<double>::infinity();

    for (std::size_t index = 0; index < poses.size(); ++index) {
      const geometry_msgs::msg::PoseStamped transformed = transformPose(poses[index], transform);
      const double dx = transformed.pose.position.x - robot_x;
      const double dy = transformed.pose.position.y - robot_y;
      const double distance_squared = dx * dx + dy * dy;

      if (distance_squared < minimum_distance_squared) {
        minimum_distance_squared = distance_squared;
        closest_index = index;
      }
    }

    geometry_msgs::msg::PoseStamped target = transformPose(poses.back(), transform);

    for (std::size_t index = closest_index; index < poses.size(); ++index) {
      const geometry_msgs::msg::PoseStamped transformed = transformPose(poses[index], transform);

      if (std::hypot(transformed.pose.position.x - robot_x,
        transformed.pose.position.y - robot_y) >= lookahead_distance_)
      {
        target = transformed;
        break;
      }
    }

    return target;
  }

  static double normalizeAngle(double angle)
  {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  // Rotates/translates a path pose into the given transform's frame using a
  // 2D (yaw-only) composition — the pose's own orientation field is left
  // untouched, matching the original Python controller's behavior.
  static geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::TransformStamped & transform)
  {
    const auto & rotation = transform.transform.rotation;
    const double yaw = std::atan2(
      2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));

    geometry_msgs::msg::PoseStamped transformed;
    transformed.header.frame_id = transform.header.frame_id;
    transformed.header.stamp = pose.header.stamp;
    transformed.pose = pose.pose;

    transformed.pose.position.x = transform.transform.translation.x
      + std::cos(yaw) * pose.pose.position.x
      - std::sin(yaw) * pose.pose.position.y;
    transformed.pose.position.y = transform.transform.translation.y
      + std::sin(yaw) * pose.pose.position.x
      + std::cos(yaw) * pose.pose.position.y;
    transformed.pose.position.z = transform.transform.translation.z + pose.pose.position.z;

    return transformed;
  }

  void publishLookaheadMarker(const geometry_msgs::msg::PoseStamped & pose)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = "lookahead";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = pose.pose.position;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    lookahead_marker_pub_->publish(marker);
  }

  // Parameters
  std::string path_topic_;
  std::string cmd_vel_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  double lookahead_distance_;
  double linear_velocity_;
  double max_angular_velocity_;
  double minimum_angular_velocity_;
  double rotate_in_place_threshold_;
  double minimum_turning_velocity_;
  double goal_tolerance_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::Path::SharedPtr global_plan_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PurePursuitMotionPlanner>();
  rclcpp::spin(node);
  node->stopRobot();
  rclcpp::shutdown();
  return 0;
}
