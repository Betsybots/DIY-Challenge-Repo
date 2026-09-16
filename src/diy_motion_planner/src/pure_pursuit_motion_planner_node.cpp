// Pure-pursuit path follower for the DIY Challenge robot.
//
// Nav2 Controller plugin implementation. See pure_pursuit_motion_planner_node.hpp.

#include "diy_motion_planner/pure_pursuit_motion_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "tf2/exceptions.h"

namespace diy_motion_planner
{

void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lookahead_distance", rclcpp::ParameterValue(lookahead_distance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".linear_velocity", rclcpp::ParameterValue(linear_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_angular_velocity", rclcpp::ParameterValue(max_angular_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rotate_in_place_threshold",
    rclcpp::ParameterValue(rotate_in_place_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".minimum_turning_velocity", rclcpp::ParameterValue(minimum_turning_velocity_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_tolerance", rclcpp::ParameterValue(goal_tolerance_));

  node->get_parameter(name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(name_ + ".linear_velocity", linear_velocity_);
  node->get_parameter(name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(name_ + ".rotate_in_place_threshold", rotate_in_place_threshold_);
  node->get_parameter(name_ + ".minimum_turning_velocity", minimum_turning_velocity_);
  node->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);

  next_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    name_ + "/next_pose", 10);
  lookahead_marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    name_ + "/lookahead_marker", 10);
  // See the PD controller's matching publisher for why this exists — lets
  // an external waypoint sequencer react to "goal reached" without polling.
  goal_reached_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    name_ + "/goal_reached", 10);

  RCLCPP_INFO(logger_, "Configured pure pursuit controller \"%s\"", name_.c_str());
}

void PurePursuitController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up pure pursuit controller \"%s\"", name_.c_str());
  next_pose_pub_.reset();
  lookahead_marker_pub_.reset();
  goal_reached_pub_.reset();
}

void PurePursuitController::activate()
{
  RCLCPP_INFO(logger_, "Activating pure pursuit controller \"%s\"", name_.c_str());
  next_pose_pub_->on_activate();
  lookahead_marker_pub_->on_activate();
  goal_reached_pub_->on_activate();
}

void PurePursuitController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating pure pursuit controller \"%s\"", name_.c_str());
  next_pose_pub_->on_deactivate();
  lookahead_marker_pub_->on_deactivate();
  goal_reached_pub_->on_deactivate();
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_INFO(
    logger_, "Received path with %zu poses in frame \"%s\"",
    path.poses.size(), path.header.frame_id.c_str());
}

void PurePursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  speed_limit_ = speed_limit;
  speed_limit_is_percentage_ = percentage;
}

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = costmap_ros_->getBaseFrameID();
  command.header.stamp = clock_->now();

  if (global_plan_.poses.empty()) {
    return command;
  }

  geometry_msgs::msg::TransformStamped path_to_pose_frame;
  try {
    path_to_pose_frame = tf_->lookupTransform(
      pose.header.frame_id, global_plan_.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN(logger_, "Could not transform path into robot frame: %s", exception.what());
    return command;
  }

  const double robot_x = pose.pose.position.x;
  const double robot_y = pose.pose.position.y;

  const geometry_msgs::msg::PoseStamped final_pose = transformPose(
    global_plan_.poses.back(), path_to_pose_frame);

  if (std::hypot(final_pose.pose.position.x - robot_x,
    final_pose.pose.position.y - robot_y) <= goal_tolerance_)
  {
    std_msgs::msg::Bool reached;
    reached.data = true;
    goal_reached_pub_->publish(reached);
    return command;
  }

  const geometry_msgs::msg::PoseStamped target = selectLookaheadPose(
    robot_x, robot_y, path_to_pose_frame);
  next_pose_pub_->publish(target);
  publishLookaheadMarker(target);

  const auto & rotation = pose.pose.orientation;
  const double robot_yaw = std::atan2(
    2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
    1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));

  const double delta_x = target.pose.position.x - robot_x;
  const double delta_y = target.pose.position.y - robot_y;
  const double distance = std::hypot(delta_x, delta_y);
  const double target_heading = std::atan2(delta_y, delta_x);
  const double heading_error = normalizeAngle(target_heading - robot_yaw);

  if (std::abs(heading_error) > rotate_in_place_threshold_) {
    command.twist.angular.z = std::copysign(max_angular_velocity_, heading_error);
    return command;
  }

  double linear_velocity_limit = linear_velocity_;
  if (speed_limit_ >= 0.0) {
    linear_velocity_limit = speed_limit_is_percentage_
      ? linear_velocity_ * speed_limit_ / 100.0
      : std::min(linear_velocity_, speed_limit_);
  }

  const double curvature = 2.0 * std::sin(heading_error) / std::max(distance, 1e-6);
  const double turning_scale = std::max(
    minimum_turning_velocity_ / linear_velocity_,
    1.0 - std::abs(heading_error) / M_PI);
  const double linear_velocity = linear_velocity_limit * turning_scale;

  const double angular_velocity = std::clamp(
    linear_velocity * curvature, -max_angular_velocity_, max_angular_velocity_);

  command.twist.linear.x = linear_velocity;
  command.twist.angular.z = angular_velocity;
  return command;
}

geometry_msgs::msg::PoseStamped PurePursuitController::selectLookaheadPose(
  double robot_x, double robot_y, const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & poses = global_plan_.poses;

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

double PurePursuitController::normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Rotates/translates a path pose into the given transform's frame using a
// 2D (yaw-only) composition — the pose's own orientation field is left
// untouched, matching the original controller's behavior.
geometry_msgs::msg::PoseStamped PurePursuitController::transformPose(
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

void PurePursuitController::publishLookaheadMarker(const geometry_msgs::msg::PoseStamped & pose)
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

}  // namespace diy_motion_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diy_motion_planner::PurePursuitController, nav2_core::Controller)
