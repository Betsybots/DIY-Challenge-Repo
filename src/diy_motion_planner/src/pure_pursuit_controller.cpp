// Pure-pursuit controller plugin implementation -- see the header for the
// architectural context (what changed vs. the original standalone node, and
// the gap around who actually invokes this).

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include "diy_motion_planner/pure_pursuit_controller.hpp"

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace diy_motion_planner
{

void PurePursuitController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = node_.lock();

  costmap_ros_ = costmap_ros;
  tf_ = tf;
  plugin_name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_distance", rclcpp::ParameterValue(0.2));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".linear_velocity", rclcpp::ParameterValue(0.3));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_angular_velocity", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_in_place_threshold", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".minimum_turning_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));

  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".linear_velocity", linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + ".rotate_in_place_threshold", rotate_in_place_threshold_);
  node->get_parameter(plugin_name_ + ".minimum_turning_velocity", minimum_turning_velocity_);
  node->get_parameter(plugin_name_ + ".goal_tolerance", goal_tolerance_);

  double transform_tolerance;
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
  transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);

  // Side-channel publishers -- same topic names as the original standalone
  // node, so diy_waypoint_sequencer and existing RViz configs need no changes.
  goal_reached_pub_ = node->create_publisher<std_msgs::msg::Bool>(
    "/pd/goal_reached", rclcpp::QoS(10));
  next_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/pd/next_pose", rclcpp::QoS(10));
  lookahead_marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
    "/pd/lookahead_marker", rclcpp::QoS(10));

  RCLCPP_INFO(
    logger_, "Configured pure pursuit controller plugin: %s", plugin_name_.c_str());
}

void PurePursuitController::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up controller: %s of type diy_motion_planner::PurePursuitController",
    plugin_name_.c_str());
  goal_reached_pub_.reset();
  next_pose_pub_.reset();
  lookahead_marker_pub_.reset();
}

void PurePursuitController::activate()
{
  RCLCPP_INFO(
    logger_, "Activating controller: %s of type diy_motion_planner::PurePursuitController",
    plugin_name_.c_str());
  goal_reached_pub_->on_activate();
  next_pose_pub_->on_activate();
  lookahead_marker_pub_->on_activate();
}

void PurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating controller: %s of type diy_motion_planner::PurePursuitController",
    plugin_name_.c_str());
  goal_reached_pub_->on_deactivate();
  next_pose_pub_->on_deactivate();
  lookahead_marker_pub_->on_deactivate();
}

void PurePursuitController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  // Not implemented -- the original standalone node never supported dynamic
  // speed limiting either (no /speed_limit subscriber). Left as a no-op
  // rather than silently misinterpreting percentage vs. absolute m/s.
  (void)speed_limit;
  (void)percentage;
}

void PurePursuitController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  RCLCPP_INFO(
    logger_, "Received path with %zu poses in frame \"%s\"",
    path.poses.size(), path.header.frame_id.c_str());
}

nav_msgs::msg::Path PurePursuitController::transformGlobalPlan(const std::string & target_frame)
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received plan with zero length");
  }

  if (global_plan_.header.frame_id == target_frame) {
    return global_plan_;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_->lookupTransform(
      target_frame, global_plan_.header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    throw nav2_core::PlannerException(
      "Unable to transform global plan into frame \"" + target_frame + "\": " + ex.what());
  }

  nav_msgs::msg::Path transformed_plan;
  transformed_plan.header.frame_id = target_frame;
  transformed_plan.header.stamp = global_plan_.header.stamp;
  transformed_plan.poses.reserve(global_plan_.poses.size());

  for (const auto & pose_in : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped pose_out;
    tf2::doTransform(pose_in, pose_out, transform);
    pose_out.header.frame_id = target_frame;
    transformed_plan.poses.push_back(pose_out);
  }

  return transformed_plan;
}

geometry_msgs::msg::PoseStamped PurePursuitController::selectLookaheadPose(
  const nav_msgs::msg::Path & transformed_plan, double robot_x, double robot_y) const
{
  const auto & poses = transformed_plan.poses;

  std::size_t closest_index = 0;
  double minimum_distance_squared = std::numeric_limits<double>::infinity();

  for (std::size_t index = 0; index < poses.size(); ++index) {
    const double dx = poses[index].pose.position.x - robot_x;
    const double dy = poses[index].pose.position.y - robot_y;
    const double distance_squared = dx * dx + dy * dy;

    if (distance_squared < minimum_distance_squared) {
      minimum_distance_squared = distance_squared;
      closest_index = index;
    }
  }

  geometry_msgs::msg::PoseStamped target = poses.back();

  for (std::size_t index = closest_index; index < poses.size(); ++index) {
    if (std::hypot(
        poses[index].pose.position.x - robot_x,
        poses[index].pose.position.y - robot_y) >= lookahead_distance_)
    {
      target = poses[index];
      break;
    }
  }

  return target;
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

geometry_msgs::msg::TwistStamped PurePursuitController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  // Goal-reached detection is self-managed (publishing /pd/goal_reached)
  // rather than delegated to goal_checker, preserving the original node's
  // exact behavior for diy_waypoint_sequencer's compatibility -- see the
  // header comment for why nothing calls this plugin's action yet regardless.
  (void)velocity;
  (void)goal_checker;

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();

  const nav_msgs::msg::Path transformed_plan = transformGlobalPlan(pose.header.frame_id);

  const double robot_x = pose.pose.position.x;
  const double robot_y = pose.pose.position.y;

  const auto & final_pose = transformed_plan.poses.back();
  if (std::hypot(
      final_pose.pose.position.x - robot_x,
      final_pose.pose.position.y - robot_y) <= goal_tolerance_)
  {
    std_msgs::msg::Bool reached;
    reached.data = true;
    goal_reached_pub_->publish(reached);
    // cmd_vel already zero-initialized -- stop the robot.
    return cmd_vel;
  }

  const geometry_msgs::msg::PoseStamped target = selectLookaheadPose(
    transformed_plan, robot_x, robot_y);
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
  const double heading_error = std::atan2(
    std::sin(target_heading - robot_yaw), std::cos(target_heading - robot_yaw));

  if (std::abs(heading_error) > rotate_in_place_threshold_) {
    cmd_vel.twist.angular.z = std::copysign(max_angular_velocity_, heading_error);
    return cmd_vel;
  }

  const double curvature = 2.0 * std::sin(heading_error) / std::max(distance, 1e-6);
  const double turning_scale = std::max(
    minimum_turning_velocity_ / linear_velocity_,
    1.0 - std::abs(heading_error) / M_PI);
  const double linear_velocity = linear_velocity_ * turning_scale;

  const double angular_velocity = std::clamp(
    linear_velocity * curvature, -max_angular_velocity_, max_angular_velocity_);

  cmd_vel.twist.linear.x = linear_velocity;
  cmd_vel.twist.angular.z = angular_velocity;

  return cmd_vel;
}

}  // namespace diy_motion_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diy_motion_planner::PurePursuitController, nav2_core::Controller)
