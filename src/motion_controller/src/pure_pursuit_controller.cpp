// Pure-pursuit controller plugin implementation -- see the header for the
// architectural context (what changed vs. the original standalone node, and
// the gap around who actually invokes this).

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "motion_controller/pure_pursuit_controller.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace motion_controller
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
    node, plugin_name_ + ".minimum_angular_velocity", rclcpp::ParameterValue(0.12));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".rotate_in_place_threshold", rclcpp::ParameterValue(1.0));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".minimum_turning_velocity", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.15));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_check_enabled", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".occupied_threshold", rclcpp::ParameterValue(253));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".unknown_is_occupied", rclcpp::ParameterValue(true));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_check_resolution", rclcpp::ParameterValue(0.05));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".collision_check_distance", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".unknown_grace_period", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_pose_jump_speed", rclcpp::ParameterValue(1.5));

  node->get_parameter(plugin_name_ + ".lookahead_distance", lookahead_distance_);
  node->get_parameter(plugin_name_ + ".linear_velocity", linear_velocity_);
  node->get_parameter(plugin_name_ + ".max_angular_velocity", max_angular_velocity_);
  node->get_parameter(plugin_name_ + ".minimum_angular_velocity", minimum_angular_velocity_);
  node->get_parameter(plugin_name_ + ".rotate_in_place_threshold", rotate_in_place_threshold_);
  node->get_parameter(plugin_name_ + ".minimum_turning_velocity", minimum_turning_velocity_);
  node->get_parameter(plugin_name_ + ".goal_tolerance", goal_tolerance_);
  node->get_parameter(plugin_name_ + ".collision_check_enabled", collision_check_enabled_);
  node->get_parameter(plugin_name_ + ".occupied_threshold", occupied_threshold_);
  node->get_parameter(plugin_name_ + ".unknown_is_occupied", unknown_is_occupied_);
  node->get_parameter(plugin_name_ + ".collision_check_resolution", collision_check_resolution_);
  node->get_parameter(plugin_name_ + ".collision_check_distance", collision_check_distance_);
  node->get_parameter(plugin_name_ + ".unknown_grace_period", unknown_grace_period_);
  node->get_parameter(plugin_name_ + ".max_pose_jump_speed", max_pose_jump_speed_);

  if (collision_check_resolution_ <= 0.0) {
    collision_check_resolution_ = 0.05;
  }

  if (collision_check_distance_ <= 0.0) {
    collision_check_distance_ = 0.5;
  }

  if (unknown_grace_period_ < 0.0) {
    unknown_grace_period_ = 0.0;
  }

  if (max_pose_jump_speed_ <= 0.0) {
    max_pose_jump_speed_ = 1.5;
  }

  has_blocked_before_ = false;
  has_valid_transformed_plan_ = false;
  has_last_final_pose_ = false;

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
    logger_, "Cleaning up controller: %s of type motion_controller::PurePursuitController",
    plugin_name_.c_str());
  goal_reached_pub_.reset();
  next_pose_pub_.reset();
  lookahead_marker_pub_.reset();
}

void PurePursuitController::activate()
{
  RCLCPP_INFO(
    logger_, "Activating controller: %s of type motion_controller::PurePursuitController",
    plugin_name_.c_str());
  goal_reached_pub_->on_activate();
  next_pose_pub_->on_activate();
  lookahead_marker_pub_->on_activate();
}

void PurePursuitController::deactivate()
{
  RCLCPP_INFO(
    logger_, "Deactivating controller: %s of type motion_controller::PurePursuitController",
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

  // Transform into the local costmap's frame ONCE here, rather than on every
  // computeVelocityCommands() tick. Re-fetching the latest map->odom
  // transform every 20Hz tick meant any localization correction (a real,
  // observed failure mode -- map_localizer applying a non-smoothed absolute
  // correction) got imported into the control loop instantly, teleporting
  // final_pose. Caching it here means the robot smoothly tracks this fixed
  // path until the next replan, at which point a fresh correction is picked
  // up all at once (bounded by the planner's own replanning rate) instead of
  // continuously mid-flight.
  try {
    global_plan_odom_ = transformGlobalPlan(costmap_ros_->getGlobalFrameID());
    has_valid_transformed_plan_ = true;
  } catch (const nav2_core::PlannerException & ex) {
    if (!has_valid_transformed_plan_) {
      // No previously-cached plan to fall back on -- nothing safe to do but
      // propagate the failure so controller_server can report it.
      throw;
    }
    RCLCPP_WARN(
      logger_,
      "setPlan(): failed to transform the new plan into \"%s\" (%s) -- "
      "continuing to track the previous plan until this succeeds.",
      costmap_ros_->getGlobalFrameID().c_str(), ex.what());
  }
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

PurePursuitController::PathBlockStatus PurePursuitController::checkPathBlockStatus(
  double robot_x,
  double robot_y,
  const nav_msgs::msg::Path & transformed_plan) const
{
  if (!collision_check_enabled_ || !costmap_ros_) {
    return PathBlockStatus::CLEAR;
  }

  const auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    return PathBlockStatus::CLEAR;
  }

  double prev_x = robot_x;
  double prev_y = robot_y;
  double distance_walked = 0.0;
  bool saw_unknown = false;

  for (const auto & stamped_pose : transformed_plan.poses) {
    const double seg_dx = stamped_pose.pose.position.x - prev_x;
    const double seg_dy = stamped_pose.pose.position.y - prev_y;
    const double seg_length = std::hypot(seg_dx, seg_dy);

    if (seg_length > 1e-6) {
      const double remaining = collision_check_distance_ - distance_walked;
      const double check_length = std::min(seg_length, remaining);
      const int samples = std::max(
        1, static_cast<int>(std::ceil(check_length / collision_check_resolution_)));

      for (int sample = 1; sample <= samples; ++sample) {
        const double ratio = (static_cast<double>(sample) / samples) * (check_length / seg_length);
        const double x = prev_x + ratio * seg_dx;
        const double y = prev_y + ratio * seg_dy;

        unsigned int mx = 0;
        unsigned int my = 0;
        if (!costmap->worldToMap(x, y, mx, my)) {
          // Outside the costmap's current bounds -- same "we have no data
          // here" meaning as NO_INFORMATION, not an automatic hard block.
          saw_unknown = true;
          continue;
        }

        const unsigned char cost = costmap->getCost(mx, my);
        if (cost == nav2_costmap_2d::NO_INFORMATION) {
          saw_unknown = true;
        } else if (cost >= occupied_threshold_) {
          return PathBlockStatus::CONFIRMED_OBSTACLE;
        }
      }
    }

    distance_walked += seg_length;
    prev_x = stamped_pose.pose.position.x;
    prev_y = stamped_pose.pose.position.y;

    if (distance_walked >= collision_check_distance_) {
      break;
    }
  }

  if (saw_unknown && unknown_is_occupied_) {
    return PathBlockStatus::UNKNOWN_ONLY;
  }

  return PathBlockStatus::CLEAR;
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

  if (!has_valid_transformed_plan_ || global_plan_odom_.poses.empty()) {
    throw nav2_core::PlannerException(
            "computeVelocityCommands() called before a valid plan was cached by setPlan()");
  }
  const nav_msgs::msg::Path & transformed_plan = global_plan_odom_;

  const double robot_x = pose.pose.position.x;
  const double robot_y = pose.pose.position.y;

  const auto & final_pose = transformed_plan.poses.back();

  // Defensive backstop for the residual jump risk right at plan-swap time:
  // even with the plan now cached instead of re-transformed every tick (see
  // setPlan()), a NEW setPlan() call re-runs the transform with the
  // then-latest map->odom, so if a localization correction landed between
  // the previous and current replan, final_pose can still discontinuously
  // jump relative to the previous tick's. Detect a physically-implausible
  // jump (faster than max_pose_jump_speed_ would allow) and hold position
  // for this tick rather than steering off a corrupted-looking reference.
  // Deliberately self-clearing: if rejected, last_final_pose_/its timestamp
  // are NOT updated, so the same (now-older) baseline is compared again next
  // tick with a larger elapsed time -- same distance / growing time means
  // the implied speed keeps dropping, so a single large correction doesn't
  // permanently wedge the controller, it just holds for a few ticks.
  const rclcpp::Time now = clock_->now();
  if (has_last_final_pose_) {
    const double jump_dx = final_pose.pose.position.x - last_final_pose_.pose.position.x;
    const double jump_dy = final_pose.pose.position.y - last_final_pose_.pose.position.y;
    const double jump_distance = std::hypot(jump_dx, jump_dy);
    const double elapsed = std::max((now - last_final_pose_time_).seconds(), 1e-3);
    const double implied_speed = jump_distance / elapsed;
    if (implied_speed > max_pose_jump_speed_) {
      RCLCPP_WARN(
        logger_,
        "Rejecting implausible plan-reference jump: %.2fm in %.3fs (%.2f m/s > "
        "max_pose_jump_speed=%.2f m/s) -- likely a map->odom localization "
        "correction; holding position for this tick.",
        jump_distance, elapsed, implied_speed, max_pose_jump_speed_);
      return cmd_vel;  // already zero-initialized -- hold, don't act on it.
    }
  }
  last_final_pose_ = final_pose;
  last_final_pose_time_ = now;
  has_last_final_pose_ = true;

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

  const PathBlockStatus block_status = checkPathBlockStatus(robot_x, robot_y, transformed_plan);
  bool blocked = false;

  if (block_status == PathBlockStatus::CONFIRMED_OBSTACLE) {
    blocked = true;
  } else if (block_status == PathBlockStatus::UNKNOWN_ONLY) {
    // Right after a ClearEntireCostmap recovery, cells ahead sit at
    // NO_INFORMATION until the next sensor scan repopulates them. Without
    // this grace window, the very next tick would instantly re-block on
    // "unknown" alone and burn the recovery retry before the obstacle could
    // ever be re-observed. Confirmed obstacles are never granted grace.
    const bool within_grace = has_blocked_before_ &&
      (now - last_blocked_time_).seconds() < unknown_grace_period_;
    blocked = !within_grace;
  }

  if (blocked) {
    has_blocked_before_ = true;
    last_blocked_time_ = now;
    throw nav2_core::PlannerException(
            "Pure pursuit lookahead segment is blocked in the local costmap");
  }

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

  double angular_velocity = std::clamp(
    linear_velocity * curvature, -max_angular_velocity_, max_angular_velocity_);

  const double effective_minimum_angular_velocity = std::clamp(
    minimum_angular_velocity_, 0.0, max_angular_velocity_);

  if (std::abs(angular_velocity) > 1e-6 &&
    std::abs(angular_velocity) < effective_minimum_angular_velocity)
  {
    angular_velocity = std::copysign(effective_minimum_angular_velocity, angular_velocity);
  }

  cmd_vel.twist.linear.x = linear_velocity;
  cmd_vel.twist.angular.z = angular_velocity;

  return cmd_vel;
}

}  // namespace motion_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(motion_controller::PurePursuitController, nav2_core::Controller)
