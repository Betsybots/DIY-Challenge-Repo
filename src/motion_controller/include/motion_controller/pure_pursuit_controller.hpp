// Pure-pursuit controller, converted from a standalone rclcpp::Node
// (pure_pursuit_motion_planner_node.cpp) into a real nav2_core::Controller
// plugin, loadable by controller_server as FollowPath's plugin.
//
// GAP TO BE AWARE OF: this plugin's setPlan()/computeVelocityCommands() are
// only ever invoked by controller_server in response to a FollowPath action
// goal. Nothing in this repo currently sends that action (no bt_navigator is
// launched) -- diy_planning's a_star_planner_node and this controller
// previously talked to each other directly over /a_star/path + cmd_vel_nav
// topics, with no action server involved at all. Until something calls
// controller_server's follow_path action (e.g. a small bridge node
// subscribing to /a_star/path and invoking the action, or reintroducing
// bt_navigator), this plugin will load and configure correctly but never
// actually run.
//
// Preserves the exact control law from the original node: closest-then-first
// -beyond-lookahead point selection, rotate-in-place above a heading-error
// threshold, curvature-based steering below it, and the same /pd/goal_reached,
// /pd/next_pose, /pd/lookahead_marker side-channel topics diy_waypoint_sequencer
// and RViz debugging already depend on -- those are NOT part of the
// nav2_core::Controller interface, so they're published from auxiliary
// lifecycle publishers created in configure()/activated in activate().

#ifndef MOTION_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
#define MOTION_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/buffer.h"

namespace motion_controller
{

class PurePursuitController : public nav2_core::Controller
{
public:
  PurePursuitController() = default;
  ~PurePursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

protected:
  // Transforms global_plan_ into target_frame using a single TF lookup (the
  // whole plan shares one rigid transform at a given instant) -- matches the
  // original node's one-lookup-per-tick design rather than transforming each
  // pose independently.
  nav_msgs::msg::Path transformGlobalPlan(const std::string & target_frame);

  geometry_msgs::msg::PoseStamped selectLookaheadPose(
    const nav_msgs::msg::Path & transformed_plan,
    double robot_x, double robot_y) const;

  bool isPathToTargetBlocked(
    double robot_x,
    double robot_y,
    const geometry_msgs::msg::PoseStamped & target) const;

  void publishLookaheadMarker(const geometry_msgs::msg::PoseStamped & pose);

  enum class PathBlockStatus { CLEAR, UNKNOWN_ONLY, CONFIRMED_OBSTACLE };

  PathBlockStatus checkPathBlockStatus(
    double robot_x,
    double robot_y,
    const nav_msgs::msg::Path & transformed_plan) const;

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  // Parameters -- same names/defaults as the original standalone node.
  double lookahead_distance_;
  double linear_velocity_;
  double max_angular_velocity_;
  double minimum_angular_velocity_;
  double rotate_in_place_threshold_;
  double minimum_turning_velocity_;
  double goal_tolerance_;
  bool collision_check_enabled_;
  bool unknown_is_occupied_;
  int occupied_threshold_;
  double collision_check_resolution_;
  double collision_check_distance_;
  double unknown_grace_period_;
  double max_pose_jump_speed_;
  rclcpp::Duration transform_tolerance_{0, 0};

  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path global_plan_odom_;
  bool has_valid_transformed_plan_ = false;

  bool has_blocked_before_ = false;
  rclcpp::Time last_blocked_time_;

  bool has_last_final_pose_ = false;
  geometry_msgs::msg::PoseStamped last_final_pose_;
  rclcpp::Time last_final_pose_time_;

  // Side-channel publishers -- not part of nav2_core::Controller, kept for
  // compatibility with diy_waypoint_sequencer (/pd/goal_reached) and existing
  // RViz debug displays (/pd/next_pose, /pd/lookahead_marker).
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>> goal_reached_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>>
    next_pose_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>>
    lookahead_marker_pub_;
};

}  // namespace motion_controller

#endif  // MOTION_CONTROLLER__PURE_PURSUIT_CONTROLLER_HPP_
