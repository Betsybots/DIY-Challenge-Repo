// Pure-pursuit path follower for the DIY Challenge robot.
//
// Nav2 Controller plugin: loaded by nav2_controller's controller_server via
// pluginlib. Receives the global plan through setPlan() and is polled for
// velocity commands through computeVelocityCommands() (no /a_star/path
// subscription or /cmd_vel publisher of its own).

#ifndef DIY_MOTION_PLANNER__PURE_PURSUIT_MOTION_PLANNER_NODE_HPP_
#define DIY_MOTION_PLANNER__PURE_PURSUIT_MOTION_PLANNER_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker.hpp"

namespace diy_motion_planner
{

class PurePursuitController : public nav2_core::Controller
{
public:
  PurePursuitController() = default;
  ~PurePursuitController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

private:
  geometry_msgs::msg::PoseStamped selectLookaheadPose(
    double robot_x, double robot_y, const geometry_msgs::msg::TransformStamped & transform);

  static double normalizeAngle(double angle);

  static geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::TransformStamped & transform);

  void publishLookaheadMarker(const geometry_msgs::msg::PoseStamped & pose);

  // Nav2 plugin context
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("PurePursuitController")};
  rclcpp::Clock::SharedPtr clock_;

  // Parameters (declared under the plugin name, e.g. "FollowPath.lookahead_distance")
  double lookahead_distance_ = 0.2;
  double linear_velocity_ = 0.3;
  double max_angular_velocity_ = 1.0;
  double rotate_in_place_threshold_ = 1.0;
  double minimum_turning_velocity_ = 0.05;
  double goal_tolerance_ = 0.15;

  // setSpeedLimit() state
  double speed_limit_ = -1.0;
  bool speed_limit_is_percentage_ = false;

  // Debug publishers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr
    lookahead_marker_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;

  nav_msgs::msg::Path global_plan_;
};

}  // namespace diy_motion_planner

#endif  // DIY_MOTION_PLANNER__PURE_PURSUIT_MOTION_PLANNER_NODE_HPP_
