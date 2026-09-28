#ifndef WALL_FOLLOWER_CONTROLLER__WALL_FOLLOWER_CONTROLLER_HPP_
#define WALL_FOLLOWER_CONTROLLER__WALL_FOLLOWER_CONTROLLER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"

namespace wall_follower_controller
{

class WallFollowerController : public nav2_core::Controller
{
public:
  WallFollowerController() = default;
  ~WallFollowerController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

private:
  struct Point2D
  {
    double x;
    double y;
  };

  struct LineFit
  {
    bool valid{false};
    double slope{0.0};
    double intercept{0.0};
    std::size_t inliers{0};
  };

  struct WallEstimate
  {
    bool valid{false};
    double center_offset{0.0};
    double heading_error{0.0};
    double corridor_width{0.0};
    double center_slope{0.0};
  };

  struct LocalPathPoint
  {
    double x{0.0};
    double y{0.0};
    double arc_distance{0.0};
  };

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud);
  nav_msgs::msg::Path transformPlan(const std::string & target_frame) const;
  std::vector<LocalPathPoint> buildLocalPath(
    const geometry_msgs::msg::PoseStamped & pose,
    double preview_distance) const;
  static double pointCurvature(const LocalPathPoint & point);
  WallEstimate estimateWalls(
    const sensor_msgs::msg::PointCloud2 & cloud) const;
  LineFit fitWallLine(const std::vector<Point2D> & points) const;
  double calculateTargetSpeed(
    const std::vector<LocalPathPoint> & local_path,
    double current_curvature) const;
  bool commandIsCollisionFree(
    const geometry_msgs::msg::PoseStamped & pose,
    double linear_velocity,
    double angular_velocity) const;
  void publishWallCenterline(const WallEstimate & walls);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;
  rclcpp::Logger logger_{rclcpp::get_logger("WallFollowerController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  std::shared_ptr<
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>>
  centerline_pub_;

  mutable std::mutex cloud_mutex_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

  nav_msgs::msg::Path global_plan_;
  nav_msgs::msg::Path transformed_plan_;
  bool has_valid_plan_{false};

  std::string pointcloud_topic_;
  std::string base_frame_;
  double transform_tolerance_;
  double cloud_timeout_;
  int point_stride_;

  double wall_min_x_;
  double wall_max_x_;
  double wall_min_abs_y_;
  double wall_max_abs_y_;
  double wall_min_z_;
  double wall_max_z_;
  int minimum_wall_points_;
  double wall_fit_residual_;
  double minimum_corridor_width_;
  double maximum_corridor_width_;

  double lateral_error_gain_;
  double heading_error_gain_;
  double maximum_wall_correction_;
  bool stop_on_wall_loss_;
  double wall_loss_speed_;

  double desired_linear_velocity_;
  double minimum_corner_velocity_;
  double maximum_angular_velocity_;
  double minimum_lookahead_distance_;
  double maximum_lookahead_distance_;
  double lookahead_time_;
  double curvature_scaling_min_radius_;
  double speed_preview_distance_;
  double maximum_lateral_acceleration_;
  double maximum_deceleration_;
  double wheelbase_;
  double maximum_steering_angle_;
  double goal_tolerance_;

  bool collision_check_enabled_;
  double collision_time_horizon_;
  double collision_step_time_;
  bool unknown_is_occupied_;

  std::atomic<double> external_speed_limit_{0.0};
};

}  // namespace wall_follower_controller

#endif  // WALL_FOLLOWER_CONTROLLER__WALL_FOLLOWER_CONTROLLER_HPP_
