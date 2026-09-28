#include "wall_follower_controller/wall_follower_controller.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace wall_follower_controller
{

using nav2_util::declare_parameter_if_not_declared;

namespace
{

double clampMagnitude(double value, double maximum)
{
  return std::clamp(value, -maximum, maximum);
}

}  // namespace

void WallFollowerController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  const auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock lifecycle node");
  }

  plugin_name_ = std::move(name);
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  const auto declare = [&](const std::string & parameter, const auto & value) {
      declare_parameter_if_not_declared(
        node, plugin_name_ + "." + parameter, rclcpp::ParameterValue(value));
    };

  declare("pointcloud_topic", std::string("/cloud_registered_body"));
  declare("base_frame", std::string("base_footprint"));
  declare("transform_tolerance", 0.1);
  declare("cloud_timeout", 0.3);
  declare("point_stride", 5);

  declare("wall_min_x", -0.1);
  declare("wall_max_x", 2.5);
  declare("wall_min_abs_y", 0.15);
  declare("wall_max_abs_y", 2.0);
  declare("wall_min_z", 0.10);
  declare("wall_max_z", 1.50);
  declare("minimum_wall_points", 30);
  declare("wall_fit_residual", 0.08);
  declare("minimum_corridor_width", 0.40);
  declare("maximum_corridor_width", 3.00);

  declare("lateral_error_gain", 1.5);
  declare("heading_error_gain", 1.0);
  declare("maximum_wall_correction", 0.75);
  declare("stop_on_wall_loss", true);
  declare("wall_loss_speed", 0.20);

  declare("desired_linear_velocity", 0.50);
  declare("minimum_corner_velocity", 0.25);
  declare("maximum_angular_velocity", 2.0);
  declare("minimum_lookahead_distance", 0.25);
  declare("maximum_lookahead_distance", 1.00);
  declare("lookahead_time", 1.0);
  declare("curvature_scaling_min_radius", 1.50);
  declare("speed_preview_distance", 6.0);
  declare("maximum_lateral_acceleration", 1.5);
  declare("maximum_deceleration", 0.8);
  declare("wheelbase", 0.31115);
  declare("maximum_steering_angle", 0.6981317);
  declare("goal_tolerance", 0.20);

  declare("collision_check_enabled", true);
  declare("collision_time_horizon", 1.0);
  declare("collision_step_time", 0.05);
  declare("unknown_is_occupied", true);

  const auto get = [&](const std::string & parameter, auto & value) {
      node->get_parameter(plugin_name_ + "." + parameter, value);
    };

  get("pointcloud_topic", pointcloud_topic_);
  get("base_frame", base_frame_);
  get("transform_tolerance", transform_tolerance_);
  get("cloud_timeout", cloud_timeout_);
  get("point_stride", point_stride_);

  get("wall_min_x", wall_min_x_);
  get("wall_max_x", wall_max_x_);
  get("wall_min_abs_y", wall_min_abs_y_);
  get("wall_max_abs_y", wall_max_abs_y_);
  get("wall_min_z", wall_min_z_);
  get("wall_max_z", wall_max_z_);
  get("minimum_wall_points", minimum_wall_points_);
  get("wall_fit_residual", wall_fit_residual_);
  get("minimum_corridor_width", minimum_corridor_width_);
  get("maximum_corridor_width", maximum_corridor_width_);

  get("lateral_error_gain", lateral_error_gain_);
  get("heading_error_gain", heading_error_gain_);
  get("maximum_wall_correction", maximum_wall_correction_);
  get("stop_on_wall_loss", stop_on_wall_loss_);
  get("wall_loss_speed", wall_loss_speed_);

  get("desired_linear_velocity", desired_linear_velocity_);
  get("minimum_corner_velocity", minimum_corner_velocity_);
  get("maximum_angular_velocity", maximum_angular_velocity_);
  get("minimum_lookahead_distance", minimum_lookahead_distance_);
  get("maximum_lookahead_distance", maximum_lookahead_distance_);
  get("lookahead_time", lookahead_time_);
  get("curvature_scaling_min_radius", curvature_scaling_min_radius_);
  get("speed_preview_distance", speed_preview_distance_);
  get("maximum_lateral_acceleration", maximum_lateral_acceleration_);
  get("maximum_deceleration", maximum_deceleration_);
  get("wheelbase", wheelbase_);
  get("maximum_steering_angle", maximum_steering_angle_);
  get("goal_tolerance", goal_tolerance_);

  get("collision_check_enabled", collision_check_enabled_);
  get("collision_time_horizon", collision_time_horizon_);
  get("collision_step_time", collision_step_time_);
  get("unknown_is_occupied", unknown_is_occupied_);

  if (
    point_stride_ < 1 || minimum_wall_points_ < 2 ||
    wall_max_x_ <= wall_min_x_ || wall_max_z_ <= wall_min_z_ ||
    wall_max_abs_y_ <= wall_min_abs_y_ ||
    maximum_corridor_width_ <= minimum_corridor_width_ ||
    desired_linear_velocity_ <= 0.0 || minimum_corner_velocity_ <= 0.0 ||
    maximum_angular_velocity_ <= 0.0 ||
    maximum_lookahead_distance_ < minimum_lookahead_distance_ ||
    curvature_scaling_min_radius_ <= 0.0 ||
    maximum_lateral_acceleration_ <= 0.0 || maximum_deceleration_ <= 0.0 ||
    wheelbase_ <= 0.0 || maximum_steering_angle_ <= 0.0 ||
    maximum_steering_angle_ >= 1.5707963267948966 ||
    collision_time_horizon_ <= 0.0 || collision_step_time_ <= 0.0)
  {
    throw nav2_core::PlannerException(
            "Wall follower controller parameters are invalid");
  }

  external_speed_limit_.store(desired_linear_velocity_);

  cloud_sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    pointcloud_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(
      &WallFollowerController::pointCloudCallback, this, std::placeholders::_1));

  centerline_pub_ = node->create_publisher<nav_msgs::msg::Path>(
    "~/" + plugin_name_ + "/wall_centerline", rclcpp::QoS(1));

  collision_checker_ = std::make_unique<
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_ros_->getCostmap());

  RCLCPP_INFO(
    logger_,
    "Configured %s using %s in frame %s",
    plugin_name_.c_str(), pointcloud_topic_.c_str(), base_frame_.c_str());
}

void WallFollowerController::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up %s", plugin_name_.c_str());
  cloud_sub_.reset();
  centerline_pub_.reset();
  collision_checker_.reset();
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    latest_cloud_.reset();
  }
  global_plan_.poses.clear();
  transformed_plan_.poses.clear();
  has_valid_plan_ = false;
}

void WallFollowerController::activate()
{
  RCLCPP_INFO(logger_, "Activating %s", plugin_name_.c_str());
  centerline_pub_->on_activate();
}

void WallFollowerController::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating %s", plugin_name_.c_str());
  centerline_pub_->on_deactivate();
}

void WallFollowerController::setSpeedLimit(
  const double & speed_limit, const bool & percentage)
{
  if (speed_limit < 0.0) {
    external_speed_limit_.store(desired_linear_velocity_);
    return;
  }

  const double limit = percentage ?
    desired_linear_velocity_ * std::clamp(speed_limit, 0.0, 100.0) / 100.0 :
    speed_limit;
  external_speed_limit_.store(std::clamp(limit, 0.0, desired_linear_velocity_));
}

void WallFollowerController::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
{
  std::lock_guard<std::mutex> lock(cloud_mutex_);
  latest_cloud_ = cloud;
}

void WallFollowerController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  transformed_plan_ = transformPlan(costmap_ros_->getGlobalFrameID());
  has_valid_plan_ = !transformed_plan_.poses.empty();
  if (!has_valid_plan_) {
    throw nav2_core::PlannerException("Received an empty global plan");
  }
}

nav_msgs::msg::Path WallFollowerController::transformPlan(
  const std::string & target_frame) const
{
  if (global_plan_.poses.empty()) {
    throw nav2_core::PlannerException("Received an empty global plan");
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
            "Unable to transform global plan: " + std::string(ex.what()));
  }

  nav_msgs::msg::Path transformed;
  transformed.header = global_plan_.header;
  transformed.header.frame_id = target_frame;
  transformed.poses.reserve(global_plan_.poses.size());
  for (const auto & input_pose : global_plan_.poses) {
    geometry_msgs::msg::PoseStamped output_pose;
    tf2::doTransform(input_pose, output_pose, transform);
    output_pose.header.frame_id = target_frame;
    transformed.poses.push_back(output_pose);
  }
  return transformed;
}

std::vector<WallFollowerController::LocalPathPoint>
WallFollowerController::buildLocalPath(
  const geometry_msgs::msg::PoseStamped & pose,
  const double preview_distance) const
{
  std::vector<LocalPathPoint> local_path;
  if (transformed_plan_.poses.empty()) {
    return local_path;
  }

  double WallFollowerController::pointCurvature(const LocalPathPoint & point)
  {
    const double squared_distance = point.x * point.x + point.y * point.y;
    if (squared_distance < 1e-6) {
      return 0.0;
    }
    return 2.0 * point.y / squared_distance;
  }

  std::size_t closest_index = 0;
  double closest_distance_squared = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < transformed_plan_.poses.size(); ++index) {
    const double dx =
      transformed_plan_.poses[index].pose.position.x - pose.pose.position.x;
    const double dy =
      transformed_plan_.poses[index].pose.position.y - pose.pose.position.y;
    const double squared_distance = dx * dx + dy * dy;
    if (squared_distance < closest_distance_squared) {
      closest_distance_squared = squared_distance;
      closest_index = index;
    }
  }

  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  double arc_distance = 0.0;
  double previous_x = transformed_plan_.poses[closest_index].pose.position.x;
  double previous_y = transformed_plan_.poses[closest_index].pose.position.y;

  for (std::size_t index = closest_index;
    index < transformed_plan_.poses.size(); ++index)
  {
    const auto & path_position = transformed_plan_.poses[index].pose.position;
    if (index > closest_index) {
      arc_distance += std::hypot(
        path_position.x - previous_x, path_position.y - previous_y);
    }
    previous_x = path_position.x;
    previous_y = path_position.y;

    const double dx = path_position.x - pose.pose.position.x;
    const double dy = path_position.y - pose.pose.position.y;
    local_path.push_back({
      cos_yaw * dx + sin_yaw * dy,
      -sin_yaw * dx + cos_yaw * dy,
      arc_distance});

    if (arc_distance >= preview_distance) {
      break;
    }
  }

  return local_path;
}

WallFollowerController::LineFit WallFollowerController::fitWallLine(
  const std::vector<Point2D> & points) const
{
  const auto fit = [](const std::vector<Point2D> & samples) {
      LineFit result;
      if (samples.size() < 2) {
        return result;
      }

      double sum_x = 0.0;
      double sum_y = 0.0;
      double sum_xx = 0.0;
      double sum_xy = 0.0;
      for (const auto & point : samples) {
        sum_x += point.x;
        sum_y += point.y;
        sum_xx += point.x * point.x;
        sum_xy += point.x * point.y;
      }

      const double count = static_cast<double>(samples.size());
      const double denominator = count * sum_xx - sum_x * sum_x;
      if (std::abs(denominator) < 1e-9) {
        return result;
      }

      result.slope = (count * sum_xy - sum_x * sum_y) / denominator;
      result.intercept = (sum_y - result.slope * sum_x) / count;
      result.inliers = samples.size();
      result.valid = true;
      return result;
    };

  const LineFit initial = fit(points);
  if (!initial.valid) {
    return initial;
  }

  std::vector<Point2D> inliers;
  inliers.reserve(points.size());
  for (const auto & point : points) {
    const double residual =
      std::abs(point.y - (initial.slope * point.x + initial.intercept));
    if (residual <= wall_fit_residual_) {
      inliers.push_back(point);
    }
  }

  LineFit refined = fit(inliers);
  refined.valid =
    refined.valid &&
    refined.inliers >= static_cast<std::size_t>(minimum_wall_points_);
  return refined;
}

WallFollowerController::WallEstimate WallFollowerController::estimateWalls(
  const sensor_msgs::msg::PointCloud2 & cloud) const
{
  WallEstimate estimate;
  tf2::Transform cloud_to_base;
  cloud_to_base.setIdentity();

  if (cloud.header.frame_id != base_frame_) {
    try {
      const auto transform = tf_->lookupTransform(
        base_frame_, cloud.header.frame_id, tf2::TimePointZero);
      tf2::fromMsg(transform.transform, cloud_to_base);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 2000,
        "Cannot transform wall cloud from %s to %s: %s",
        cloud.header.frame_id.c_str(), base_frame_.c_str(), ex.what());
      return estimate;
    }
  }

  std::vector<Point2D> left_points;
  std::vector<Point2D> right_points;

  sensor_msgs::PointCloud2ConstIterator<float> x_iterator(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_iterator(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_iterator(cloud, "z");

  int point_index = 0;
  for (; x_iterator != x_iterator.end();
    ++x_iterator, ++y_iterator, ++z_iterator, ++point_index)
  {
    if (point_index % point_stride_ != 0) {
      continue;
    }

    const tf2::Vector3 transformed_point =
      cloud_to_base * tf2::Vector3(*x_iterator, *y_iterator, *z_iterator);
    const double x = transformed_point.x();
    const double y = transformed_point.y();
    const double z = transformed_point.z();

    if (
      !std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      x < wall_min_x_ || x > wall_max_x_ ||
      z < wall_min_z_ || z > wall_max_z_ ||
      std::abs(y) < wall_min_abs_y_ || std::abs(y) > wall_max_abs_y_)
    {
      continue;
    }

    if (y > 0.0) {
      left_points.push_back({x, y});
    } else {
      right_points.push_back({x, y});
    }
  }

  const LineFit left_wall = fitWallLine(left_points);
  const LineFit right_wall = fitWallLine(right_points);
  if (!left_wall.valid || !right_wall.valid) {
    return estimate;
  }

  estimate.corridor_width = left_wall.intercept - right_wall.intercept;
  if (
    estimate.corridor_width < minimum_corridor_width_ ||
    estimate.corridor_width > maximum_corridor_width_)
  {
    return estimate;
  }

  estimate.center_offset =
    0.5 * (left_wall.intercept + right_wall.intercept);
  estimate.center_slope = 0.5 * (left_wall.slope + right_wall.slope);
  estimate.heading_error = std::atan(estimate.center_slope);
  estimate.valid = true;
  return estimate;
}

double WallFollowerController::calculateTargetSpeed(
  const std::vector<LocalPathPoint> & local_path,
  const double current_curvature) const
{
  const double speed_limit = external_speed_limit_.load();
  const double cruise_speed =
    std::min(desired_linear_velocity_, speed_limit);
  double target_speed = cruise_speed;

  const auto curvatureSpeed = [&](const double curvature) {
      const double absolute_curvature = std::abs(curvature);
      if (absolute_curvature < 1e-6) {
        return cruise_speed;
      }

      const double radius = 1.0 / absolute_curvature;
      double speed = cruise_speed;
      if (radius < curvature_scaling_min_radius_) {
        const double minimum_speed =
          std::min(minimum_corner_velocity_, cruise_speed);
        speed = std::max(
          minimum_speed,
          cruise_speed * radius / curvature_scaling_min_radius_);
      }
      speed = std::min(
        speed,
        std::sqrt(maximum_lateral_acceleration_ / absolute_curvature));
      speed = std::min(speed, maximum_angular_velocity_ / absolute_curvature);
      return speed;
    };

  target_speed = std::min(target_speed, curvatureSpeed(current_curvature));

  for (const auto & point : local_path) {
    if (point.arc_distance <= 0.0) {
      continue;
    }
    const double corner_speed = curvatureSpeed(pointCurvature(point));
    const double braking_limited_speed = std::sqrt(
      corner_speed * corner_speed +
      2.0 * maximum_deceleration_ * point.arc_distance);
    target_speed = std::min(target_speed, braking_limited_speed);
  }

  return std::max(0.0, target_speed);
}

bool WallFollowerController::commandIsCollisionFree(
  const geometry_msgs::msg::PoseStamped & pose,
  const double linear_velocity,
  const double angular_velocity) const
{
  if (!collision_check_enabled_ || !collision_checker_) {
    return true;
  }

  double x = pose.pose.position.x;
  double y = pose.pose.position.y;
  double yaw = tf2::getYaw(pose.pose.orientation);
  const auto footprint = costmap_ros_->getRobotFootprint();

  for (double time = 0.0;
    time <= collision_time_horizon_; time += collision_step_time_)
  {
    const double cost =
      collision_checker_->footprintCostAtPose(x, y, yaw, footprint);
    const bool is_unknown = cost == nav2_costmap_2d::NO_INFORMATION;
    const bool is_lethal =
      !is_unknown && cost >= nav2_costmap_2d::LETHAL_OBSTACLE;
    if (cost < 0.0 || is_lethal || (unknown_is_occupied_ && is_unknown)) {
      return false;
    }

    x += linear_velocity * std::cos(yaw) * collision_step_time_;
    y += linear_velocity * std::sin(yaw) * collision_step_time_;
    yaw += angular_velocity * collision_step_time_;
  }

  return true;
}

void WallFollowerController::publishWallCenterline(
  const WallEstimate & walls)
{
  if (!centerline_pub_ || !centerline_pub_->is_activated() || !walls.valid) {
    return;
  }

  nav_msgs::msg::Path path;
  path.header.frame_id = base_frame_;
  path.header.stamp = clock_->now();

  constexpr int sample_count = 20;
  for (int index = 0; index <= sample_count; ++index) {
    const double ratio = static_cast<double>(index) / sample_count;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x =
      wall_min_x_ + ratio * (wall_max_x_ - wall_min_x_);
    pose.pose.position.y =
      walls.center_slope * pose.pose.position.x + walls.center_offset;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }

  centerline_pub_->publish(path);
}

geometry_msgs::msg::TwistStamped WallFollowerController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  (void)goal_checker;
  if (!has_valid_plan_) {
    throw nav2_core::PlannerException("No valid global plan is available");
  }

  geometry_msgs::msg::TwistStamped command;
  command.header.frame_id = base_frame_;
  command.header.stamp = clock_->now();

  const auto & goal = transformed_plan_.poses.back().pose.position;
  if (std::hypot(
      goal.x - pose.pose.position.x,
      goal.y - pose.pose.position.y) <= goal_tolerance_)
  {
    return command;
  }

  sensor_msgs::msg::PointCloud2::SharedPtr cloud;
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    cloud = latest_cloud_;
  }

  WallEstimate walls;
  bool cloud_is_fresh = false;
  if (cloud) {
    const rclcpp::Time cloud_time(cloud->header.stamp);
    cloud_is_fresh =
      std::abs((clock_->now() - cloud_time).seconds()) <= cloud_timeout_;
    if (cloud_is_fresh) {
      walls = estimateWalls(*cloud);
    }
  }

  if ((!cloud_is_fresh || !walls.valid) && stop_on_wall_loss_) {
    throw nav2_core::PlannerException(
            "Fresh, valid left and right wall estimates are unavailable");
  }

  const double lookahead_distance = std::clamp(
    std::abs(velocity.linear.x) * lookahead_time_,
    minimum_lookahead_distance_,
    maximum_lookahead_distance_);
  const double required_preview =
    std::max(speed_preview_distance_, lookahead_distance);
  const auto local_path = buildLocalPath(pose, required_preview);
  if (local_path.empty()) {
    throw nav2_core::PlannerException("Unable to build local path");
  }

  const LocalPathPoint * target = &local_path.back();
  for (const auto & point : local_path) {
    if (point.arc_distance >= lookahead_distance && point.x > 0.0) {
      target = &point;
      break;
    }
  }

  double curvature = pointCurvature(*target);
  if (walls.valid) {
    const double wall_correction = clampMagnitude(
      lateral_error_gain_ * walls.center_offset +
      heading_error_gain_ * walls.heading_error,
      maximum_wall_correction_);
    curvature += wall_correction;
    publishWallCenterline(walls);
  }

  const double maximum_curvature =
    std::tan(maximum_steering_angle_) / wheelbase_;
  curvature = clampMagnitude(curvature, maximum_curvature);

  double linear_velocity = calculateTargetSpeed(local_path, curvature);
  if (!walls.valid) {
    linear_velocity = std::min(linear_velocity, wall_loss_speed_);
  }
  double angular_velocity = linear_velocity * curvature;
  if (std::abs(angular_velocity) > maximum_angular_velocity_) {
    linear_velocity = std::min(
      linear_velocity,
      maximum_angular_velocity_ / std::max(std::abs(curvature), 1e-6));
    angular_velocity = linear_velocity * curvature;
  }

  if (!commandIsCollisionFree(pose, linear_velocity, angular_velocity)) {
    throw nav2_core::PlannerException(
            "Wall follower command predicts a footprint collision");
  }

  command.twist.linear.x = linear_velocity;
  command.twist.angular.z = angular_velocity;

  RCLCPP_DEBUG(
    logger_,
    "wall valid=%d offset=%.3f heading=%.3f width=%.3f curvature=%.3f "
    "cmd=[%.3f, %.3f]",
    walls.valid, walls.center_offset, walls.heading_error, walls.corridor_width,
    curvature, linear_velocity, angular_velocity);

  return command;
}

}  // namespace wall_follower_controller

PLUGINLIB_EXPORT_CLASS(
  wall_follower_controller::WallFollowerController,
  nav2_core::Controller);
