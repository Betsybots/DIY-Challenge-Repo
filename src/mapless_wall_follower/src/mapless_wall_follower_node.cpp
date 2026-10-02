#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace mapless_wall_follower
{

class MaplessWallFollower : public rclcpp::Node
{
public:
  MaplessWallFollower()
  : Node("mapless_wall_follower")
  {
    cloud_topic_ = declare_parameter("cloud_topic", std::string("/cloud_registered_body"));
    // Cloud frames accepted as the robot body frame (x forward, y left, z up).
    // FAST-LIO in this repo stamps /cloud_registered_body as base_footprint.
    accepted_cloud_frames_ = declare_parameter(
      "accepted_cloud_frames", std::vector<std::string>{"base_link", "base_footprint"});
    cmd_vel_topic_ = declare_parameter("cmd_vel_topic", std::string("/cmd_vel_nav"));
    enable_topic_ = declare_parameter(
      "enable_topic", std::string("/mapless_wall_follower/enable"));
    state_topic_ = declare_parameter(
      "state_topic", std::string("/mapless_wall_follower/state"));
    debug_log_frequency_ = declare_parameter("debug_log_frequency", 2.0);
    enabled_.store(declare_parameter("start_enabled", false));
    control_frequency_ = declare_parameter("control_frequency", 20.0);
    cloud_timeout_ = declare_parameter("cloud_timeout", 0.30);
    point_stride_ = declare_parameter("point_stride", 3);

    min_x_ = declare_parameter("min_x", -0.20);
    max_x_ = declare_parameter("max_x", 3.00);
    min_z_ = declare_parameter("min_z", 0.10);
    max_z_ = declare_parameter("max_z", 1.20);
    side_min_abs_y_ = declare_parameter("side_min_abs_y", 0.22);
    side_max_abs_y_ = declare_parameter("side_max_abs_y", 1.50);
    front_half_width_ = declare_parameter("front_half_width", 0.30);
    min_wall_points_ = declare_parameter("min_wall_points", 20);
    fit_residual_threshold_ = declare_parameter("fit_residual_threshold", 0.08);
    max_fit_rms_ = declare_parameter("max_fit_rms", 0.06);
    max_fit_points_ = declare_parameter("max_fit_points", 2000);
    wall_detection_hold_time_ = declare_parameter("wall_detection_hold_time", 0.20);

    wall_lookahead_ = declare_parameter("wall_lookahead", 0.80);
    min_corridor_width_ = declare_parameter("min_corridor_width", 0.70);
    max_corridor_width_ = declare_parameter("max_corridor_width", 1.40);
    right_wall_target_distance_ = declare_parameter("right_wall_target_distance", 0.46);
    left_wall_target_distance_ = declare_parameter("left_wall_target_distance", 0.46);
    center_gain_ = declare_parameter("center_gain", 1.8);
    right_wall_gain_ = declare_parameter("right_wall_gain", 2.2);
    left_wall_gain_ = declare_parameter("left_wall_gain", 2.2);
    heading_gain_ = declare_parameter("heading_gain", 1.6);
    right_turn_curvature_bias_ = declare_parameter("right_turn_curvature_bias", 0.80);

    turn_enter_front_distance_ = declare_parameter("turn_enter_front_distance", 1.20);
    emergency_stop_distance_ = declare_parameter("emergency_stop_distance", 0.45);

    straight_speed_ = declare_parameter("straight_speed", 0.30);
    turn_speed_ = declare_parameter("turn_speed", 0.15);
    min_speed_ = declare_parameter("min_speed", 0.15);
    max_lateral_acceleration_ = declare_parameter("max_lateral_acceleration", 0.50);
    max_curvature_ = declare_parameter("max_curvature", 2.70);
    max_yaw_rate_ = declare_parameter("max_yaw_rate", 1.35);
    max_linear_acceleration_ = declare_parameter("max_linear_acceleration", 0.40);
    max_linear_deceleration_ = declare_parameter("max_linear_deceleration", 0.80);
    max_curvature_rate_ = declare_parameter("max_curvature_rate", 3.00);
    preview_wall_offset_ = declare_parameter("preview_wall_offset", 0.16);
    control_latency_ = declare_parameter("control_latency", 0.20);
    curvature_feedforward_gain_ = declare_parameter("curvature_feedforward_gain", 0.0);
    curvature_fit_min_span_ = declare_parameter("curvature_fit_min_span", 1.0);

    recovery_max_attempts_ = declare_parameter("recovery_max_attempts", 3);
    recovery_reverse_distance_ = declare_parameter("recovery_reverse_distance", 0.08);
    recovery_reverse_speed_ = declare_parameter("recovery_reverse_speed", 0.20);
    recovery_curvature_ = declare_parameter("recovery_curvature", 1.50);
    recovery_pause_ = declare_parameter("recovery_pause", 0.30);
    recovery_reset_distance_ = declare_parameter("recovery_reset_distance", 0.50);
    recovery_flip_after_attempts_ = declare_parameter("recovery_flip_after_attempts", 2);

    if (control_frequency_ <= 0.0 || debug_log_frequency_ <= 0.0 ||
      point_stride_ < 1 || min_wall_points_ < 2 ||
      max_fit_points_ < min_wall_points_ || min_x_ >= max_x_ || min_z_ >= max_z_ ||
      wall_detection_hold_time_ < 0.0 || wall_detection_hold_time_ > cloud_timeout_ ||
      side_min_abs_y_ >= side_max_abs_y_ || wall_lookahead_ <= 0.0 ||
      min_corridor_width_ >= max_corridor_width_ ||
      right_wall_target_distance_ <= 0.0 || left_wall_target_distance_ <= 0.0 ||
      right_wall_gain_ <= 0.0 || left_wall_gain_ <= 0.0 ||
      emergency_stop_distance_ <= 0.0 ||
      emergency_stop_distance_ >= turn_enter_front_distance_ ||
      straight_speed_ <= 0.0 || turn_speed_ <= 0.0 ||
      min_speed_ <= 0.0 || min_speed_ > turn_speed_ || min_speed_ > straight_speed_ ||
      max_curvature_ <= 0.0 || max_yaw_rate_ <= 0.0 ||
      max_linear_acceleration_ <= 0.0 || max_linear_deceleration_ <= 0.0 ||
      max_curvature_rate_ <= 0.0 || preview_wall_offset_ <= 0.0 ||
      control_latency_ < 0.0 || curvature_feedforward_gain_ < 0.0 ||
      curvature_fit_min_span_ <= 0.0 ||
      recovery_max_attempts_ < 0 || recovery_reverse_distance_ <= 0.0 ||
      recovery_reverse_speed_ <= 0.0 || recovery_curvature_ < 0.0 ||
      recovery_curvature_ > max_curvature_ || recovery_pause_ < 0.0 ||
      recovery_reset_distance_ <= 0.0 || recovery_flip_after_attempts_ < 0)
    {
      throw std::invalid_argument("Invalid mapless wall follower parameters");
    }

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MaplessWallFollower::cloudCallback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      enable_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        enabled_.store(message->data);
        RCLCPP_INFO(
          get_logger(), "Wall follower %s by enable command",
          message->data ? "ENABLED" : "DISABLED");
      });
    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / control_frequency_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MaplessWallFollower::controlLoop, this));
    last_control_time_ = now();
    last_debug_log_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "Mapless wall follower ready: cloud=%s, enable=%s, cmd=%s, state=%s",
      cloud_topic_.c_str(), enable_topic_.c_str(), cmd_vel_topic_.c_str(),
      state_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "Expected cloud frame/axes: base_footprint, +x forward, +y left, +z up");
  }

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
    double heading{0.0};
    double rms{std::numeric_limits<double>::infinity()};
    std::size_t inliers{0};
    // Signed wall curvature from the quadratic refit; slope/intercept are then its tangent at x=0.
    double curvature{0.0};
    bool curvature_valid{false};
  };

  struct WallMeasurement
  {
    LineFit left;
    LineFit right;
    double left_y{0.0};
    double right_y{0.0};
    double front_distance{std::numeric_limits<double>::infinity()};
    double path_clearance{std::numeric_limits<double>::infinity()};
    double path_clearance_y{0.0};
    std::size_t left_point_count{0};
    std::size_t right_point_count{0};
    bool left_held{false};
    bool right_held{false};
    bool pair_inconsistent{false};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  struct SpeedProfile
  {
    double preview_curvature{0.0};
    double curve_limit{std::numeric_limits<double>::infinity()};
    double clearance_limit{std::numeric_limits<double>::infinity()};
  };

  enum class Mode
  {
    STOPPED,
    CENTERING,
    RIGHT_WALL,
    LEFT_WALL
  };

  enum class RecoveryPhase
  {
    NONE,
    BRAKING,
    REVERSING,
    SETTLING
  };

  static LineFit leastSquares(const std::vector<Point2D> & points)
  {
    LineFit fit;
    if (points.size() < 2) {
      return fit;
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_xx = 0.0;
    double sum_xy = 0.0;
    for (const auto & point : points) {
      sum_x += point.x;
      sum_y += point.y;
      sum_xx += point.x * point.x;
      sum_xy += point.x * point.y;
    }

    const double count = static_cast<double>(points.size());
    const double denominator = count * sum_xx - sum_x * sum_x;
    if (std::abs(denominator) < 1e-8) {
      return fit;
    }

    fit.slope = (count * sum_xy - sum_x * sum_y) / denominator;
    fit.intercept = (sum_y - fit.slope * sum_x) / count;
    fit.heading = std::atan(fit.slope);
    fit.inliers = points.size();

    double squared_error = 0.0;
    const double normalizer = std::sqrt(1.0 + fit.slope * fit.slope);
    for (const auto & point : points) {
      const double residual =
        (point.y - fit.slope * point.x - fit.intercept) / normalizer;
      squared_error += residual * residual;
    }
    fit.rms = std::sqrt(squared_error / count);
    fit.valid = true;
    return fit;
  }

  LineFit fitWall(const std::vector<Point2D> & points) const
  {
    if (points.size() < static_cast<std::size_t>(min_wall_points_)) {
      return {};
    }

    const std::size_t candidate_count = std::min<std::size_t>(20, points.size());
    const std::size_t stride = std::max<std::size_t>(1, points.size() / candidate_count);
    std::size_t best_inlier_count = 0;
    double best_rms = std::numeric_limits<double>::infinity();
    double best_slope = 0.0;
    double best_intercept = 0.0;

    for (std::size_t first = 0; first < points.size(); first += stride) {
      for (std::size_t second = first + stride; second < points.size(); second += stride) {
        const double dx = points[second].x - points[first].x;
        if (std::abs(dx) < 0.20) {
          continue;
        }

        const double slope = (points[second].y - points[first].y) / dx;
        const double intercept = points[first].y - slope * points[first].x;
        const double normalizer = std::sqrt(1.0 + slope * slope);
        std::size_t inlier_count = 0;
        double squared_error = 0.0;

        for (const auto & point : points) {
          const double residual =
            std::abs(point.y - slope * point.x - intercept) / normalizer;
          if (residual <= fit_residual_threshold_) {
            ++inlier_count;
            squared_error += residual * residual;
          }
        }

        if (inlier_count == 0) {
          continue;
        }
        const double rms = std::sqrt(squared_error / static_cast<double>(inlier_count));
        if (inlier_count > best_inlier_count ||
          (inlier_count == best_inlier_count && rms < best_rms))
        {
          best_inlier_count = inlier_count;
          best_rms = rms;
          best_slope = slope;
          best_intercept = intercept;
        }
      }
    }

    if (best_inlier_count < static_cast<std::size_t>(min_wall_points_)) {
      return {};
    }

    std::vector<Point2D> inliers;
    inliers.reserve(best_inlier_count);
    const double normalizer = std::sqrt(1.0 + best_slope * best_slope);
    for (const auto & point : points) {
      const double residual =
        std::abs(point.y - best_slope * point.x - best_intercept) / normalizer;
      if (residual <= fit_residual_threshold_) {
        inliers.push_back(point);
      }
    }

    LineFit fit = leastSquares(inliers);
    fit.valid =
      fit.valid &&
      fit.inliers >= static_cast<std::size_t>(min_wall_points_) &&
      fit.rms <= max_fit_rms_;
    if (fit.valid && curvature_feedforward_gain_ > 0.0) {
      refineQuadratic(points, fit);
    }
    return fit;
  }

  // Refit y = a + b*x + c*x^2 around the line fit so curved walls keep their bend.
  void refineQuadratic(const std::vector<Point2D> & points, LineFit & fit) const
  {
    // Steep walls (hairpin front wall) are not functions of x; keep the line fit.
    if (std::abs(fit.slope) > 1.0) {
      return;
    }
    double a = fit.intercept;
    double b = fit.slope;
    double c = 0.0;
    for (int iteration = 0; iteration < 3; ++iteration) {
      double s[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
      double t[3] = {0.0, 0.0, 0.0};
      double min_x = std::numeric_limits<double>::infinity();
      double max_x = -std::numeric_limits<double>::infinity();
      std::size_t count = 0;
      for (const auto & point : points) {
        if (std::abs(point.y - (a + b * point.x + c * point.x * point.x)) >
          fit_residual_threshold_)
        {
          continue;
        }
        const double x2 = point.x * point.x;
        s[0] += 1.0;
        s[1] += point.x;
        s[2] += x2;
        s[3] += x2 * point.x;
        s[4] += x2 * x2;
        t[0] += point.y;
        t[1] += point.x * point.y;
        t[2] += x2 * point.y;
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        ++count;
      }
      if (count < static_cast<std::size_t>(min_wall_points_) ||
        max_x - min_x < curvature_fit_min_span_)
      {
        return;
      }
      // Cramer's rule on the 3x3 normal equations.
      const double det =
        s[0] * (s[2] * s[4] - s[3] * s[3]) - s[1] * (s[1] * s[4] - s[3] * s[2]) +
        s[2] * (s[1] * s[3] - s[2] * s[2]);
      if (std::abs(det) < 1e-9) {
        return;
      }
      a = (t[0] * (s[2] * s[4] - s[3] * s[3]) - s[1] * (t[1] * s[4] - s[3] * t[2]) +
        s[2] * (t[1] * s[3] - s[2] * t[2])) / det;
      b = (s[0] * (t[1] * s[4] - t[2] * s[3]) - t[0] * (s[1] * s[4] - s[3] * s[2]) +
        s[2] * (s[1] * t[2] - t[1] * s[2])) / det;
      c = (s[0] * (s[2] * t[2] - s[3] * t[1]) - s[1] * (s[1] * t[2] - t[1] * s[2]) +
        t[0] * (s[1] * s[3] - s[2] * s[2])) / det;
    }
    double squared_error = 0.0;
    std::size_t inliers = 0;
    for (const auto & point : points) {
      const double residual = point.y - (a + b * point.x + c * point.x * point.x);
      if (std::abs(residual) <= fit_residual_threshold_) {
        squared_error += residual * residual;
        ++inliers;
      }
    }
    if (inliers < static_cast<std::size_t>(min_wall_points_)) {
      return;
    }
    fit.intercept = a;
    fit.slope = b;
    fit.heading = std::atan(b);
    fit.curvature = std::clamp(
      2.0 * c / std::pow(1.0 + b * b, 1.5), -max_curvature_, max_curvature_);
    fit.curvature_valid = true;
    fit.inliers = inliers;
    fit.rms = std::sqrt(squared_error / static_cast<double>(inliers));
  }

  // Arc length along the commanded path before the point enters the swept corridor.
  double arcDistance(double x, double y, double curvature) const
  {
    if (std::abs(curvature) < 1e-3) {
      return (x > 0.0 && std::abs(y) <= front_half_width_) ?
             x : std::numeric_limits<double>::infinity();
    }
    const double radius = 1.0 / std::abs(curvature);
    const double inner_y = curvature > 0.0 ? y : -y;
    const double radial = std::hypot(x, inner_y - radius);
    if (std::abs(radial - radius) > front_half_width_) {
      return std::numeric_limits<double>::infinity();
    }
    const double angle = std::atan2(x, radius - inner_y);
    return angle > 0.0 ? radius * angle : std::numeric_limits<double>::infinity();
  }

  // Centerline curvature of a bend whose outer wall enters the front band at this distance.
  double previewCurvature(double front_distance) const
  {
    if (!std::isfinite(front_distance)) {
      return 0.0;
    }
    const double offset = preview_wall_offset_;
    if (front_distance <= offset) {
      return max_curvature_;
    }
    return std::min(
      max_curvature_,
      2.0 * offset / (front_distance * front_distance - offset * offset));
  }

  static bool firstFitIsBetter(const LineFit & first, const LineFit & second)
  {
    if (first.inliers != second.inliers) {
      return first.inliers > second.inliers;
    }
    return first.rms <= second.rms;
  }

  // On an impossible corridor width keep the better wall instead of dropping both.
  void rejectInconsistentWallPair(WallMeasurement & measurement) const
  {
    if (!measurement.left.valid || !measurement.right.valid) {
      return;
    }
    const double width = measurement.left_y - measurement.right_y;
    if (width >= min_corridor_width_ && width <= max_corridor_width_) {
      return;
    }
    measurement.pair_inconsistent = true;
    if (firstFitIsBetter(measurement.left, measurement.right)) {
      measurement.right.valid = false;
      measurement.right_held = false;
    } else {
      measurement.left.valid = false;
      measurement.left_held = false;
    }
  }

  bool isRecent(const rclcpp::Time & stamp, const rclcpp::Time & current) const
  {
    if (stamp.nanoseconds() == 0) {
      return false;
    }
    const double age = (current - stamp).seconds();
    return age >= 0.0 && age <= wall_detection_hold_time_;
  }

  void holdRecentWalls(WallMeasurement & measurement) const
  {
    if (!measurement.left.valid && isRecent(last_left_wall_stamp_, measurement.stamp)) {
      measurement.left = last_left_wall_;
      measurement.left_y = last_left_y_;
      measurement.left.valid = true;
      measurement.left_held = true;
    }
    if (!measurement.right.valid && isRecent(last_right_wall_stamp_, measurement.stamp)) {
      measurement.right = last_right_wall_;
      measurement.right_y = last_right_y_;
      measurement.right.valid = true;
      measurement.right_held = true;
    }
  }

  void rememberCurrentWalls(const WallMeasurement & measurement)
  {
    if (measurement.left.valid && !measurement.left_held) {
      last_left_wall_ = measurement.left;
      last_left_y_ = measurement.left_y;
      last_left_wall_stamp_ = measurement.stamp;
    }
    if (measurement.right.valid && !measurement.right_held) {
      last_right_wall_ = measurement.right;
      last_right_y_ = measurement.right_y;
      last_right_wall_stamp_ = measurement.stamp;
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    if (std::find(accepted_cloud_frames_.begin(), accepted_cloud_frames_.end(),
        cloud->header.frame_id) == accepted_cloud_frames_.end())
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring cloud in frame %s (accepted_cloud_frames does not include it)",
        cloud->header.frame_id.c_str());
      return;
    }

    std::vector<Point2D> left_points;
    std::vector<Point2D> right_points;
    left_points.reserve(std::min<std::size_t>(cloud->width, max_fit_points_));
    right_points.reserve(std::min<std::size_t>(cloud->width, max_fit_points_));
    double front_distance = std::numeric_limits<double>::infinity();
    double path_clearance = std::numeric_limits<double>::infinity();
    double path_clearance_y = 0.0;
    const double clearance_curvature = commanded_curvature_.load();

    try {
      sensor_msgs::PointCloud2ConstIterator<float> x_iterator(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y_iterator(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z_iterator(*cloud, "z");
      std::size_t point_index = 0;

      for (; x_iterator != x_iterator.end(); ++x_iterator, ++y_iterator, ++z_iterator) {
        if ((point_index++ % static_cast<std::size_t>(point_stride_)) != 0) {
          continue;
        }

        const double x = *x_iterator;
        const double y = *y_iterator;
        const double z = *z_iterator;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
          z < min_z_ || z > max_z_)
        {
          continue;
        }

        if (x > 0.0 && std::abs(y) <= front_half_width_) {
          front_distance = std::min(front_distance, x);
        }
        const double arc_distance = arcDistance(x, y, clearance_curvature);
        if (arc_distance < path_clearance) {
          path_clearance = arc_distance;
          path_clearance_y = y;
        }
        if (x < min_x_ || x > max_x_) {
          continue;
        }

        if (y >= side_min_abs_y_ && y <= side_max_abs_y_ &&
          left_points.size() < static_cast<std::size_t>(max_fit_points_))
        {
          left_points.push_back({x, y});
        } else if (
          y <= -side_min_abs_y_ && y >= -side_max_abs_y_ &&
          right_points.size() < static_cast<std::size_t>(max_fit_points_))
        {
          right_points.push_back({x, y});
        }
      }
    } catch (const std::runtime_error & exception) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Invalid PointCloud2 fields: %s", exception.what());
      return;
    }

    WallMeasurement measurement;
    measurement.left = fitWall(left_points);
    measurement.right = fitWall(right_points);
    measurement.front_distance = front_distance;
    measurement.path_clearance = path_clearance;
    measurement.path_clearance_y = path_clearance_y;
    measurement.left_point_count = left_points.size();
    measurement.right_point_count = right_points.size();
    measurement.stamp = now();

    if (measurement.left.valid) {
      measurement.left_y =
        measurement.left.slope * wall_lookahead_ + measurement.left.intercept;
      measurement.left.valid = measurement.left_y > 0.0;
    }
    if (measurement.right.valid) {
      measurement.right_y =
        measurement.right.slope * wall_lookahead_ + measurement.right.intercept;
      measurement.right.valid = measurement.right_y < 0.0;
    }
    rejectInconsistentWallPair(measurement);
    holdRecentWalls(measurement);
    rejectInconsistentWallPair(measurement);
    rememberCurrentWalls(measurement);

    std::lock_guard<std::mutex> lock(measurement_mutex_);
    latest_measurement_ = measurement;
    have_measurement_ = true;
  }

  void controlLoop()
  {
    WallMeasurement measurement;
    {
      std::lock_guard<std::mutex> lock(measurement_mutex_);
      if (!have_measurement_) {
        stop("WAITING_FOR_CLOUD");
        return;
      }
      measurement = latest_measurement_;
    }

    const rclcpp::Time current_time = now();
    const double cloud_age = (current_time - measurement.stamp).seconds();
    double dt = (current_time - last_control_time_).seconds();
    last_control_time_ = current_time;
    if (dt <= 0.0 || dt > 0.5) {
      dt = 1.0 / control_frequency_;
    }

    if (!enabled_.load()) {
      mode_ = Mode::STOPPED;
      recovery_phase_ = RecoveryPhase::NONE;
      recovery_attempts_ = 0;
      stop("DISABLED", dt, &measurement, cloud_age);
      return;
    }

    if (cloud_age > cloud_timeout_) {
      mode_ = Mode::STOPPED;
      recovery_phase_ = RecoveryPhase::NONE;
      stop("STALE_CLOUD", dt, &measurement, cloud_age);
      return;
    }
    if (recovery_phase_ != RecoveryPhase::NONE) {
      runRecovery(dt, measurement, cloud_age);
      return;
    }
    if (measurement.path_clearance <= emergency_stop_distance_) {
      mode_ = Mode::STOPPED;
      if (recovery_attempts_ < recovery_max_attempts_) {
        startRecovery(measurement);
        runRecovery(dt, measurement, cloud_age);
        return;
      }
      stop("EMERGENCY_FRONT_STOP", dt, &measurement, cloud_age);
      return;
    }

    const bool both_walls = measurement.left.valid && measurement.right.valid;
    if (both_walls) {
      mode_ = Mode::CENTERING;
    } else if (measurement.left.valid) {
      mode_ = Mode::LEFT_WALL;
    } else if (measurement.right.valid) {
      mode_ = Mode::RIGHT_WALL;
    } else {
      mode_ = Mode::STOPPED;
      stop("NO_WALL_DETECTED", dt, &measurement, cloud_age);
      return;
    }

    double curvature = 0.0;
    double road_curvature = 0.0;
    double requested_speed = straight_speed_;
    double lateral_error = 0.0;
    double heading_error = 0.0;
    std::string state = "CENTERING";

    if (mode_ == Mode::CENTERING) {
      lateral_error = 0.5 * (measurement.left_y + measurement.right_y);
      heading_error =
        0.5 * (measurement.left.heading + measurement.right.heading);
      const int curved_walls =
        static_cast<int>(measurement.left.curvature_valid) +
        static_cast<int>(measurement.right.curvature_valid);
      if (curved_walls > 0) {
        road_curvature = curvature_feedforward_gain_ *
          (measurement.left.curvature + measurement.right.curvature) / curved_walls;
      }
      curvature = road_curvature + center_gain_ * lateral_error + heading_gain_ * heading_error;
    } else if (mode_ == Mode::RIGHT_WALL) {
      lateral_error = measurement.right_y + right_wall_target_distance_;
      heading_error = measurement.right.heading;
      curvature =
        right_wall_gain_ * lateral_error +
        heading_gain_ * heading_error;
      if (measurement.front_distance < turn_enter_front_distance_) {
        const double front_ratio = std::clamp(
          (turn_enter_front_distance_ - measurement.front_distance) /
          (turn_enter_front_distance_ - emergency_stop_distance_),
          0.0, 1.0);
        curvature -= right_turn_curvature_bias_ * front_ratio;
      }
      requested_speed = turn_speed_;
      state = "RIGHT_WALL";
    } else if (mode_ == Mode::LEFT_WALL) {
      lateral_error = measurement.left_y - left_wall_target_distance_;
      heading_error = measurement.left.heading;
      curvature =
        left_wall_gain_ * lateral_error +
        heading_gain_ * heading_error;
      requested_speed = turn_speed_;
      state = "LEFT_WALL";
    } else {
      stop("STOPPED", dt, &measurement, cloud_age);
      return;
    }

    curvature = std::clamp(curvature, -max_curvature_, max_curvature_);

    SpeedProfile profile;
    profile.preview_curvature = previewCurvature(measurement.front_distance);
    const double profile_curvature =
      std::max(std::abs(curvature), profile.preview_curvature);
    if (profile_curvature > 1e-4) {
      profile.curve_limit = std::min(
        std::sqrt(max_lateral_acceleration_ / profile_curvature),
        max_yaw_rate_ / profile_curvature);
    }
    const double braking_distance =
      measurement.path_clearance - emergency_stop_distance_ -
      last_linear_command_ * control_latency_;
    profile.clearance_limit =
      std::sqrt(2.0 * max_linear_deceleration_ * std::max(0.0, braking_distance));
    requested_speed = std::max(
      min_speed_,
      std::min({requested_speed, profile.curve_limit, profile.clearance_limit}));
    const double requested_yaw_rate = std::clamp(
      requested_speed * curvature, -max_yaw_rate_, max_yaw_rate_);

    publishRateLimited(requested_speed, curvature, dt);
    forward_since_recovery_ += std::max(0.0, last_linear_command_) * dt;
    if (forward_since_recovery_ >= recovery_reset_distance_) {
      recovery_attempts_ = 0;
    }
    publishState(state);
    logDebug(
      state, &measurement, cloud_age, lateral_error, heading_error,
      curvature, requested_speed, requested_yaw_rate, &profile, road_curvature);
  }

  void startRecovery(const WallMeasurement & measurement)
  {
    // +1 turns the heading left (toward +y); pick the side away from the blocking point.
    const double obstacle_y = measurement.path_clearance_y;
    if (std::abs(obstacle_y) > 0.05 || std::abs(last_curvature_command_) < 1e-3) {
      recovery_turn_sign_ = obstacle_y > 0.0 ? -1.0 : 1.0;
    } else {
      // Head-on: keep turning the way the controller was already steering.
      recovery_turn_sign_ = last_curvature_command_ > 0.0 ? 1.0 : -1.0;
    }
    ++recovery_attempts_;
    const bool flipped =
      recovery_flip_after_attempts_ > 0 && recovery_attempts_ > recovery_flip_after_attempts_;
    if (flipped) {
      recovery_turn_sign_ = -recovery_turn_sign_;
    }
    forward_since_recovery_ = 0.0;
    recovery_travel_ = 0.0;
    recovery_timer_ = 0.0;
    recovery_phase_ = RecoveryPhase::BRAKING;
    RCLCPP_WARN(
      get_logger(),
      "Wall too close (clearance %.3f m, y %.3f m): recovery %d/%d, reversing and turning %s%s",
      measurement.path_clearance, obstacle_y, recovery_attempts_, recovery_max_attempts_,
      recovery_turn_sign_ > 0.0 ? "LEFT" : "RIGHT", flipped ? " (FLIPPED fallback)" : "");
  }

  // Stop, reverse a short distance while yawing away from the wall, then stop and pre-steer away.
  void runRecovery(double dt, const WallMeasurement & measurement, double cloud_age)
  {
    // Twist yaw = speed * curvature, so reversing needs the opposite curvature sign.
    const double reverse_curvature = -recovery_turn_sign_ * recovery_curvature_;
    const double forward_curvature = recovery_turn_sign_ * recovery_curvature_;
    std::string state;

    switch (recovery_phase_) {
      case RecoveryPhase::BRAKING:
        state = "RECOVERY_BRAKING";
        if (std::abs(last_linear_command_) >= 1e-3) {
          // Still rolling forward: steer away from the wall, not toward it.
          publishRateLimited(0.0, forward_curvature, dt);
        } else {
          // Stopped: pre-set the reverse steering in place before backing up.
          publishRateLimited(0.0, reverse_curvature, dt);
          recovery_timer_ += dt;
          if (recovery_timer_ >= recovery_pause_ &&
            std::abs(last_curvature_command_ - reverse_curvature) < 0.1)
          {
            recovery_phase_ = RecoveryPhase::REVERSING;
          }
        }
        break;
      case RecoveryPhase::REVERSING:
        state = "RECOVERY_REVERSING";
        publishRateLimited(-recovery_reverse_speed_, reverse_curvature, dt);
        recovery_travel_ += std::abs(last_linear_command_) * dt;
        if (recovery_travel_ >= recovery_reverse_distance_) {
          recovery_timer_ = 0.0;
          recovery_phase_ = RecoveryPhase::SETTLING;
        }
        break;
      case RecoveryPhase::SETTLING:
        state = "RECOVERY_SETTLING";
        publishRateLimited(0.0, forward_curvature, dt);
        if (std::abs(last_linear_command_) < 1e-3) {
          recovery_timer_ += dt;
          if (recovery_timer_ >= recovery_pause_) {
            recovery_phase_ = RecoveryPhase::NONE;
          }
        }
        break;
      case RecoveryPhase::NONE:
        return;
    }

    publishState(state);
    logDebug(state, &measurement, cloud_age, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  // Limits speed and curvature separately so braking never tightens the commanded arc.
  void publishRateLimited(double speed, double curvature, double dt)
  {
    const double speed_delta = speed - last_linear_command_;
    const double speed_limit =
      (speed_delta >= 0.0 ? max_linear_acceleration_ : max_linear_deceleration_) * dt;
    const double curvature_limit = max_curvature_rate_ * dt;
    last_linear_command_ += std::clamp(speed_delta, -speed_limit, speed_limit);
    last_curvature_command_ += std::clamp(
      curvature - last_curvature_command_, -curvature_limit, curvature_limit);
    commanded_curvature_.store(last_curvature_command_);
    last_yaw_command_ = std::clamp(
      last_linear_command_ * last_curvature_command_, -max_yaw_rate_, max_yaw_rate_);

    geometry_msgs::msg::Twist command;
    command.linear.x = last_linear_command_;
    command.angular.z = last_yaw_command_;
    cmd_pub_->publish(command);
  }

  void stop(
    const std::string & state, double dt = 0.0,
    const WallMeasurement * measurement = nullptr,
    double cloud_age = std::numeric_limits<double>::infinity())
  {
    if (dt <= 0.0) {
      dt = 1.0 / control_frequency_;
    }
    publishRateLimited(0.0, last_curvature_command_, dt);
    publishState(state);
    logDebug(state, measurement, cloud_age, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  void logDebug(
    const std::string & state, const WallMeasurement * measurement,
    double cloud_age, double lateral_error, double heading_error,
    double curvature, double requested_speed, double requested_yaw_rate,
    const SpeedProfile * profile = nullptr, double road_curvature = 0.0)
  {
    const rclcpp::Time current_time = now();
    if ((current_time - last_debug_log_time_).seconds() <
      1.0 / debug_log_frequency_)
    {
      return;
    }
    last_debug_log_time_ = current_time;

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "state=" << state
           << " enabled=" << (enabled_.load() ? "true" : "false")
           << " cloud_age=" << cloud_age;
    if (measurement != nullptr) {
      stream << " front_distance=" << measurement->front_distance
             << " path_clearance=" << measurement->path_clearance
             << " path_clearance_y=" << measurement->path_clearance_y
             << " left_points=" << measurement->left_point_count
             << " right_points=" << measurement->right_point_count
             << " left_valid=" << (measurement->left.valid ? "true" : "false")
             << " right_valid=" << (measurement->right.valid ? "true" : "false")
             << " left_held=" << (measurement->left_held ? "true" : "false")
             << " right_held=" << (measurement->right_held ? "true" : "false")
             << " pair_inconsistent="
             << (measurement->pair_inconsistent ? "true" : "false")
             << " left_y=" << measurement->left_y
             << " right_y=" << measurement->right_y
             << " left_heading=" << measurement->left.heading
             << " right_heading=" << measurement->right.heading
             << " left_inliers=" << measurement->left.inliers
             << " right_inliers=" << measurement->right.inliers
             << " left_rms=" << measurement->left.rms
             << " right_rms=" << measurement->right.rms;
    }
    if (profile != nullptr) {
      stream << " preview_curvature=" << profile->preview_curvature
             << " curve_speed_limit=" << profile->curve_limit
             << " clearance_speed_limit=" << profile->clearance_limit
             << " road_curvature=" << road_curvature;
    }
    stream << " lateral_error=" << lateral_error
           << " heading_error=" << heading_error
           << " curvature=" << curvature
           << " requested_speed=" << requested_speed
           << " requested_yaw_rate=" << requested_yaw_rate
           << " command_speed=" << last_linear_command_
           << " command_curvature=" << last_curvature_command_
           << " command_yaw_rate=" << last_yaw_command_;

    RCLCPP_INFO(get_logger(), "CONTROL DEBUG | %s", stream.str().c_str());
  }

  void publishState(const std::string & state)
  {
    if (state == last_state_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = state;
    state_pub_->publish(message);
    last_state_ = state;
    RCLCPP_INFO(get_logger(), "Wall follower state: %s", state.c_str());
  }

  std::string cloud_topic_;
  std::vector<std::string> accepted_cloud_frames_;
  std::string cmd_vel_topic_;
  std::string enable_topic_;
  std::string state_topic_;
  double debug_log_frequency_;
  double control_frequency_;
  double cloud_timeout_;
  int point_stride_;
  double min_x_;
  double max_x_;
  double min_z_;
  double max_z_;
  double side_min_abs_y_;
  double side_max_abs_y_;
  double front_half_width_;
  int min_wall_points_;
  double fit_residual_threshold_;
  double max_fit_rms_;
  int max_fit_points_;
  double wall_detection_hold_time_;
  double wall_lookahead_;
  double min_corridor_width_;
  double max_corridor_width_;
  double right_wall_target_distance_;
  double left_wall_target_distance_;
  double center_gain_;
  double right_wall_gain_;
  double left_wall_gain_;
  double heading_gain_;
  double right_turn_curvature_bias_;
  double turn_enter_front_distance_;
  double emergency_stop_distance_;
  double straight_speed_;
  double turn_speed_;
  double min_speed_;
  double max_lateral_acceleration_;
  double max_curvature_;
  double max_yaw_rate_;
  double max_linear_acceleration_;
  double max_linear_deceleration_;
  double max_curvature_rate_;
  double preview_wall_offset_;
  double control_latency_;
  double curvature_feedforward_gain_;
  double curvature_fit_min_span_;
  int recovery_max_attempts_;
  double recovery_reverse_distance_;
  double recovery_reverse_speed_;
  double recovery_curvature_;
  double recovery_pause_;
  double recovery_reset_distance_;
  int recovery_flip_after_attempts_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex measurement_mutex_;
  std::atomic_bool enabled_{false};
  WallMeasurement latest_measurement_;
  bool have_measurement_{false};
  LineFit last_left_wall_;
  LineFit last_right_wall_;
  double last_left_y_{0.0};
  double last_right_y_{0.0};
  rclcpp::Time last_left_wall_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_right_wall_stamp_{0, 0, RCL_ROS_TIME};
  Mode mode_{Mode::STOPPED};
  double last_linear_command_{0.0};
  double last_curvature_command_{0.0};
  double last_yaw_command_{0.0};
  std::atomic<double> commanded_curvature_{0.0};
  RecoveryPhase recovery_phase_{RecoveryPhase::NONE};
  int recovery_attempts_{0};
  double recovery_turn_sign_{1.0};
  double recovery_travel_{0.0};
  double recovery_timer_{0.0};
  double forward_since_recovery_{0.0};
  std::string last_state_;
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_debug_log_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace mapless_wall_follower

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mapless_wall_follower::MaplessWallFollower>());
  rclcpp::shutdown();
  return 0;
}
