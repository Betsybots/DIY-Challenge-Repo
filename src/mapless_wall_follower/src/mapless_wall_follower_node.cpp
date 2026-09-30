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
    max_lateral_acceleration_ = declare_parameter("max_lateral_acceleration", 0.50);
    max_curvature_ = declare_parameter("max_curvature", 2.70);
    max_yaw_rate_ = declare_parameter("max_yaw_rate", 1.35);
    max_linear_acceleration_ = declare_parameter("max_linear_acceleration", 0.40);
    max_linear_deceleration_ = declare_parameter("max_linear_deceleration", 0.80);
    max_yaw_acceleration_ = declare_parameter("max_yaw_acceleration", 1.50);

    if (control_frequency_ <= 0.0 || debug_log_frequency_ <= 0.0 ||
      point_stride_ < 1 || min_wall_points_ < 2 ||
      max_fit_points_ < min_wall_points_ || min_x_ >= max_x_ || min_z_ >= max_z_ ||
      side_min_abs_y_ >= side_max_abs_y_ || wall_lookahead_ <= 0.0 ||
      min_corridor_width_ >= max_corridor_width_ ||
      right_wall_target_distance_ <= 0.0 || left_wall_target_distance_ <= 0.0 ||
      right_wall_gain_ <= 0.0 || left_wall_gain_ <= 0.0 ||
      emergency_stop_distance_ <= 0.0 ||
      emergency_stop_distance_ >= turn_enter_front_distance_ ||
      straight_speed_ <= 0.0 || turn_speed_ <= 0.0 ||
      max_curvature_ <= 0.0 || max_yaw_rate_ <= 0.0 ||
      max_linear_acceleration_ <= 0.0 || max_linear_deceleration_ <= 0.0 ||
      max_yaw_acceleration_ <= 0.0)
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
      "Expected cloud frame/axes: base_link, +x forward, +y left, +z up");
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
  };

  struct WallMeasurement
  {
    LineFit left;
    LineFit right;
    double left_y{0.0};
    double right_y{0.0};
    double front_distance{std::numeric_limits<double>::infinity()};
    std::size_t left_point_count{0};
    std::size_t right_point_count{0};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  };

  enum class Mode
  {
    STOPPED,
    CENTERING,
    RIGHT_WALL,
    LEFT_WALL
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
    return fit;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud)
  {
    if (cloud->header.frame_id != "base_link") {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Expected cloud frame base_link but received %s",
        cloud->header.frame_id.c_str());
      return;
    }

    std::vector<Point2D> left_points;
    std::vector<Point2D> right_points;
    left_points.reserve(std::min<std::size_t>(cloud->width, max_fit_points_));
    right_points.reserve(std::min<std::size_t>(cloud->width, max_fit_points_));
    double front_distance = std::numeric_limits<double>::infinity();

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
    if (measurement.left.valid && measurement.right.valid) {
      const double width = measurement.left_y - measurement.right_y;
      if (width < min_corridor_width_ || width > max_corridor_width_) {
        measurement.left.valid = false;
        measurement.right.valid = false;
      }
    }

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
      stop("DISABLED", dt, &measurement, cloud_age);
      return;
    }

    if (cloud_age > cloud_timeout_) {
      mode_ = Mode::STOPPED;
      stop("STALE_CLOUD", dt, &measurement, cloud_age);
      return;
    }
    if (measurement.front_distance <= emergency_stop_distance_) {
      mode_ = Mode::STOPPED;
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
    double requested_speed = straight_speed_;
    double lateral_error = 0.0;
    double heading_error = 0.0;
    std::string state = "CENTERING";

    if (mode_ == Mode::CENTERING) {
      lateral_error = 0.5 * (measurement.left_y + measurement.right_y);
      heading_error =
        0.5 * (measurement.left.heading + measurement.right.heading);
      curvature = center_gain_ * lateral_error + heading_gain_ * heading_error;
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
    if (std::abs(curvature) > 1e-4) {
      requested_speed = std::min(
        requested_speed,
        std::sqrt(max_lateral_acceleration_ / std::abs(curvature)));
    }
    double requested_yaw_rate = requested_speed * curvature;
    requested_yaw_rate = std::clamp(requested_yaw_rate, -max_yaw_rate_, max_yaw_rate_);

    publishRateLimited(requested_speed, requested_yaw_rate, dt);
    publishState(state);
    logDebug(
      state, &measurement, cloud_age, lateral_error, heading_error,
      curvature, requested_speed, requested_yaw_rate);
  }

  void publishRateLimited(double speed, double yaw_rate, double dt)
  {
    const double speed_delta = speed - last_linear_command_;
    const double speed_limit =
      (speed_delta >= 0.0 ? max_linear_acceleration_ : max_linear_deceleration_) * dt;
    const double yaw_limit = max_yaw_acceleration_ * dt;
    last_linear_command_ += std::clamp(speed_delta, -speed_limit, speed_limit);
    last_yaw_command_ += std::clamp(
      yaw_rate - last_yaw_command_, -yaw_limit, yaw_limit);

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
    publishRateLimited(0.0, 0.0, dt);
    publishState(state);
    logDebug(state, measurement, cloud_age, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  void logDebug(
    const std::string & state, const WallMeasurement * measurement,
    double cloud_age, double lateral_error, double heading_error,
    double curvature, double requested_speed, double requested_yaw_rate)
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
             << " left_points=" << measurement->left_point_count
             << " right_points=" << measurement->right_point_count
             << " left_valid=" << (measurement->left.valid ? "true" : "false")
             << " right_valid=" << (measurement->right.valid ? "true" : "false")
             << " left_y=" << measurement->left_y
             << " right_y=" << measurement->right_y
             << " left_heading=" << measurement->left.heading
             << " right_heading=" << measurement->right.heading
             << " left_inliers=" << measurement->left.inliers
             << " right_inliers=" << measurement->right.inliers
             << " left_rms=" << measurement->left.rms
             << " right_rms=" << measurement->right.rms;
    }
    stream << " lateral_error=" << lateral_error
           << " heading_error=" << heading_error
           << " curvature=" << curvature
           << " requested_speed=" << requested_speed
           << " requested_yaw_rate=" << requested_yaw_rate
           << " command_speed=" << last_linear_command_
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
  double max_lateral_acceleration_;
  double max_curvature_;
  double max_yaw_rate_;
  double max_linear_acceleration_;
  double max_linear_deceleration_;
  double max_yaw_acceleration_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex measurement_mutex_;
  std::atomic_bool enabled_{false};
  WallMeasurement latest_measurement_;
  bool have_measurement_{false};
  Mode mode_{Mode::STOPPED};
  double last_linear_command_{0.0};
  double last_yaw_command_{0.0};
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
