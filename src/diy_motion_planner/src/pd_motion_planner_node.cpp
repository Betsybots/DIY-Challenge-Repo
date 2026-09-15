// PD path-following motion planner for the DIY Challenge robot (C++ port of
// motion_planner/pd_motion_planner_node.py).
//
// Consumes the nav_msgs/Path published by the A* planner, obtains the robot
// pose from TF, selects a look-ahead point on the path, and publishes
// velocity commands.
//
// Simulation:
//     /a_star/path -> PD -> /cmd_vel -> gz_ros_bridge -> Gazebo
//
// Hardware:
//     /a_star/path -> PD -> /cmd_vel_nav -> cmd_vel_mux -> /cmd_vel_safe

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
#include "tf2/LinearMath/Transform.h"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

class PDMotionPlanner : public rclcpp::Node
{
public:
  PDMotionPlanner()
  : Node("pd_motion_planner_node")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    kp_ = declare_parameter("kp", 2.0);
    kd_ = declare_parameter("kd", 0.1);
    step_size_ = declare_parameter("step_size", 0.2);
    max_linear_velocity_ = declare_parameter("max_linear_velocity", 0.3);
    max_angular_velocity_ = declare_parameter("max_angular_velocity", 1.0);
    path_topic_ = declare_parameter("path_topic", std::string("/a_star/path"));
    cmd_vel_topic_ = declare_parameter("cmd_vel_topic", std::string("/cmd_vel"));
    odom_frame_ = declare_parameter("odom_frame", std::string("odom"));
    base_frame_ = declare_parameter("base_frame", std::string("base_link"));
    goal_tolerance_ = declare_parameter("goal_tolerance", 0.15);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 10,
      std::bind(&PDMotionPlanner::pathCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    next_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);
    lookahead_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/pd/lookahead_marker", 10);
    goal_reached_pub_ = create_publisher<std_msgs::msg::Bool>("/pd/goal_reached", 10);

    last_cycle_time_ = get_clock()->now();

    timer_ = create_wall_timer(100ms, std::bind(&PDMotionPlanner::controlLoop, this));

    RCLCPP_INFO(get_logger(), "PD controller ready: %s -> %s",
      path_topic_.c_str(), cmd_vel_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Frames: %s -> %s", odom_frame_.c_str(), base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "Goal tolerance: %.2f m", goal_tolerance_);
  }

private:
  void pathCallback(const nav_msgs::msg::Path::SharedPtr path)
  {
    global_plan_ = path;

    prev_linear_error_ = 0.0;
    prev_angular_error_ = 0.0;
    last_cycle_time_ = get_clock()->now();

    RCLCPP_INFO(get_logger(), "Received path with %zu poses in frame \"%s\"",
      path->poses.size(), path->header.frame_id.c_str());
  }

  void controlLoop()
  {
    if (!global_plan_ || global_plan_->poses.empty()) {
      return;
    }

    geometry_msgs::msg::TransformStamped robot_pose_transform;
    try {
      robot_pose_transform = tf_buffer_->lookupTransform(
        odom_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN(get_logger(), "Could not transform from %s to %s: %s",
        odom_frame_.c_str(), base_frame_.c_str(), exception.what());
      return;
    }

    const std::string target_frame = robot_pose_transform.header.frame_id;

    if (!transformPlan(target_frame)) {
      RCLCPP_ERROR(get_logger(), "Unable to transform plan into %s", target_frame.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped robot_pose;
    robot_pose.header = robot_pose_transform.header;
    robot_pose.pose.position.x = robot_pose_transform.transform.translation.x;
    robot_pose.pose.position.y = robot_pose_transform.transform.translation.y;
    robot_pose.pose.position.z = robot_pose_transform.transform.translation.z;
    robot_pose.pose.orientation = robot_pose_transform.transform.rotation;

    const auto & goal_pose = global_plan_->poses.back();
    const double goal_dx = goal_pose.pose.position.x - robot_pose.pose.position.x;
    const double goal_dy = goal_pose.pose.position.y - robot_pose.pose.position.y;
    const double goal_distance = std::hypot(goal_dx, goal_dy);

    RCLCPP_INFO(get_logger(), "Distance to goal: %.3f m", goal_distance);

    if (goal_distance <= goal_tolerance_) {
      RCLCPP_INFO(get_logger(), "Goal reached! Distance = %.3f m", goal_distance);

      std_msgs::msg::Bool reached;
      reached.data = true;
      goal_reached_pub_->publish(reached);

      stopRobot();

      global_plan_.reset();
      prev_linear_error_ = 0.0;
      prev_angular_error_ = 0.0;

      return;
    }

    const geometry_msgs::msg::PoseStamped next_pose = getNextPose(robot_pose);
    next_pose_pub_->publish(next_pose);
    publishLookaheadMarker(next_pose);

    tf2::Transform robot_tf;
    tf2::fromMsg(robot_pose.pose, robot_tf);

    tf2::Transform next_pose_tf;
    tf2::fromMsg(next_pose.pose, next_pose_tf);

    // Equivalent to: next_pose_robot_tf = robot_tf.inverse() * next_pose_tf
    const tf2::Transform next_pose_robot_tf = robot_tf.inverse() * next_pose_tf;

    const double linear_error = next_pose_robot_tf.getOrigin().x();
    const double angular_error = next_pose_robot_tf.getOrigin().y();

    const rclcpp::Time current_time = get_clock()->now();
    double dt = (current_time - last_cycle_time_).seconds();
    last_cycle_time_ = current_time;
    if (dt <= 1e-6) {
      dt = 0.1;
    }

    const double linear_error_derivative = (linear_error - prev_linear_error_) / dt;
    const double angular_error_derivative = (angular_error - prev_angular_error_) / dt;

    double linear_velocity = kp_ * linear_error + kd_ * linear_error_derivative;
    double angular_velocity = kp_ * angular_error + kd_ * angular_error_derivative;

    linear_velocity = std::clamp(linear_velocity, -max_linear_velocity_, max_linear_velocity_);
    angular_velocity = std::clamp(
      angular_velocity, -max_angular_velocity_, max_angular_velocity_);

    geometry_msgs::msg::Twist command;
    command.linear.x = linear_velocity;
    command.angular.z = angular_velocity;
    cmd_pub_->publish(command);

    prev_linear_error_ = linear_error;
    prev_angular_error_ = angular_error;
  }

  // Look-ahead selection:
  //   1. Find the path point closest to the robot.
  //   2. Search forward from that path index.
  //   3. Select the first point farther than step_size.
  //   4. If no point satisfies that condition, use the final goal.
  geometry_msgs::msg::PoseStamped getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    const auto & poses = global_plan_->poses;

    std::size_t closest_index = 0;
    double minimum_distance_squared = std::numeric_limits<double>::infinity();

    for (std::size_t index = 0; index < poses.size(); ++index) {
      const double dx = poses[index].pose.position.x - robot_pose.pose.position.x;
      const double dy = poses[index].pose.position.y - robot_pose.pose.position.y;
      const double distance_squared = dx * dx + dy * dy;

      if (distance_squared < minimum_distance_squared) {
        minimum_distance_squared = distance_squared;
        closest_index = index;
      }
    }

    geometry_msgs::msg::PoseStamped next_pose = poses.back();

    for (std::size_t index = closest_index; index < poses.size(); ++index) {
      const double dx = poses[index].pose.position.x - robot_pose.pose.position.x;
      const double dy = poses[index].pose.position.y - robot_pose.pose.position.y;

      if (std::hypot(dx, dy) > step_size_) {
        next_pose = poses[index];
        break;
      }
    }

    return next_pose;
  }

  bool transformPlan(const std::string & frame)
  {
    if (global_plan_->header.frame_id == frame) {
      return true;
    }

    const std::string source_frame = global_plan_->header.frame_id;

    geometry_msgs::msg::TransformStamped transform;
    try {
      // target_frame <- source_frame
      transform = tf_buffer_->lookupTransform(frame, source_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_ERROR(get_logger(), "Couldn't transform plan from %s to %s: %s",
        source_frame.c_str(), frame.c_str(), exception.what());
      return false;
    }

    for (auto & pose : global_plan_->poses) {
      tf2::doTransform(pose, pose, transform);
      pose.header.frame_id = frame;
    }

    global_plan_->header.frame_id = frame;

    return true;
  }

  void publishLookaheadMarker(const geometry_msgs::msg::PoseStamped & next_pose)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = next_pose.header.frame_id;
    marker.header.stamp = get_clock()->now();
    marker.ns = "lookahead";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = next_pose.pose.position;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    lookahead_marker_pub_->publish(marker);
  }

  void stopRobot()
  {
    geometry_msgs::msg::Twist stop_command;
    stop_command.linear.x = 0.0;
    stop_command.angular.z = 0.0;
    cmd_pub_->publish(stop_command);
  }

  // Parameters
  double kp_;
  double kd_;
  double step_size_;
  double max_linear_velocity_;
  double max_angular_velocity_;
  double goal_tolerance_;
  std::string path_topic_;
  std::string cmd_vel_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Controller state
  nav_msgs::msg::Path::SharedPtr global_plan_;
  double prev_linear_error_ {0.0};
  double prev_angular_error_ {0.0};
  rclcpp::Time last_cycle_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PDMotionPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
