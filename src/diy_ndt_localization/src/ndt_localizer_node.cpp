/**
 * ndt_localizer_node.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * NDT-OMP based map-matching localization node for Team Juggernauts 2026.
 *
 * WHAT THIS DOES:
 *   1. Loads GlobalMap.pcd (produced by LIO-SAM offline mapping run) at startup.
 *   2. Subscribes to /hesai/points (10 Hz raw lidar scans from Hesai QT64).
 *   3. For each scan: downsample → run NDT-OMP alignment against the map.
 *   4. If alignment fitness score is good: publish map→odom TF + /ndt_pose.
 *   5. If fitness is bad (tunnel / featureless area): hold last good TF frozen.
 *
 * WHY NDT-OMP INSTEAD OF AMCL:
 *   AMCL works on 2D floor maps (.pgm). Our course has ramps, gravel, a tunnel —
 *   a flat 2D map is not usable. NDT-OMP matches directly against the 3D point
 *   cloud GlobalMap.pcd, which LIO-SAM captures accurately with loop closure.
 *
 * OUTPUT:
 *   /ndt_pose              (nav_msgs/Odometry) — fed into EKF2 as map-frame input
 *   TF: map → odom         — consumed by Nav2 for global planning
 *   /ndt_fitness_score     (std_msgs/Float32)  — monitor in RViz during testing
 *
 * INITIAL POSE:
 *   On startup the robot is assumed to be at the origin of the map frame (start line).
 *   If the robot is placed elsewhere, publish a 2D Pose Estimate from RViz to
 *   /initialpose — the node will use it as the NDT seed.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <rclcpp/rclcpp.hpp>

// Message types
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32.hpp>

// TF publishing
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// PCL — point cloud processing
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

// NDT-OMP — OpenMP accelerated Normal Distributions Transform
#include <pclomp/ndt_omp.h>

// Standard library
#include <string>
#include <memory>
#include <mutex>

// Convenience type alias — we use PointXYZI (intensity) from Hesai
using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

class NdtLocalizerNode : public rclcpp::Node
{
public:
  NdtLocalizerNode()
  : Node("ndt_localizer_node")
  {
    // ── Load parameters from ndt_localizer.yaml ────────────────────────────
    map_path_           = declare_parameter<std::string>("map_path", "");
    ndt_resolution_     = declare_parameter<double>("ndt_resolution", 1.0);
    ndt_step_size_      = declare_parameter<double>("ndt_step_size", 0.1);
    ndt_outlier_ratio_  = declare_parameter<double>("ndt_outlier_ratio", 0.55);
    ndt_max_iterations_ = declare_parameter<int>("ndt_max_iterations", 30);
    num_threads_        = declare_parameter<int>("num_threads", 4);
    voxel_leaf_size_    = declare_parameter<double>("voxel_leaf_size", 0.2);
    min_scan_range_     = declare_parameter<double>("min_scan_range", 0.5);
    max_scan_range_     = declare_parameter<double>("max_scan_range", 50.0);
    fitness_threshold_  = declare_parameter<double>("fitness_score_threshold", 1.0);
    map_frame_          = declare_parameter<std::string>("map_frame", "map");
    odom_frame_         = declare_parameter<std::string>("odom_frame", "odom");
    base_link_frame_    = declare_parameter<std::string>("base_link_frame", "base_link");
    input_cloud_topic_  = declare_parameter<std::string>("input_cloud_topic", "/hesai/points");
    output_pose_topic_  = declare_parameter<std::string>("output_pose_topic", "/ndt_pose");
    fitness_topic_      = declare_parameter<std::string>("fitness_score_topic", "/ndt_fitness_score");
    publish_tf_         = declare_parameter<bool>("publish_tf", true);

    // ── Load GlobalMap.pcd ─────────────────────────────────────────────────
    // This is a blocking call at startup. Map must exist before the node is useful.
    map_cloud_.reset(new PointCloud());
    if (pcl::io::loadPCDFile<PointT>(map_path_, *map_cloud_) == -1) {
      RCLCPP_FATAL(get_logger(), "Failed to load map: %s", map_path_.c_str());
      RCLCPP_FATAL(get_logger(), "Run LIO-SAM offline mapping first and update map_path in config.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Map loaded: %s  (%zu points)", map_path_.c_str(), map_cloud_->size());

    // ── Configure NDT-OMP ──────────────────────────────────────────────────
    ndt_.reset(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    ndt_->setResolution(ndt_resolution_);
    ndt_->setStepSize(ndt_step_size_);
    ndt_->setOulierRatio(ndt_outlier_ratio_);
    ndt_->setMaximumIterations(ndt_max_iterations_);
    ndt_->setNumThreads(num_threads_);
    // DIRECT7 searches 7 neighbouring voxels — good balance of speed vs accuracy
    ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);

    // Set the loaded map as the registration target
    ndt_->setInputTarget(map_cloud_);
    RCLCPP_INFO(get_logger(), "NDT-OMP configured: res=%.1f step=%.2f threads=%d",
      ndt_resolution_, ndt_step_size_, num_threads_);

    // ── Initial pose — robot starts at map origin (start line) ─────────────
    // Identity matrix = robot at (0,0,0) facing +X in map frame.
    // Override via /initialpose topic (RViz 2D Pose Estimate button).
    current_pose_matrix_ = Eigen::Matrix4f::Identity();
    pose_initialized_ = true;  // start from origin; set false if you want to wait for /initialpose

    // ── TF broadcaster ─────────────────────────────────────────────────────
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ── Publishers ─────────────────────────────────────────────────────────
    pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_pose_topic_, 10);
    fitness_pub_ = create_publisher<std_msgs::msg::Float32>(fitness_topic_, 10);

    // ── Subscribers ────────────────────────────────────────────────────────
    // Incoming lidar scans from Hesai QT64 at 10 Hz
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&NdtLocalizerNode::cloudCallback, this, std::placeholders::_1));

    // Allow RViz "2D Pose Estimate" to re-seed the NDT initial guess
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10,
      std::bind(&NdtLocalizerNode::initialPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "NDT localizer ready. Listening on %s", input_cloud_topic_.c_str());
  }

private:
  // ── Callback: new lidar scan received ────────────────────────────────────
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!pose_initialized_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Waiting for initial pose. Publish a 2D Pose Estimate in RViz.");
      return;
    }

    // Convert ROS PointCloud2 → PCL PointCloud
    PointCloud::Ptr raw_cloud(new PointCloud());
    pcl::fromROSMsg(*msg, *raw_cloud);

    // ── Filter 1: range passthrough ────────────────────────────────────────
    // Remove points too close (robot body) or too far (noise)
    PointCloud::Ptr range_filtered(new PointCloud());
    for (const auto & pt : raw_cloud->points) {
      double dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
      if (dist >= min_scan_range_ && dist <= max_scan_range_) {
        range_filtered->points.push_back(pt);
      }
    }

    // ── Filter 2: voxel grid downsample ────────────────────────────────────
    // Reduces point count for faster NDT alignment
    PointCloud::Ptr filtered_cloud(new PointCloud());
    pcl::VoxelGrid<PointT> voxel_filter;
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.setInputCloud(range_filtered);
    voxel_filter.filter(*filtered_cloud);

    if (filtered_cloud->empty()) {
      RCLCPP_WARN(get_logger(), "Filtered cloud is empty — skipping this scan.");
      return;
    }

    // ── Run NDT-OMP alignment ──────────────────────────────────────────────
    // Uses the previous pose as the initial guess (warm start = faster convergence)
    std::lock_guard<std::mutex> lock(pose_mutex_);
    ndt_->setInputSource(filtered_cloud);

    PointCloud::Ptr aligned(new PointCloud());
    ndt_->align(*aligned, current_pose_matrix_);

    double fitness_score = ndt_->getFitnessScore();
    bool converged = ndt_->hasConverged();

    // Publish fitness score for monitoring — high score = bad match
    std_msgs::msg::Float32 score_msg;
    score_msg.data = static_cast<float>(fitness_score);
    fitness_pub_->publish(score_msg);

    // ── Reliability gate ───────────────────────────────────────────────────
    // If fitness is too high or NDT didn't converge, hold the last good pose.
    // This keeps map->odom TF stable through the tunnel or featureless areas.
    if (!converged || fitness_score > fitness_threshold_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "NDT unreliable: converged=%s fitness=%.3f (threshold=%.3f) — holding last pose.",
        converged ? "yes" : "no", fitness_score, fitness_threshold_);
      // Republish last known TF to prevent Nav2 from stalling
      publishTF(msg->header.stamp, current_pose_matrix_);
      return;
    }

    // Good match — update stored pose and publish
    current_pose_matrix_ = ndt_->getFinalTransformation();

    RCLCPP_DEBUG(get_logger(), "NDT OK: fitness=%.4f iterations=%d",
      fitness_score, ndt_->getFinalNumIteration());

    publishPose(msg->header.stamp, current_pose_matrix_, fitness_score);
    if (publish_tf_) {
      publishTF(msg->header.stamp, current_pose_matrix_);
    }
  }

  // ── Callback: RViz initial pose estimate ─────────────────────────────────
  // Allows re-seeding NDT if robot is placed away from map origin
  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);

    // Convert geometry_msgs Pose → Eigen 4x4 transform matrix for NDT seed
    Eigen::Quaternionf q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    Eigen::Matrix3f rot = q.toRotationMatrix();

    current_pose_matrix_ = Eigen::Matrix4f::Identity();
    current_pose_matrix_.block<3, 3>(0, 0) = rot;
    current_pose_matrix_(0, 3) = msg->pose.pose.position.x;
    current_pose_matrix_(1, 3) = msg->pose.pose.position.y;
    current_pose_matrix_(2, 3) = msg->pose.pose.position.z;

    pose_initialized_ = true;
    RCLCPP_INFO(get_logger(), "Initial pose set from RViz: x=%.2f y=%.2f",
      msg->pose.pose.position.x, msg->pose.pose.position.y);
  }

  // ── Publish /ndt_pose (Odometry) ─────────────────────────────────────────
  // This is fed into EKF2 as the map-frame position correction input
  void publishPose(
    const rclcpp::Time & stamp,
    const Eigen::Matrix4f & pose_matrix,
    double fitness_score)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = map_frame_;
    odom_msg.child_frame_id = base_link_frame_;

    // Extract translation from 4x4 matrix
    odom_msg.pose.pose.position.x = pose_matrix(0, 3);
    odom_msg.pose.pose.position.y = pose_matrix(1, 3);
    odom_msg.pose.pose.position.z = pose_matrix(2, 3);

    // Extract rotation and convert to quaternion
    Eigen::Matrix3f rot = pose_matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rot);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // Covariance scaled by fitness score — worse match = higher uncertainty for EKF
    double cov = fitness_score * 0.1;
    odom_msg.pose.covariance[0]  = cov;   // x
    odom_msg.pose.covariance[7]  = cov;   // y
    odom_msg.pose.covariance[35] = cov;   // yaw

    pose_pub_->publish(odom_msg);
  }

  // ── Publish map → odom TF ─────────────────────────────────────────────────
  // Nav2 reads this TF to know where the robot is in the global map frame.
  // The transform represents: "odom origin is at this pose in map frame"
  void publishTF(const rclcpp::Time & stamp, const Eigen::Matrix4f & pose_matrix)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = odom_frame_;

    tf_msg.transform.translation.x = pose_matrix(0, 3);
    tf_msg.transform.translation.y = pose_matrix(1, 3);
    tf_msg.transform.translation.z = pose_matrix(2, 3);

    Eigen::Matrix3f rot = pose_matrix.block<3, 3>(0, 0);
    Eigen::Quaternionf q(rot);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  // ── Member variables ──────────────────────────────────────────────────────

  // Parameters
  std::string map_path_, map_frame_, odom_frame_, base_link_frame_;
  std::string input_cloud_topic_, output_pose_topic_, fitness_topic_;
  double ndt_resolution_, ndt_step_size_, ndt_outlier_ratio_;
  double voxel_leaf_size_, min_scan_range_, max_scan_range_, fitness_threshold_;
  int ndt_max_iterations_, num_threads_;
  bool publish_tf_;

  // NDT-OMP instance and map
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt_;
  PointCloud::Ptr map_cloud_;

  // Current pose estimate (4x4 homogeneous transform matrix)
  Eigen::Matrix4f current_pose_matrix_;
  bool pose_initialized_{false};
  std::mutex pose_mutex_;  // protects current_pose_matrix_ across callbacks

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fitness_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NdtLocalizerNode>());
  rclcpp::shutdown();
  return 0;
}
