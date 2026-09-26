/*
 * Copyright (c) 2016-2020, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include <boost/chrono.hpp>
#include <boost/shared_ptr.hpp>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mcl_3dl/msg/status.hpp>
#include <mcl_3dl/srv/resize_particle.hpp>
#include <mcl_3dl/srv/load_pcd.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <mcl_3dl/chunked_kdtree.h>
#include <mcl_3dl/cloud_accum.h>
#include <mcl_3dl/filter.h>
#include <mcl_3dl/filter_vec3.h>
#include <mcl_3dl/imu_measurement_model_base.h>
#include <mcl_3dl/imu_measurement_models/imu_measurement_model_gravity.h>
#include <mcl_3dl/lidar_measurement_model_base.h>
#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_beam.h>
#include <mcl_3dl/lidar_measurement_models/lidar_measurement_model_likelihood.h>
#include <mcl_3dl/motion_prediction_model_base.h>
#include <mcl_3dl/motion_prediction_models/motion_prediction_model_differential_drive.h>
#include <mcl_3dl/nd.h>
#include <mcl_3dl/noise_generators/multivariate_noise_generator.h>
#include <mcl_3dl/parameters.h>
#include <mcl_3dl/pf.h>
#include <mcl_3dl/point_cloud_random_samplers/point_cloud_sampler_with_normal.h>
#include <mcl_3dl/point_cloud_random_samplers/point_cloud_uniform_sampler.h>
#include <mcl_3dl/point_conversion.h>
#include <mcl_3dl/point_types.h>
#include <mcl_3dl/quat.h>
#include <mcl_3dl/raycast.h>
#include <mcl_3dl/state_6dof.h>
#include <mcl_3dl/vec3.h>


namespace mcl_3dl
{
namespace
{
builtin_interfaces::msg::Time toMsg(const rclcpp::Time& time)
{
  const int64_t nanoseconds = time.nanoseconds();
  builtin_interfaces::msg::Time message;
  message.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  message.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return message;
}
}  // namespace

class MCL3dlNode : public rclcpp::Node
{
protected:
  using PointType = mcl_3dl::PointXYZIL;
  std::shared_ptr<pf::ParticleFilter<State6DOF, float, ParticleWeightedMeanQuat, std::default_random_engine>> pf_;

  class MyPointRepresentation : public pcl::PointRepresentation<PointType>
  {
    using pcl::PointRepresentation<PointType>::nr_dimensions_;

  public:
    MyPointRepresentation()
    {
      nr_dimensions_ = 3;
      trivial_ = true;
    }

    virtual void copyToFloatArray(const PointType& p, float* out) const
    {
      out[0] = p.x;
      out[1] = p.y;
      out[2] = p.z;
    }
  };
  void cbMapcloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    RCLCPP_INFO(get_logger(), "map received");
    pcl::PointCloud<PointType>::Ptr pc_tmp(new pcl::PointCloud<PointType>);
    if (!mcl_3dl::fromROSMsg(*msg, *pc_tmp))
    {
      has_map_ = false;
      return;
    }
    const rclcpp::Time map_stamp(msg->header.stamp);
    pcl_conversions::toPCL(map_stamp.nanoseconds() == 0 ? get_clock()->now() : map_stamp, pc_tmp->header.stamp);

    loadMapCloud(pc_tmp);
  }
  void cbMapcloudUpdate(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    RCLCPP_INFO(get_logger(), "map_update received");
    pcl::PointCloud<PointType>::Ptr pc_tmp(new pcl::PointCloud<PointType>);
    if (!mcl_3dl::fromROSMsg(*msg, *pc_tmp))
      return;

    pc_update_.reset(new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> ds;
    ds.setInputCloud(pc_tmp);
    ds.setLeafSize(params_.update_downsample_x_, params_.update_downsample_y_, params_.update_downsample_z_);
    ds.filter(*pc_update_);
  }

  void cbPosition(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
  {
    const double len2 =
        msg->pose.pose.orientation.x * msg->pose.pose.orientation.x +
        msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
        msg->pose.pose.orientation.z * msg->pose.pose.orientation.z +
        msg->pose.pose.orientation.w * msg->pose.pose.orientation.w;
    if (std::abs(len2 - 1.0) > 0.1)
    {
      RCLCPP_ERROR(get_logger(), "Discarded invalid initialpose. The orientation must be unit quaternion.");
      return;
    }

    geometry_msgs::msg::PoseStamped pose_in, pose;
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;
    try
    {
      const geometry_msgs::msg::TransformStamped trans = tfbuf_.lookupTransform(
          params_.frame_ids_["map"], pose_in.header.frame_id, pose_in.header.stamp, rclcpp::Duration::from_seconds(1.0));
      tf2::doTransform(pose_in, pose, trans);
    }
    catch (tf2::TransformException& e)
    {
      return;
    }
    const State6DOF mean(Vec3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                         Quat(pose.pose.orientation.x,
                              pose.pose.orientation.y,
                              pose.pose.orientation.z,
                              pose.pose.orientation.w));
    const MultivariateNoiseGenerator<float> noise_gen(mean, msg->pose.covariance);
    pf_->initUsingNoiseGenerator(noise_gen);

    pc_update_.reset();
    const auto integ_reset_func = [](State6DOF& s)
    {
      s.odom_err_integ_lin_ = Vec3();
      s.odom_err_integ_ang_ = Vec3();
    };
    pf_->predict(integ_reset_func);

    publishParticles();
  }

  void cbOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
  {
    odom_ =
        State6DOF(
            Vec3(msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 msg->pose.pose.position.z),
            Quat(msg->pose.pose.orientation.x,
                 msg->pose.pose.orientation.y,
                 msg->pose.pose.orientation.z,
                 msg->pose.pose.orientation.w));
    if (!has_odom_)
    {
      odom_prev_ = odom_;
      odom_last_ = rclcpp::Time(msg->header.stamp);
      has_odom_ = true;
      return;
    }
    const float dt = (rclcpp::Time(msg->header.stamp) - odom_last_).seconds();
    if (dt < 0.0 || dt > 5.0)
    {
      RCLCPP_WARN(get_logger(), "Detected time jump in odometry. Resetting.");
      has_odom_ = false;
      return;
    }
    else if (dt > 0.05)
    {
      motion_prediction_model_->setOdoms(odom_prev_, odom_, dt);
      const auto prediction_func = [this](State6DOF& s)
      {
        motion_prediction_model_->predict(s);
      };
      pf_->predict(prediction_func);
      odom_last_ = rclcpp::Time(msg->header.stamp);
      odom_prev_ = odom_;
    }
    if (params_.fake_imu_)
    {
      const Vec3 accel = odom_.rot_ * Vec3(0.0, 0.0, 1.0);
      std::shared_ptr<sensor_msgs::msg::Imu> imu(new sensor_msgs::msg::Imu);
      imu->header = msg->header;
      imu->linear_acceleration.x = accel.x_;
      imu->linear_acceleration.y = accel.y_;
      imu->linear_acceleration.z = accel.z_;
      imu->orientation = msg->pose.pose.orientation;
      cbImu(imu);
    }
  }
  void cbCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    status_ = mcl_3dl::msg::Status();
    status_.header.stamp = toMsg(get_clock()->now());
    status_.status = mcl_3dl::msg::Status::NORMAL;
    status_.error = mcl_3dl::msg::Status::ERROR_NORMAL;
    status_.convergence_status = mcl_3dl::msg::Status::CONVERGENCE_STATUS_NORMAL;

    if (!has_map_)
      return;

    accum_->push(
        msg->header.frame_id,
        msg,
        std::bind(&MCL3dlNode::measure, this),
        std::bind(&MCL3dlNode::accumCloud, this, std::placeholders::_1),
        std::bind(&MCL3dlNode::accumClear, this));
  }

  void accumClear()
  {
    pc_local_accum_.reset(new pcl::PointCloud<PointType>);
    pc_local_accum_->header.frame_id = params_.frame_ids_["odom"];
    pc_accum_header_.clear();
  }

  bool accumCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
  {
    sensor_msgs::msg::PointCloud2 pc_bl;
    try
    {
      const geometry_msgs::msg::TransformStamped trans = tfbuf_.lookupTransform(
          params_.frame_ids_["odom"], msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
      tf2::doTransform(*msg, pc_bl, trans);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_INFO(get_logger(), "Failed to transform pointcloud: %s", e.what());
      return false;
    }
    pcl::PointCloud<PointType>::Ptr pc_tmp(new pcl::PointCloud<PointType>);
    if (!mcl_3dl::fromROSMsg(pc_bl, *pc_tmp))
    {
      RCLCPP_INFO(get_logger(), "Failed to convert pointcloud");
      return false;
    }

    for (auto& p : pc_tmp->points)
    {
      p.label = pc_accum_header_.size();
    }
    *pc_local_accum_ += *pc_tmp;
    pc_accum_header_.push_back(msg->header);
    return true;
  }

  void measure()
  {
    cnt_measure_++;
    if (cnt_measure_ % static_cast<size_t>(params_.skip_measure_) != 0)
    {
      return;
    }

    if (pc_accum_header_.empty())
    {
      RCLCPP_ERROR(get_logger(), "MCL measure function is called without available pointcloud");
      return;
    }
    const std_msgs::msg::Header& header = pc_accum_header_.back();

    try
    {
      const geometry_msgs::msg::TransformStamped trans = tfbuf_.lookupTransform(
          params_.frame_ids_["base_link"],
          pc_local_accum_->header.frame_id,
          header.stamp, rclcpp::Duration::from_seconds(0.1));

      const Eigen::Affine3f trans_eigen =
          Eigen::Translation3f(
              trans.transform.translation.x,
              trans.transform.translation.y,
              trans.transform.translation.z) *
          Eigen::Quaternionf(
              trans.transform.rotation.w,
              trans.transform.rotation.x,
              trans.transform.rotation.y,
              trans.transform.rotation.z);
      pcl::transformPointCloud(*pc_local_accum_, *pc_local_accum_, trans_eigen);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_INFO(get_logger(), "Failed to transform pointcloud: %s", e.what());
      return;
    }
    std::vector<Vec3> origins;
    for (const auto& h : pc_accum_header_)
    {
      try
      {
        const geometry_msgs::msg::TransformStamped trans = tfbuf_.lookupTransform(
            params_.frame_ids_["base_link"], header.stamp, h.frame_id, h.stamp, params_.frame_ids_["odom"]);
        origins.push_back(Vec3(trans.transform.translation.x,
                               trans.transform.translation.y,
                               trans.transform.translation.z));
      }
      catch (tf2::TransformException& e)
      {
        RCLCPP_INFO(get_logger(), "Failed to transform pointcloud: %s", e.what());
        return;
      }
    }

    const auto ts = boost::chrono::high_resolution_clock::now();

    pcl::PointCloud<PointType>::Ptr pc_local_full(new pcl::PointCloud<PointType>);
    pcl::VoxelGrid<PointType> ds;
    ds.setInputCloud(pc_local_accum_);
    ds.setLeafSize(params_.downsample_x_, params_.downsample_y_, params_.downsample_z_);
    ds.filter(*pc_local_full);

    if (params_.use_random_sampler_with_normal_)
    {
      const State6DOF prev_mean = pf_->expectation();
      const float cov_ratio = std::max(0.1f, static_cast<float>(params_.num_particles_) / pf_->getParticleSize());
      const std::vector<State6DOF> prev_cov = pf_->covariance(1.0, cov_ratio);
      sampler_->setParticleStatistics(prev_mean, prev_cov);
    }

    std::map<std::string, pcl::PointCloud<PointType>::Ptr> pc_locals;
    for (const auto& lm : lidar_measurements_)
    {
      lm.second->setGlobalLocalizationStatus(
          params_.num_particles_, pf_->getParticleSize());
      pc_locals[lm.first] = lm.second->filter(pc_local_full, *sampler_);
    }

    if (pc_locals["likelihood"]->size() == 0)
    {
      RCLCPP_ERROR(get_logger(), "All points are filtered out. Failed to localize.");
      status_.error = mcl_3dl::msg::Status::ERROR_POINTS_NOT_FOUND;
      diag_updater_.force_update();
      return;
    }

    if (pc_locals["beam"] && pc_locals["beam"]->size() == 0)
    {
      RCLCPP_DEBUG(get_logger(), "All beam points are filtered out. Skipping beam model.");
    }

    float match_ratio_min = 1.0;
    float match_ratio_max = 0.0;
    NormalLikelihood<float> odom_error_lin_nd(params_.odom_err_integ_lin_sigma_);
    NormalLikelihood<float> odom_error_ang_nd(params_.odom_err_integ_ang_sigma_);
    const auto measure_func = [this, &pc_locals,
                               &origins,
                               &odom_error_lin_nd,
                               &match_ratio_min, &match_ratio_max](const State6DOF& s) -> float
    {
      float likelihood = 1;
      std::map<std::string, float> qualities;
      for (const auto& lm : lidar_measurements_)
      {
        const LidarMeasurementResult result = lm.second->measure(
            kdtree_, pc_locals[lm.first], origins, s);
        likelihood *= result.likelihood;
        qualities[lm.first] = result.quality;
      }
      if (match_ratio_min > qualities["likelihood"])
        match_ratio_min = qualities["likelihood"];
      if (match_ratio_max < qualities["likelihood"])
        match_ratio_max = qualities["likelihood"];

      // odometry error integration
      const float odom_error =
          odom_error_lin_nd(s.odom_err_integ_lin_.norm());
      return likelihood * odom_error;
    };
    pf_->measure(measure_func);

    if (static_cast<int>(pf_->getParticleSize()) > params_.num_particles_)
    {
      const auto bias_func = [](const State6DOF& s, float& p_bias) -> void
      {
        p_bias = 1.0;
      };
      pf_->bias(bias_func);
    }
    else
    {
      NormalLikelihood<float> nl_lin(params_.bias_var_dist_);
      NormalLikelihood<float> nl_ang(params_.bias_var_ang_);
      const auto bias_func = [this, &nl_lin, &nl_ang](const State6DOF& s, float& p_bias) -> void
      {
        const float lin_diff = (s.pos_ - state_prev_.pos_).norm();
        Vec3 axis;
        float ang_diff;
        (s.rot_ * state_prev_.rot_.inv()).getAxisAng(axis, ang_diff);
        p_bias = nl_lin(lin_diff) * nl_ang(ang_diff) + 1e-6;
        assert(std::isfinite(p_bias));
      };
      pf_->bias(bias_func);
    }
    auto e = pf_->expectationBiased();
    const auto e_max = pf_->max();

    assert(std::isfinite(e.pos_.x_));
    assert(std::isfinite(e.pos_.y_));
    assert(std::isfinite(e.pos_.z_));
    assert(std::isfinite(e.rot_.x_));
    assert(std::isfinite(e.rot_.y_));
    assert(std::isfinite(e.rot_.z_));
    assert(std::isfinite(e.rot_.w_));

    e.rot_.normalize();

    if (lidar_measurements_["beam"])
    {
      visualization_msgs::msg::MarkerArray markers;

      pcl::PointCloud<PointType>::Ptr pc_particle_beam(new pcl::PointCloud<PointType>);
      *pc_particle_beam = *pc_locals["beam"];
      e.transform(*pc_particle_beam);
      const auto beam_model = std::dynamic_pointer_cast<LidarMeasurementModelBeam>(lidar_measurements_["beam"]);
      for (const auto& p : pc_particle_beam->points)
      {
        const int beam_header_id = p.label;
        const Vec3 pos = e.pos_ + e.rot_ * origins[beam_header_id];
        const Vec3 end(p.x, p.y, p.z);
        mcl_3dl::Raycast<PointType>::CastResult point;
        const LidarMeasurementModelBeam::BeamStatus beam_status = beam_model->getBeamStatus(kdtree_, pos, end, point);

        if (beam_status != LidarMeasurementModelBeam::BeamStatus::LONG)
        {
          visualization_msgs::msg::Marker marker;
          marker.header.frame_id = params_.frame_ids_["map"];
          marker.header.stamp = header.stamp;
          marker.ns = "Ray collisions";
          marker.id = markers.markers.size();
          marker.type = visualization_msgs::msg::Marker::CUBE;
          marker.action = 0;
          marker.pose.position.x = point.pos_.x_;
          marker.pose.position.y = point.pos_.y_;
          marker.pose.position.z = point.pos_.z_;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
          marker.lifetime = rclcpp::Duration::from_seconds(0.2);
          marker.frame_locked = true;
          switch (beam_status)
          {
            case LidarMeasurementModelBeam::BeamStatus::HIT:
              marker.color.a = 0.8;
              marker.color.r = 0.0;
              marker.color.g = 1.0;
              marker.color.b = 0.0;
              break;
            case LidarMeasurementModelBeam::BeamStatus::SHORT:
              marker.color.a = 0.8;
              marker.color.r = 1.0;
              marker.color.g = 0.0;
              marker.color.b = 0.0;
              break;
            case LidarMeasurementModelBeam::BeamStatus::TOTAL_REFLECTION:
              marker.color.a = 0.2;
              marker.color.r = 0.0;
              marker.color.g = 1.0;
              marker.color.b = 0.0;
              break;
            default:
              break;
          }
          markers.markers.push_back(marker);
        }

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = params_.frame_ids_["map"];
        marker.header.stamp = header.stamp;
        marker.ns = "Rays";
        marker.id = markers.markers.size();
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = 0;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.04;
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker.frame_locked = true;
        marker.points.resize(2);
        marker.points[0].x = pos.x_;
        marker.points[0].y = pos.y_;
        marker.points[0].z = pos.z_;
        marker.points[1].x = end.x_;
        marker.points[1].y = end.y_;
        marker.points[1].z = end.z_;
        marker.colors.resize(2);

        switch (beam_status)
        {
          case LidarMeasurementModelBeam::BeamStatus::HIT:
            marker.colors[0].a = 0.5;
            marker.colors[0].r = 0.0;
            marker.colors[0].g = 1.0;
            marker.colors[0].b = 0.0;
            marker.colors[1].a = 0.8;
            marker.colors[1].r = 0.0;
            marker.colors[1].g = 1.0;
            marker.colors[1].b = 0.0;
            break;
          case LidarMeasurementModelBeam::BeamStatus::SHORT:
            marker.colors[0].a = 0.5;
            marker.colors[0].r = 1.0;
            marker.colors[0].g = 0.0;
            marker.colors[0].b = 0.0;
            marker.colors[1].a = 0.8;
            marker.colors[1].r = 1.0;
            marker.colors[1].g = 0.0;
            marker.colors[1].b = 0.0;
            break;
          case LidarMeasurementModelBeam::BeamStatus::LONG:
            marker.colors[0].a = 0.5;
            marker.colors[0].r = 0.0;
            marker.colors[0].g = 0.0;
            marker.colors[0].b = 1.0;
            marker.colors[1].a = 0.8;
            marker.colors[1].r = 0.0;
            marker.colors[1].g = 0.0;
            marker.colors[1].b = 1.0;
            break;
          case LidarMeasurementModelBeam::BeamStatus::TOTAL_REFLECTION:
            marker.colors[0].a = 0.2;
            marker.colors[0].r = 0.0;
            marker.colors[0].g = 1.0;
            marker.colors[0].b = 0.0;
            marker.colors[1].a = 0.2;
            marker.colors[1].r = 0.0;
            marker.colors[1].g = 1.0;
            marker.colors[1].b = 0.0;
            break;
        }
        markers.markers.push_back(marker);
      }

      pcl::PointCloud<PointType>::Ptr pc_particle(new pcl::PointCloud<PointType>);
      *pc_particle = *pc_locals["likelihood"];
      e.transform(*pc_particle);
      for (const auto& p : pc_particle->points)
      {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = params_.frame_ids_["map"];
        marker.header.stamp = header.stamp;
        marker.ns = "Sample points";
        marker.id = markers.markers.size();
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = 0;
        marker.pose.position.x = p.x;
        marker.pose.position.y = p.y;
        marker.pose.position.z = p.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
        marker.lifetime = rclcpp::Duration::from_seconds(0.2);
        marker.frame_locked = true;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;

        markers.markers.push_back(marker);
      }

      pub_debug_marker_->publish(markers);
    }

    Vec3 map_pos;
    Quat map_rot;
    map_pos = e.pos_ - e.rot_ * odom_.rot_.inv() * odom_.pos_;
    map_rot = e.rot_ * odom_.rot_.inv();

    bool jump = false;
    if (static_cast<int>(pf_->getParticleSize()) > params_.num_particles_)
    {
      jump = true;
      state_prev_ = e;
    }
    else
    {
      Vec3 jump_axis;
      float jump_ang;
      float jump_dist = (e.pos_ - state_prev_.pos_).norm();
      (e.rot_.inv() * state_prev_.rot_).getAxisAng(jump_axis, jump_ang);
      if (jump_dist > params_.jump_dist_ ||
          fabs(jump_ang) > params_.jump_ang_)
      {
        RCLCPP_INFO(get_logger(), "Pose jumped pos:%0.3f, ang:%0.3f", jump_dist, jump_ang);
        jump = true;

        const auto integ_reset_func = [](State6DOF& s)
        {
          s.odom_err_integ_lin_ = Vec3();
          s.odom_err_integ_ang_ = Vec3();
        };
        pf_->predict(integ_reset_func);
      }
      state_prev_ = e;
    }
    geometry_msgs::msg::TransformStamped trans;
    if (has_odom_)
      trans.header.stamp = toMsg(odom_last_ + tf_tolerance_base_ + *params_.tf_tolerance_);
    else
      trans.header.stamp = toMsg(get_clock()->now() + tf_tolerance_base_ + *params_.tf_tolerance_);
    trans.header.frame_id = params_.frame_ids_["map"];
    trans.child_frame_id = params_.frame_ids_["odom"];
    const auto rpy = map_rot.getRPY();
    if (jump)
    {
      f_ang_->set(rpy);
      f_pos_->set(map_pos);
    }
    map_rot.setRPY(f_ang_->in(rpy));
    map_pos = f_pos_->in(map_pos);
    trans.transform.translation = tf2::toMsg(tf2::Vector3(map_pos.x_, map_pos.y_, map_pos.z_));
    trans.transform.rotation = tf2::toMsg(tf2::Quaternion(map_rot.x_, map_rot.y_, map_rot.z_, map_rot.w_));

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.push_back(trans);

    e.rot_ = map_rot * odom_.rot_;
    e.pos_ = map_pos + map_rot * odom_.pos_;

    assert(std::isfinite(e.pos_.x_));
    assert(std::isfinite(e.pos_.y_));
    assert(std::isfinite(e.pos_.z_));
    assert(std::isfinite(e.rot_.x_));
    assert(std::isfinite(e.rot_.y_));
    assert(std::isfinite(e.rot_.z_));
    assert(std::isfinite(e.rot_.w_));

    trans.header.frame_id = params_.frame_ids_["map"];
    trans.child_frame_id = params_.frame_ids_["floor"];
    trans.transform.translation = tf2::toMsg(tf2::Vector3(0.0, 0.0, e.pos_.z_));
    trans.transform.rotation = tf2::toMsg(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    transforms.push_back(trans);

    if (params_.publish_tf_)
      tfb_->sendTransform(transforms);

    // Calculate covariance from sampled particles to reduce calculation cost on global localization.
    // Use the number of original particles or at least 10% of full particles.
    const auto cov = pf_->covariance(
        1.0,
        std::max(
            0.1f, static_cast<float>(params_.num_particles_) / pf_->getParticleSize()));

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = header.stamp;
    pose.header.frame_id = trans.header.frame_id;
    pose.pose.pose.position.x = e.pos_.x_;
    pose.pose.pose.position.y = e.pos_.y_;
    pose.pose.pose.position.z = e.pos_.z_;
    pose.pose.pose.orientation.x = e.rot_.x_;
    pose.pose.pose.orientation.y = e.rot_.y_;
    pose.pose.pose.orientation.z = e.rot_.z_;
    pose.pose.pose.orientation.w = e.rot_.w_;
    for (size_t i = 0; i < 36; i++)
    {
      pose.pose.covariance[i] = cov[i / 6][i % 6];
    }
    pub_pose_->publish(pose);

    if (!global_localization_fix_cnt_)
    {
      if (std::sqrt(pose.pose.covariance[0] + pose.pose.covariance[1 * 6 + 1]) > params_.std_warn_thresh_[0] ||
          std::sqrt(pose.pose.covariance[2 * 6 + 2]) > params_.std_warn_thresh_[1] ||
          std::sqrt(pose.pose.covariance[5 * 6 + 5]) > params_.std_warn_thresh_[2])
      {
        status_.convergence_status = mcl_3dl::msg::Status::CONVERGENCE_STATUS_LARGE_STD_VALUE;
      }
    }

    if (status_.convergence_status != mcl_3dl::msg::Status::CONVERGENCE_STATUS_LARGE_STD_VALUE)
    {
      Vec3 fix_axis;
      const float fix_ang = std::sqrt(
          pose.pose.covariance[3 * 6 + 3] + pose.pose.covariance[4 * 6 + 4] + pose.pose.covariance[5 * 6 + 5]);
      const float fix_dist = std::sqrt(
          pose.pose.covariance[0] + pose.pose.covariance[1 * 6 + 1] + pose.pose.covariance[2 * 6 + 2]);
      RCLCPP_DEBUG(get_logger(), "cov: lin %0.3f ang %0.3f", fix_dist, fix_ang);
      if (fix_dist < params_.fix_dist_ &&
          fabs(fix_ang) < params_.fix_ang_)
      {
        RCLCPP_DEBUG(get_logger(), "Localization fixed");
        status_.convergence_status = mcl_3dl::msg::Status::CONVERGENCE_STATUS_CONVERGED;
      }
    }

    if (params_.output_pcd_)
    {
      pcl::PointCloud<PointType>::Ptr pc_particle(new pcl::PointCloud<PointType>);
      *pc_particle = *pc_locals["likelihood"];
      e.transform(*pc_particle);
      *pc_all_accum_ += *pc_particle;
    }

    const rclcpp::Time header_stamp(header.stamp);
    if ((header_stamp > match_output_last_ + *params_.match_output_interval_ ||
       header_stamp + rclcpp::Duration::from_seconds(1.0) < match_output_last_) &&
      (pub_matched_->get_subscription_count() > 0 || pub_unmatched_->get_subscription_count() > 0))
    {
      match_output_last_ = header_stamp;

      pcl::PointCloud<pcl::PointXYZ>::Ptr pc_match(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr pc_unmatch(new pcl::PointCloud<pcl::PointXYZ>);

      pcl::PointCloud<PointType>::Ptr pc_local(new pcl::PointCloud<PointType>);
      *pc_local = *pc_local_full;

      e.transform(*pc_local);

      std::vector<int> id(1);
      std::vector<float> sqdist(1);
      const double match_dist_sq = params_.match_output_dist_ * params_.match_output_dist_;
      for (const auto& p : pc_local->points)
      {
        if (!kdtree_->radiusSearch(p, params_.unmatch_output_dist_, id, sqdist, 1))
        {
          pc_unmatch->points.emplace_back(p.x, p.y, p.z);
        }
        else if (sqdist[0] < match_dist_sq)
        {
          pc_match->points.emplace_back(p.x, p.y, p.z);
        }
      }
      if (pub_matched_->get_subscription_count() > 0)
      {
        sensor_msgs::msg::PointCloud2 pc2;
        pcl::toROSMsg(*pc_match, pc2);
        pc2.header.stamp = header.stamp;
        pc2.header.frame_id = params_.frame_ids_["map"];
        pub_matched_->publish(pc2);
      }
      if (pub_unmatched_->get_subscription_count() > 0)
      {
        sensor_msgs::msg::PointCloud2 pc2;
        pcl::toROSMsg(*pc_unmatch, pc2);
        pc2.header.stamp = header.stamp;
        pc2.header.frame_id = params_.frame_ids_["map"];
        pub_unmatched_->publish(pc2);
      }
    }

    publishParticles();

    pf_->resample(State6DOF(
        Vec3(params_.resample_var_x_,
             params_.resample_var_y_,
             params_.resample_var_z_),
        Vec3(params_.resample_var_roll_,
             params_.resample_var_pitch_,
             params_.resample_var_yaw_)));

    std::normal_distribution<float> noise(0.0, 1.0);
    const auto update_noise_func = [this, &noise](State6DOF& s)
    {
      s.noise_ll_ = noise(engine_) * params_.odom_err_lin_lin_;
      s.noise_la_ = noise(engine_) * params_.odom_err_lin_ang_;
      s.noise_aa_ = noise(engine_) * params_.odom_err_ang_ang_;
      s.noise_al_ = noise(engine_) * params_.odom_err_ang_lin_;
    };
    pf_->predict(update_noise_func);

    const auto tnow = boost::chrono::high_resolution_clock::now();
    RCLCPP_DEBUG(get_logger(), "MCL (%0.3f sec.)",
              boost::chrono::duration<float>(tnow - ts).count());
    const auto err_integ_map = e_max.rot_ * e_max.odom_err_integ_lin_;
    RCLCPP_DEBUG(get_logger(), "odom error integral lin: %0.3f, %0.3f, %0.3f, "
              "ang: %0.3f, %0.3f, %0.3f, "
              "pos: %0.3f, %0.3f, %0.3f, "
              "err on map: %0.3f, %0.3f, %0.3f",
              e_max.odom_err_integ_lin_.x_,
              e_max.odom_err_integ_lin_.y_,
              e_max.odom_err_integ_lin_.z_,
              e_max.odom_err_integ_ang_.x_,
              e_max.odom_err_integ_ang_.y_,
              e_max.odom_err_integ_ang_.z_,
              e_max.pos_.x_,
              e_max.pos_.y_,
              e_max.pos_.z_,
              err_integ_map.x_,
              err_integ_map.y_,
              err_integ_map.z_);
    RCLCPP_DEBUG(get_logger(), "match ratio min: %0.3f, max: %0.3f, pos: %0.3f, %0.3f, %0.3f",
              match_ratio_min,
              match_ratio_max,
              e.pos_.x_,
              e.pos_.y_,
              e.pos_.z_);
    if (match_ratio_max < params_.match_ratio_thresh_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Low match_ratio. Expansion resetting.");
      pf_->noise(State6DOF(
          Vec3(params_.expansion_var_x_,
               params_.expansion_var_y_,
               params_.expansion_var_z_),
          Vec3(params_.expansion_var_roll_,
               params_.expansion_var_pitch_,
               params_.expansion_var_yaw_)));
      status_.status = mcl_3dl::msg::Status::EXPANSION_RESETTING;
    }

    rclcpp::Time localized_current = get_clock()->now();
    float dt = (localized_current - localized_last_).seconds();
    if (dt > 1.0)
      dt = 1.0;
    else if (dt < 0.0)
      dt = 0.0;
    tf_tolerance_base_ = rclcpp::Duration::from_seconds(localize_rate_->in(dt));
    localized_last_ = localized_current;

    if (static_cast<int>(pf_->getParticleSize()) > params_.num_particles_)
    {
      const int reduced = pf_->getParticleSize() * 0.75;
      if (reduced > params_.num_particles_)
      {
        pf_->resizeParticle(reduced);
      }
      else
      {
        pf_->resizeParticle(params_.num_particles_);
      }
      // wait 99.7% fix (three-sigma)
      global_localization_fix_cnt_ = 1 + std::ceil(params_.lpf_step_) * 3.0;
    }
    if (global_localization_fix_cnt_)
    {
      global_localization_fix_cnt_--;
      status_.status = mcl_3dl::msg::Status::GLOBAL_LOCALIZATION;
    }

    status_.match_ratio = match_ratio_max;
    status_.particle_size = pf_->getParticleSize();
    diag_updater_.force_update();
  }
  void cbLandmark(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
  {
    NormalLikelihoodNd<float, 6> nd(
        Eigen::Matrix<double, 6, 6>(
            msg->pose.covariance.data())
            .cast<float>());
    const State6DOF measured(
        Vec3(msg->pose.pose.position.x,
             msg->pose.pose.position.y,
             msg->pose.pose.position.z),
        Quat(msg->pose.pose.orientation.x,
             msg->pose.pose.orientation.y,
             msg->pose.pose.orientation.z,
             msg->pose.pose.orientation.w));
    const auto measure_func = [this, &measured, &nd](const State6DOF& s) -> float
    {
      State6DOF diff = s - measured;
      const Vec3 rot_rpy = diff.rot_.getRPY();
      const Eigen::Matrix<float, 6, 1> diff_vec =
          (Eigen::MatrixXf(6, 1) << diff.pos_.x_,
           diff.pos_.y_,
           diff.pos_.z_,
           rot_rpy.x_,
           rot_rpy.y_,
           rot_rpy.z_)
              .finished();

      const auto n = nd(diff_vec);
      return n;
    };
    pf_->measure(measure_func);

    pf_->resample(State6DOF(
        Vec3(params_.resample_var_x_,
             params_.resample_var_y_,
             params_.resample_var_z_),
        Vec3(params_.resample_var_roll_,
             params_.resample_var_pitch_,
             params_.resample_var_yaw_)));

    publishParticles();
  }
  void cbImu(const sensor_msgs::msg::Imu::ConstSharedPtr& msg)
  {
    const Vec3 acc = f_acc_->in(Vec3(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z));

    if (!has_imu_)
    {
      f_acc_->set(Vec3());
      imu_last_ = rclcpp::Time(msg->header.stamp);
      has_imu_ = true;
      return;
    }

    const float dt = (rclcpp::Time(msg->header.stamp) - imu_last_).seconds();
    if (dt < 0.0 || dt > 5.0)
    {
      RCLCPP_WARN(get_logger(), "Detected time jump in imu. Resetting.");
      has_imu_ = false;
      return;
    }
    else if (dt > 0.05)
    {
      Vec3 acc_measure = acc.normalized();
      try
      {
        geometry_msgs::msg::Vector3 in, out;
        in.x = acc_measure.x_;
        in.y = acc_measure.y_;
        in.z = acc_measure.z_;
        // assuming imu frame is regit on base_link
        const geometry_msgs::msg::TransformStamped trans = tfbuf_.lookupTransform(
            params_.frame_ids_["base_link"], msg->header.frame_id, rclcpp::Time(0, 0, get_clock()->get_clock_type()));
        tf2::doTransform(in, out, trans);
        acc_measure = Vec3(out.x, out.y, out.z);

        imu_quat_.x_ = msg->orientation.x;
        imu_quat_.y_ = msg->orientation.y;
        imu_quat_.z_ = msg->orientation.z;
        imu_quat_.w_ = msg->orientation.w;
        Vec3 axis;
        float angle;
        imu_quat_.getAxisAng(axis, angle);
        axis = Quat(trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w) *
               axis;
        imu_quat_.setAxisAng(axis, angle);
      }
      catch (tf2::TransformException& e)
      {
        return;
      }

      imu_measurement_model_->setAccMeasure(acc_measure);
      auto imu_measure_func = [this](const State6DOF& s) -> float
      {
        return imu_measurement_model_->measure(s);
      };
      pf_->measure(imu_measure_func);

      imu_last_ = rclcpp::Time(msg->header.stamp);

      if (params_.fake_odom_)
      {
        std::shared_ptr<nav_msgs::msg::Odometry> odom(new nav_msgs::msg::Odometry);
        odom->header.frame_id = params_.frame_ids_["base_link"];
        odom->header.stamp = msg->header.stamp;
        odom->pose.pose.orientation.x = imu_quat_.x_;
        odom->pose.pose.orientation.y = imu_quat_.y_;
        odom->pose.pose.orientation.z = imu_quat_.z_;
        odom->pose.pose.orientation.w = imu_quat_.w_;
        cbOdom(odom);
      }
    }
  }
  void cbResizeParticle(
      const std::shared_ptr<mcl_3dl::srv::ResizeParticle::Request> request,
      std::shared_ptr<mcl_3dl::srv::ResizeParticle::Response> /* response */)
  {
    pf_->resizeParticle(request->size);
    publishParticles();
  }
  void cbExpansionReset(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    pf_->noise(State6DOF(
        Vec3(params_.expansion_var_x_,
             params_.expansion_var_y_,
             params_.expansion_var_z_),
        Vec3(params_.expansion_var_roll_,
             params_.expansion_var_pitch_,
             params_.expansion_var_yaw_)));
    publishParticles();
    response->success = true;
  }
  void cbGlobalLocalization(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!has_map_)
    {
      response->success = false;
      response->message = "No map received.";
      return;
    }
    pcl::PointCloud<PointType>::Ptr points(new pcl::PointCloud<PointType>);

    pcl::VoxelGrid<PointType> ds;
    ds.setInputCloud(pc_map_);
    ds.setLeafSize(
        params_.global_localization_grid_,
        params_.global_localization_grid_,
        params_.global_localization_grid_);
    ds.filter(*points);

    pcl::KdTreeFLANN<PointType>::Ptr kdtree(new pcl::KdTreeFLANN<PointType>);
    kdtree->setPointRepresentation(point_rep_);
    kdtree->setInputCloud(points);

    const auto pc_filter = [this, &kdtree](const PointType& p)
    {
      std::vector<int> id(1);
      std::vector<float> sqdist(1);
      auto p2 = p;
      p2.z += 0.01 + params_.global_localization_grid_;

      return kdtree->radiusSearch(
          p2, params_.global_localization_grid_, id, sqdist, 1);
    };
    points->erase(
        std::remove_if(points->begin(), points->end(), pc_filter),
        points->end());

    const int dir = params_.global_localization_div_yaw_;
    pf_->resizeParticle(points->size() * dir);
    auto pit = points->begin();

    const float prob = 1.0 / static_cast<float>(points->size());
    int cnt = 0;
    for (auto& particle : *pf_)
    {
      assert(pit != points->end());
      particle.probability_ = prob;
      particle.probability_bias_ = 1.0;
      particle.state_ = State6DOF(
          Vec3(pit->x, pit->y, pit->z),
          (Quat(Vec3(0.0, 0.0, 2.0 * M_PI * cnt / dir)) * imu_quat_).normalized());
      if (++cnt >= dir)
      {
        cnt = 0;
        ++pit;
      }
    }
    response->success = true;
    response->message = std::to_string(pf_->getParticleSize()) + " particles";
  }

  void publishParticles()
  {
    geometry_msgs::msg::PoseArray pa;
    if (has_odom_)
      pa.header.stamp = toMsg(odom_last_ + tf_tolerance_base_ + *params_.tf_tolerance_);
    else
      pa.header.stamp = toMsg(get_clock()->now() + tf_tolerance_base_ + *params_.tf_tolerance_);
    pa.header.frame_id = params_.frame_ids_["map"];
    for (size_t i = 0; i < pf_->getParticleSize(); i++)
    {
      geometry_msgs::msg::Pose pm;
      auto p = pf_->getParticle(i);
      p.rot_.normalize();
      pm.position.x = p.pos_.x_;
      pm.position.y = p.pos_.y_;
      pm.position.z = p.pos_.z_;
      pm.orientation.x = p.rot_.x_;
      pm.orientation.y = p.rot_.y_;
      pm.orientation.z = p.rot_.z_;
      pm.orientation.w = p.rot_.w_;
      pa.poses.push_back(pm);
    }

    pub_particle_->publish(pa);
  }

  void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (status_.error == mcl_3dl::msg::Status::ERROR_POINTS_NOT_FOUND)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Valid points does not found.");
    }
    else if (status_.convergence_status == mcl_3dl::msg::Status::CONVERGENCE_STATUS_LARGE_STD_VALUE)
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Too Large Standard Deviation.");
    }
    else
    {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
    }

    stat.add("Map Availability", has_map_ ? "true" : "false");
    stat.add("Odometry Availability", has_odom_ ? "true" : "false");
    stat.add("IMU Availability", has_imu_ ? "true" : "false");

    status_.entropy = pf_->getEntropy();
    pub_status_->publish(status_);
  }

  void loadMapCloud(const pcl::PointCloud<PointType>::Ptr& map_cloud)
  {
    pc_map_.reset(new pcl::PointCloud<PointType>);
    pc_map2_.reset();
    pc_update_.reset();
    pcl::VoxelGrid<PointType> ds;
    ds.setInputCloud(map_cloud);
    ds.setLeafSize(params_.map_downsample_x_, params_.map_downsample_y_, params_.map_downsample_z_);
    ds.filter(*pc_map_);
    pc_all_accum_.reset(new pcl::PointCloud<PointType>);
    has_map_ = true;

    accumClear();
    accum_->reset();

    RCLCPP_INFO(get_logger(), "map original: %d points", static_cast<int>(map_cloud->points.size()));
    RCLCPP_INFO(get_logger(), "map reduced: %d points", static_cast<int>(pc_map_->points.size()));

    // output map for visualization purposes:
    cbMapUpdateTimer();
  }

  void cbLoadPCD(
      const std::shared_ptr<mcl_3dl::srv::LoadPCD::Request> req,
      std::shared_ptr<mcl_3dl::srv::LoadPCD::Response> resp)
  {
    RCLCPP_INFO(get_logger(), "map received");

    pcl::PointCloud<PointType>::Ptr pc_tmp(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile<PointType>(req->pcd_path, *pc_tmp) == -1)
    {
      RCLCPP_ERROR(get_logger(), "Couldn't read file %s", req->pcd_path.c_str());
      has_map_ = false;
      resp->success = false;
      return;
    }

    pcl_conversions::toPCL(get_clock()->now(), pc_tmp->header.stamp);
    pc_tmp->header.frame_id = params_.frame_ids_["map"];

    loadMapCloud(pc_tmp);

    resp->success = true;
  }

public:
  MCL3dlNode()
    : rclcpp::Node("mcl_3dl")
    , tfbuf_(this->get_clock())
    , tfl_(tfbuf_, std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}))
    , localized_last_(static_cast<int64_t>(0), RCL_ROS_TIME)
    , match_output_last_(static_cast<int64_t>(0), RCL_ROS_TIME)
    , odom_last_(static_cast<int64_t>(0), RCL_ROS_TIME)
    , cnt_measure_(0)
    , global_localization_fix_cnt_(0)
    , point_rep_(new MyPointRepresentation)
    , diag_updater_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}))
    , engine_(seed_gen_())
  {
    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(
      std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {}));
  }
  bool configure()
  {
    if (!params_.load(*this))
    {
      RCLCPP_ERROR(get_logger(), "Failed to load parameters");
      return false;
    }

    if (!params_.fake_odom_)
    {
      int odom_queue_size;
        declare_parameter("odom_queue_size", 200);
        get_parameter("odom_queue_size", odom_queue_size);
        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
          "odom", odom_queue_size, std::bind(&MCL3dlNode::cbOdom, this, std::placeholders::_1));
    }
    if (!params_.fake_imu_)
    {
      int imu_queue_size;
        declare_parameter("imu_queue_size", 200);
        get_parameter("imu_queue_size", imu_queue_size);
        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
          "imu/data", imu_queue_size, std::bind(&MCL3dlNode::cbImu, this, std::placeholders::_1));
    }

    int cloud_queue_size;
    declare_parameter("cloud_queue_size", 100);
    get_parameter("cloud_queue_size", cloud_queue_size);
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>("cloud", cloud_queue_size, std::bind(&MCL3dlNode::cbCloud, this, std::placeholders::_1));
    sub_mapcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>("mapcloud", 1, std::bind(&MCL3dlNode::cbMapcloud, this, std::placeholders::_1));
    sub_mapcloud_update_ = create_subscription<sensor_msgs::msg::PointCloud2>("mapcloud_update", 1, std::bind(&MCL3dlNode::cbMapcloudUpdate, this, std::placeholders::_1));
    sub_position_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 1, std::bind(&MCL3dlNode::cbPosition, this, std::placeholders::_1));
    sub_landmark_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("mcl_measurement", 1, std::bind(&MCL3dlNode::cbLandmark, this, std::placeholders::_1));

    pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 5);
    pub_particle_ = create_publisher<geometry_msgs::msg::PoseArray>("particles", 1);
    pub_mapcloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("updated_map", rclcpp::QoS(1).transient_local());
    pub_debug_marker_ = create_publisher<visualization_msgs::msg::MarkerArray>("debug_marker", 1);
    pub_status_ = create_publisher<mcl_3dl::msg::Status>("status", 1);
    pub_matched_ = create_publisher<sensor_msgs::msg::PointCloud2>("matched", 2);
    pub_unmatched_ = create_publisher<sensor_msgs::msg::PointCloud2>("unmatched", 2);

    srv_particle_size_ = create_service<mcl_3dl::srv::ResizeParticle>(
      "resize_particle", std::bind(&MCL3dlNode::cbResizeParticle, this,
                      std::placeholders::_1, std::placeholders::_2));
    srv_global_localization_ = create_service<std_srvs::srv::Trigger>(
      "global_localization", std::bind(&MCL3dlNode::cbGlobalLocalization, this,
                        std::placeholders::_1, std::placeholders::_2));
    srv_expansion_reset_ = create_service<std_srvs::srv::Trigger>(
      "expansion_resetting", std::bind(&MCL3dlNode::cbExpansionReset, this,
                        std::placeholders::_1, std::placeholders::_2));
    srv_load_pcd_ = create_service<mcl_3dl::srv::LoadPCD>(
      "load_pcd", std::bind(&MCL3dlNode::cbLoadPCD, this,
                   std::placeholders::_1, std::placeholders::_2));

    point_rep_->setRescaleValues(params_.dist_weight_.data());

    pf_.reset(new pf::ParticleFilter<State6DOF,
                                     float,
                                     ParticleWeightedMeanQuat,
                                     std::default_random_engine>(params_.num_particles_));
    pf_->init(params_.initial_pose_, params_.initial_pose_std_);

    f_pos_.reset(new FilterVec3(
        Filter::FILTER_LPF,
        Vec3(params_.lpf_step_, params_.lpf_step_, params_.lpf_step_),
        Vec3()));
    f_ang_.reset(new FilterVec3(
        Filter::FILTER_LPF,
        Vec3(params_.lpf_step_, params_.lpf_step_, params_.lpf_step_),
        Vec3(), true));
    f_acc_.reset(new FilterVec3(
        Filter::FILTER_LPF,
        Vec3(params_.acc_lpf_step_, params_.acc_lpf_step_, params_.acc_lpf_step_),
        Vec3()));

    if (params_.accum_cloud_ == 0)
      accum_.reset(new CloudAccumulationLogicPassThrough());
    else
      accum_.reset(
          new CloudAccumulationLogic(params_.accum_cloud_, params_.total_accum_cloud_max_));

    imu_quat_ = Quat(0.0, 0.0, 0.0, 1.0);

    has_odom_ = has_map_ = has_imu_ = false;
    localize_rate_.reset(new Filter(Filter::FILTER_LPF, 5.0, 0.0));

    if (params_.use_random_sampler_with_normal_)
    {
      sampler_ = std::make_unique<PointCloudSamplerWithNormal<PointType>>(params_.random_sampler_with_normal_params_);
    }
    else
    {
      sampler_ = std::make_unique<PointCloudUniformSampler<PointType>>();
    }
    imu_measurement_model_ = ImuMeasurementModelBase::Ptr(new ImuMeasurementModelGravity(params_.acc_var_));
    motion_prediction_model_ = MotionPredictionModelBase::Ptr(
        new MotionPredictionModelDifferentialDrive(params_.odom_err_integ_lin_tc_,
                                                   params_.odom_err_integ_ang_tc_));

    lidar_measurements_["likelihood"] = LidarMeasurementModelBase::Ptr(
        new LidarMeasurementModelLikelihood(params_.lidar_measurement_likelihood_params_));
    lidar_measurements_["beam"] = LidarMeasurementModelBase::Ptr(
        new LidarMeasurementModelBeam(params_.lidar_measurement_beam_params_));

    float max_search_radius = 0;
    for (const auto& lm : lidar_measurements_)
    {
      max_search_radius = std::max(max_search_radius, lm.second->getMaxSearchRange());
    }

    RCLCPP_DEBUG(get_logger(), "max_search_radius: %0.3f", max_search_radius);
    kdtree_.reset(new ChunkedKdtree<PointType>(params_.map_chunk_, max_search_radius));
    kdtree_->setEpsilon(params_.map_grid_min_ / 16);
    kdtree_->setPointRepresentation(point_rep_);

    map_update_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(params_.map_update_interval_->to_chrono<std::chrono::nanoseconds>()),
      std::bind(&MCL3dlNode::cbMapUpdateTimer, this));

    diag_updater_.setHardwareID("none");
    diag_updater_.add("Status", this, &MCL3dlNode::diagnoseStatus);

    return true;
  }
  ~MCL3dlNode()
  {
    if (params_.output_pcd_ && pc_all_accum_)
    {
      std::cerr << "mcl_3dl: saving pcd file.";
      std::cerr << " (" << pc_all_accum_->points.size() << " points)" << std::endl;
      pcl::io::savePCDFileBinary("mcl_3dl.pcd", *pc_all_accum_);
    }
  }

  void cbMapUpdateTimer()
  {
    if (has_map_)
    {
      const auto ts = boost::chrono::high_resolution_clock::now();
      if (pc_update_)
      {
        if (!pc_map2_)
          pc_map2_.reset(new pcl::PointCloud<PointType>);
        *pc_map2_ = *pc_map_ + *pc_update_;
        pc_update_.reset();
        pcl_conversions::toPCL(get_clock()->now(), pc_map2_->header.stamp);
      }
      else
      {
        if (pc_map2_)
          return;
        pc_map2_ = pc_map_;
      }
      kdtree_->setInputCloud(pc_map2_);

      sensor_msgs::msg::PointCloud2 out;
      pcl::toROSMsg(*pc_map2_, out);
      pub_mapcloud_->publish(out);
      const auto tnow = boost::chrono::high_resolution_clock::now();
      RCLCPP_DEBUG(get_logger(), "Map update (%0.3f sec.)",
                boost::chrono::duration<float>(tnow - ts).count());
    }
  }

protected:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_mapcloud_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_mapcloud_update_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_position_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_landmark_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_particle_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_mapcloud_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_marker_;
  rclcpp::Publisher<mcl_3dl::msg::Status>::SharedPtr pub_status_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_matched_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_unmatched_;
  rclcpp::TimerBase::SharedPtr map_update_timer_;
  rclcpp::Service<mcl_3dl::srv::ResizeParticle>::SharedPtr srv_particle_size_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_global_localization_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_expansion_reset_;
  rclcpp::Service<mcl_3dl::srv::LoadPCD>::SharedPtr srv_load_pcd_;

  tf2_ros::Buffer tfbuf_;
  tf2_ros::TransformListener tfl_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;

  std::shared_ptr<FilterVec3> f_pos_;
  std::shared_ptr<FilterVec3> f_ang_;
  std::shared_ptr<FilterVec3> f_acc_;
  std::shared_ptr<Filter> localize_rate_;
  rclcpp::Time localized_last_;
  rclcpp::Duration tf_tolerance_base_{0, 0};

  Parameters params_;

  rclcpp::Time match_output_last_;
  rclcpp::Time odom_last_;
  bool has_map_;
  bool has_odom_;
  bool has_imu_;
  State6DOF odom_;
  State6DOF odom_prev_;
  State6DOF state_prev_;
  rclcpp::Time imu_last_;
  size_t cnt_measure_;
  Quat imu_quat_;
  size_t global_localization_fix_cnt_;
  diagnostic_updater::Updater diag_updater_;
  mcl_3dl::msg::Status status_;

  MyPointRepresentation::Ptr point_rep_;

  pcl::PointCloud<PointType>::Ptr pc_map_;
  pcl::PointCloud<PointType>::Ptr pc_map2_;
  pcl::PointCloud<PointType>::Ptr pc_update_;
  pcl::PointCloud<PointType>::Ptr pc_all_accum_;
  ChunkedKdtree<PointType>::Ptr kdtree_;

  CloudAccumulationLogicBase::Ptr accum_;
  pcl::PointCloud<PointType>::Ptr pc_local_accum_;
  std::vector<std_msgs::msg::Header> pc_accum_header_;

  std::map<
      std::string,
      LidarMeasurementModelBase::Ptr>
      lidar_measurements_;
  std::unique_ptr<PointCloudRandomSampler<PointType>> sampler_;
  ImuMeasurementModelBase::Ptr imu_measurement_model_;
  MotionPredictionModelBase::Ptr motion_prediction_model_;

  std::random_device seed_gen_;
  std::default_random_engine engine_;
};
}  // namespace mcl_3dl

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto mcl = std::make_shared<mcl_3dl::MCL3dlNode>();
  if (!mcl->configure())
  {
    return 1;
  }
  rclcpp::spin(mcl);

  return 0;
}
