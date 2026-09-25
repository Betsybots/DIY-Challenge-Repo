#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <queue>
#include <filesystem>
#include "pgos/commons.h"
#include "pgos/simple_pgo.h"
#include "slam_interfaces/srv/save_maps.hpp"
#include <pcl/io/io.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/cloud_registered_body";
    std::string odom_topic = "/Odometry";
    std::string imu_topic = "/imu/data";
    std::string map_frame = "map";
    std::string local_frame = "odom";
    bool reject_imu_shocks = true;
    double gravity_magnitude = 9.81;
    double shock_accel_deviation_threshold = 4.0;
    double shock_cooldown = 0.3;
    // Ported from map_localizer's own fix for the same problem (see its
    // CHANGES_REPORT.md, "TF smoothing (no more teleport on recovery)"):
    // m_pgo->offsetR()/offsetT() can shift instantly whenever
    // smoothAndUpdate() reoptimizes (every new keyframe, and especially
    // right after a loop-closure factor is added) -- broadcasting that
    // directly is what showed up as map->odom TF jumps. <= 0 restores
    // instant-snap (REP-105-compliant, jump-tolerant) behavior.
    double offset_smoothing_time_constant = 0.5;
    // Republish /loop_pgo/corrected_map every N *added* keyframes even with
    // no loop closure, so RViz shows the map building up incrementally
    // instead of only ever updating at (rare, and previously invisible)
    // closure events. 0 disables this and falls back to on-closure-only.
    int corrected_map_keyframe_interval = 20;
};

struct NodeState
{
    std::mutex message_mutex;
    std::queue<CloudWithPose> cloud_buffer;
    double last_message_time = 0.0;
    double reject_clouds_until = 0.0;
    // Smoothed map->odom offset actually broadcast over TF -- see
    // advanceSmoothedOffset(). m_pgo->offsetR()/offsetT() itself is left
    // untouched (still the raw optimized value everything else uses).
    M3D last_offset_r = M3D::Identity();
    V3D last_offset_t = V3D::Zero();
    rclcpp::Time last_smooth_time;
};

class PGONode : public rclcpp::Node
{
public:
    PGONode() : Node("pgo_node")
    {
        RCLCPP_INFO(this->get_logger(), "PGO node started");
        loadParameters();
        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile());
        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            m_node_config.imu_topic, rclcpp::SensorDataQoS(),
            std::bind(&PGONode::imuCB, this, std::placeholders::_1));
        m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/loop_pgo/loop_markers", 10000);
        // Live view of the ACTUAL, currently-optimized map (every keyframe's
        // stored body cloud re-transformed by its latest post-ISAM2 global
        // pose) -- republished whenever a loop closure updates the graph, so
        // the real alignment at the seam can be watched directly in RViz
        // instead of inferred from ICP fitness or requiring a save_maps call.
        // transient_local so a late-subscribing RViz still gets the last one.
        m_corrected_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/loop_pgo/corrected_map", rclcpp::QoS(1).reliable().transient_local());
        // Live "what is loop closure comparing right now" view: target submap
        // (intensity=0) + source submap aligned by the best ICP guess found
        // so far (intensity=100), republished on EVERY searchForLoopPairs()
        // call that reaches ICP -- whether that candidate is ultimately
        // accepted or rejected by a later gate. Unlike /loop_pgo/corrected_map
        // (only ever published after a closure is actually accepted, which is
        // why it sat publishing nothing while closures never fired), this is
        // meant for watching proximity/loop-closure detection happen live in
        // RViz, including rejected attempts, while tuning thresholds.
        m_loop_candidate_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/loop_pgo/loop_candidate_cloud", rclcpp::QoS(1).reliable().transient_local());
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_timer = this->create_wall_timer(50ms, std::bind(&PGONode::timerCB, this));
        m_save_map_srv = this->create_service<slam_interfaces::srv::SaveMaps>("/loop_pgo/save_maps", std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2));
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());
        m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_node_config.odom_topic = config["odom_topic"].as<std::string>();
        m_node_config.imu_topic = config["imu_topic"].as<std::string>("/imu/data");
        m_node_config.map_frame = config["map_frame"].as<std::string>();
        m_node_config.local_frame = config["local_frame"].as<std::string>();
        m_node_config.reject_imu_shocks = config["reject_imu_shocks"].as<bool>(true);
        m_node_config.gravity_magnitude = config["gravity_magnitude"].as<double>(9.81);
        m_node_config.shock_accel_deviation_threshold = config["shock_accel_deviation_threshold"].as<double>(4.0);
        m_node_config.shock_cooldown = config["shock_cooldown"].as<double>(0.3);
        m_node_config.offset_smoothing_time_constant = config["offset_smoothing_time_constant"] ?
            config["offset_smoothing_time_constant"].as<double>() :
            m_node_config.offset_smoothing_time_constant;
        m_node_config.corrected_map_keyframe_interval = config["corrected_map_keyframe_interval"].as<int>(20);

        m_pgo_config.key_pose_delta_deg = config["key_pose_delta_deg"].as<double>();
        m_pgo_config.key_pose_delta_trans = config["key_pose_delta_trans"].as<double>();
        m_pgo_config.loop_search_radius = config["loop_search_radius"].as<double>();
        m_pgo_config.loop_score_tresh = config["loop_score_tresh"].as<double>();
        m_pgo_config.num_exclude_recent = config["num_exclude_recent"].as<int>(50);
        m_pgo_config.sc_dist_thres = config["sc_dist_thres"].as<double>(0.13);
        m_pgo_config.loop_submap_half_range = config["loop_submap_half_range"].as<int>();
        m_pgo_config.submap_resolution = config["submap_resolution"].as<double>();
        m_pgo_config.min_loop_detect_duration = config["min_loop_detect_duration"].as<double>();
        m_pgo_config.loop_consistency_count = config["loop_consistency_count"].as<int>(3);
        m_pgo_config.odom_trans_noise_per_meter = config["odom_trans_noise_per_meter"].as<double>(0.05);
        m_pgo_config.odom_rot_noise_per_rad = config["odom_rot_noise_per_rad"].as<double>(0.05);
    }

    void imuCB(const sensor_msgs::msg::Imu::ConstSharedPtr &imu_msg)
    {
        if (!m_node_config.reject_imu_shocks)
            return;

        const auto &accel = imu_msg->linear_acceleration;
        const double accel_norm = std::sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
        if (std::abs(accel_norm - m_node_config.gravity_magnitude) <= m_node_config.shock_accel_deviation_threshold)
            return;

        const double stamp = rclcpp::Time(imu_msg->header.stamp).seconds();
        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        m_state.reject_clouds_until = std::max(m_state.reject_clouds_until, stamp + m_node_config.shock_cooldown);
    }

    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {

        std::lock_guard<std::mutex> lock(m_state.message_mutex);
        const double cloud_time = rclcpp::Time(cloud_msg->header.stamp).seconds();
        if (m_node_config.reject_imu_shocks && cloud_time <= m_state.reject_clouds_until)
            return;

        CloudWithPose cp;
        cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);
        if (cp.pose.second < m_state.last_message_time)
        {
            RCLCPP_WARN(this->get_logger(), "Received out of order message");
            return;
        }
        m_state.last_message_time = cp.pose.second;

        cp.pose.r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                       odom_msg->pose.pose.orientation.x,
                                       odom_msg->pose.pose.orientation.y,
                                       odom_msg->pose.pose.orientation.z)
                        .toRotationMatrix();
        cp.pose.t = V3D(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
        cp.cloud = CloudType::Ptr(new CloudType);
        pcl::fromROSMsg(*cloud_msg, *cp.cloud);
        m_state.cloud_buffer.push(cp);
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_node_config.map_frame;
        transformStamped.child_frame_id = m_node_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    void publishLoopMarkers(builtin_interfaces::msg::Time &time)
    {
        if (m_loop_marker_pub->get_subscription_count() == 0)
            return;
        if (m_pgo->historyPairs().size() == 0)
            return;

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker nodes_marker;
        visualization_msgs::msg::Marker edges_marker;
        nodes_marker.header.frame_id = m_node_config.map_frame;
        nodes_marker.header.stamp = time;
        nodes_marker.ns = "pgo_nodes";
        nodes_marker.id = 0;
        nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.3;
        nodes_marker.scale.y = 0.3;
        nodes_marker.scale.z = 0.3;
        nodes_marker.color.r = 1.0;
        nodes_marker.color.g = 0.8;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;

        edges_marker.header.frame_id = m_node_config.map_frame;
        edges_marker.header.stamp = time;
        edges_marker.ns = "pgo_edges";
        edges_marker.id = 1;
        edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.1;
        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.0;
        edges_marker.color.a = 1.0;

        std::vector<KeyPoseWithCloud> &poses = m_pgo->keyPoses();
        std::vector<std::pair<size_t, size_t>> &pairs = m_pgo->historyPairs();
        for (size_t i = 0; i < pairs.size(); i++)
        {
            size_t i1 = pairs[i].first;
            size_t i2 = pairs[i].second;
            geometry_msgs::msg::Point p1, p2;
            p1.x = poses[i1].t_global.x();
            p1.y = poses[i1].t_global.y();
            p1.z = poses[i1].t_global.z();

            p2.x = poses[i2].t_global.x();
            p2.y = poses[i2].t_global.y();
            p2.z = poses[i2].t_global.z();

            nodes_marker.points.push_back(p1);
            nodes_marker.points.push_back(p2);
            edges_marker.points.push_back(p1);
            edges_marker.points.push_back(p2);
        }

        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);
        m_loop_marker_pub->publish(marker_array);
    }

    // Rebuilds and republishes the FULL map from every keyframe's stored
    // body_cloud, transformed by its CURRENT (post-optimization) global
    // pose. Reuses getSubMap() with a half_range covering the whole
    // trajectory -- no separate "full map" logic needed in SimplePGO. Only
    // called right after a loop closure actually updated the graph (see
    // timerCB()), not every keyframe, since it's O(all points so far) and
    // only meaningful to re-publish when something changed.
    void publishCorrectedMap(builtin_interfaces::msg::Time &time)
    {
        // Deliberately NOT gated on get_subscription_count(): this is only
        // called on an accepted loop closure (rare), and transient_local's
        // whole point is that a RViz added AFTER that moment still gets the
        // last one -- which requires publish() to have actually run while
        // no one was listening yet.
        if (m_pgo->keyPoses().empty())
            return;

        CloudType::Ptr merged = m_pgo->getSubMap(
            static_cast<int>(m_pgo->keyPoses().size()) - 1,
            static_cast<int>(m_pgo->keyPoses().size()),
            m_pgo_config.submap_resolution);

        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*merged, cloud_msg);
        cloud_msg.header.frame_id = m_node_config.map_frame;
        cloud_msg.header.stamp = time;
        m_corrected_map_pub->publish(cloud_msg);
    }

    // See m_loop_candidate_pub's construction comment -- published every
    // cycle a candidate reached ICP, accepted or not.
    void publishLoopCandidateCloud(builtin_interfaces::msg::Time &time)
    {
        if (!m_pgo->hasCandidateCloudsThisCycle())
            return;
        if (m_loop_candidate_pub->get_subscription_count() > 0)
        {
            CloudType::Ptr merged(new CloudType);
            *merged += *m_pgo->candidateTargetCloud();
            *merged += *m_pgo->candidateSourceCloud();
            sensor_msgs::msg::PointCloud2 cloud_msg;
            pcl::toROSMsg(*merged, cloud_msg);
            cloud_msg.header.frame_id = m_node_config.map_frame;
            cloud_msg.header.stamp = time;
            m_loop_candidate_pub->publish(cloud_msg);
        }
        m_pgo->clearCandidateCloudsFlag();
    }

    // Blends m_state.last_offset_r/t toward the current m_pgo->offsetR()/
    // offsetT() (the raw, possibly just-jumped optimized value) instead of
    // snapping to it in one TF tick. Same exponential-smoothing math as
    // map_localizer's advanceSmoothedOffset().
    void advanceSmoothedOffset()
    {
        const rclcpp::Time now = this->now();
        if (m_state.last_smooth_time.nanoseconds() == 0)
        {
            m_state.last_smooth_time = now;
            m_state.last_offset_r = m_pgo->offsetR();
            m_state.last_offset_t = m_pgo->offsetT();
            return;
        }

        const double dt = (now - m_state.last_smooth_time).seconds();
        m_state.last_smooth_time = now;
        if (dt <= 0.0 || dt > 1.0)
            return;

        if (m_node_config.offset_smoothing_time_constant <= 0.0)
        {
            m_state.last_offset_r = m_pgo->offsetR();
            m_state.last_offset_t = m_pgo->offsetT();
            return;
        }

        const double alpha = 1.0 - std::exp(-dt / m_node_config.offset_smoothing_time_constant);
        m_state.last_offset_t = (1.0 - alpha) * m_state.last_offset_t + alpha * m_pgo->offsetT();
        Eigen::Quaterniond current(m_state.last_offset_r);
        Eigen::Quaterniond target(m_pgo->offsetR());
        current.normalize();
        target.normalize();
        m_state.last_offset_r = current.slerp(alpha, target).toRotationMatrix();
    }

    void timerCB()
    {
        // Was `std::lock_guard<std::mutex>(m_state.message_mutex);` (no
        // variable name) -- that constructs and immediately destroys an
        // unnamed temporary, locking and unlocking on that one statement
        // before the pop loop below ever runs. It protected nothing. Harmless
        // today only because main() spins this node single-threaded with no
        // callback groups (syncCB/timerCB never actually run concurrently),
        // but a landmine if that ever changes -- map_localizer already made
        // exactly that jump to a MultiThreadedExecutor. Also extended the
        // lock to cover the buffer read below, which was unprotected too.
        CloudWithPose cp;
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            if (m_state.cloud_buffer.empty())
                return;
            cp = m_state.cloud_buffer.front();
            while (!m_state.cloud_buffer.empty())
            {
                m_state.cloud_buffer.pop();
            }
        }
        builtin_interfaces::msg::Time cur_time;
        cur_time.sec = cp.pose.sec;
        cur_time.nanosec = cp.pose.nsec;
        if (!m_pgo->addKeyPose(cp))
        {

            advanceSmoothedOffset();
            sendBroadCastTF(cur_time);
            return;
        }

        m_pgo->searchForLoopPairs();

        // Capture BEFORE smoothAndUpdate() -- it consumes/clears the
        // pending pairs as part of adding them to the graph, so hasLoop()
        // would always read false if checked afterward.
        const bool had_loop_this_cycle = m_pgo->hasLoop();

        m_pgo->smoothAndUpdate();

        advanceSmoothedOffset();

        sendBroadCastTF(cur_time);

        publishLoopMarkers(cur_time);

        publishLoopCandidateCloud(cur_time);

        // On-closure publish always fires (rare, and the one case that must
        // never be missed); the periodic one is just for incremental
        // feedback between closures, so skip it on a cycle already covered
        // by the former to avoid rebuilding the full map twice in one tick.
        const size_t n = m_pgo->keyPoses().size();
        const bool periodic_due = m_node_config.corrected_map_keyframe_interval > 0 &&
                                   n % static_cast<size_t>(m_node_config.corrected_map_keyframe_interval) == 0;
        if (had_loop_this_cycle || periodic_due)
            publishCorrectedMap(cur_time);
    }

    void saveMapsCB(const std::shared_ptr<slam_interfaces::srv::SaveMaps::Request> request, std::shared_ptr<slam_interfaces::srv::SaveMaps::Response> response)
    {
        if (!std::filesystem::exists(request->file_path))
        {
            response->success = false;
            response->message = request->file_path + " IS NOT EXISTS!";
            return;
        }

        if (m_pgo->keyPoses().size() == 0)
        {
            response->success = false;
            response->message = "NO POSES!";
            return;
        }

        std::filesystem::path p_dir(request->file_path);
        std::filesystem::path patches_dir = p_dir / "patches";
        std::filesystem::path poses_txt_path = p_dir / "poses.txt";
        std::filesystem::path map_path = p_dir / "map.pcd";

        if (request->save_patches)
        {
            if (std::filesystem::exists(patches_dir))
            {
                std::filesystem::remove_all(patches_dir);
            }

            std::filesystem::create_directories(patches_dir);

            if (std::filesystem::exists(poses_txt_path))
            {
                std::filesystem::remove(poses_txt_path);
            }
            RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
        }
        RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

        std::ofstream txt_file(poses_txt_path);

        CloudType::Ptr ret(new CloudType);
        for (size_t i = 0; i < m_pgo->keyPoses().size(); i++)
        {

            CloudType::Ptr body_cloud = m_pgo->keyPoses()[i].body_cloud;
            if (request->save_patches)
            {
                std::string patch_name = std::to_string(i) + ".pcd";
                std::filesystem::path patch_path = patches_dir / patch_name;
                pcl::io::savePCDFileBinary(patch_path.string(), *body_cloud);
                Eigen::Quaterniond q(m_pgo->keyPoses()[i].r_global);
                V3D t = m_pgo->keyPoses()[i].t_global;
                txt_file << patch_name << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
            }
            CloudType::Ptr world_cloud(new CloudType);
            pcl::transformPointCloud(*body_cloud, *world_cloud, m_pgo->keyPoses()[i].t_global, Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));
            *ret += *world_cloud;
        }
        txt_file.close();
        pcl::io::savePCDFileBinary(map_path.string(), *ret);
        response->success = true;
        response->message = "SAVE SUCCESS!";
    }

private:
    NodeConfig m_node_config;
    Config m_pgo_config;
    NodeState m_state;
    std::shared_ptr<SimplePGO> m_pgo;
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_corrected_map_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_loop_candidate_pub;
    rclcpp::Service<slam_interfaces::srv::SaveMaps>::SharedPtr m_save_map_srv;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PGONode>());
    rclcpp::shutdown();
    return 0;
}