#include <queue>
#include <mutex>
#include <filesystem>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "slam_interfaces/srv/relocalize.hpp"
#include "slam_interfaces/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/cloud_registered_body";
    std::string odom_topic = "/Odometry";
    std::string map_frame = "map";
    std::string local_frame = "odom";
    double update_hz = 1.0;
    // Optional: PCD map to auto-load at startup so /relocalize isn't required
    // just to start localizing. Empty means no map is loaded until a
    // /relocalize service call supplies one.
    std::string map_path = "";
    // Optional: fixed start pose (map frame) applied once at startup after
    // map_path loads, for robots that always launch from the same physical
    // spot. Only used if start_x is present in the yaml; otherwise the first
    // alignment falls back to assuming odom origin == map origin, or waits
    // for /initialpose or /relocalize.
    bool has_start_pose = false;
    double start_x = 0.0;
    double start_y = 0.0;
    double start_z = 0.0;
    double start_yaw = 0.0;
    double start_roll = 0.0;
    double start_pitch = 0.0;
    // Reject an ICP realignment if it moves map->odom further than this from
    // the last accepted offset (a good VGICP fitness score does not rule out
    // a well-scoring wrong local minimum in corridors/flat walls/symmetric
    // rooms, especially while the robot sits still between goals).
    double max_offset_jump_dist = 0.5;
    double max_offset_jump_yaw = 0.3;
    // AMCL-style transform_tolerance: the broadcast map->odom TF is stamped
    // this far ahead of the sensor capture time, so a lookup at "now()"
    // (always a little later, due to processing/network latency) isn't
    // rejected as stale even though we only actually recompute the offset
    // at update_hz.
    double transform_tolerance = 0.1;
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_received = false;
    bool service_received = false;
    bool localize_success = false;
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
    bool has_aligned_once = false;
    // -1 = no successful alignment yet; otherwise the refine-stage VGICP
    // fitness score from the last accepted offset, used as a rough proxy
    // for /amcl_pose covariance.
    double last_fitness_score = -1.0;
    int consecutive_align_failures = 0;
};

class LocalizerNode : public rclcpp::Node
{
public:
    LocalizerNode() : Node("map_localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Map Localizer Node Started");
        loadParameters();
        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_config.cloud_topic, qos.get_rmw_qos_profile());
        m_odom_sub.subscribe(this, m_config.odom_topic, qos.get_rmw_qos_profile());

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10), m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(std::bind(&LocalizerNode::syncCB, this, std::placeholders::_1, std::placeholders::_2));
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);

        if (!m_config.map_path.empty())
        {
            if (std::filesystem::exists(m_config.map_path) && m_localizer->loadMap(m_config.map_path))
            {
                m_map_loaded = true;
                RCLCPP_INFO(this->get_logger(), "AUTO-LOADED MAP: %s", m_config.map_path.c_str());
                if (m_config.has_start_pose)
                {
                    applyInitialGuess(m_config.start_x, m_config.start_y, m_config.start_z,
                                      m_config.start_yaw, m_config.start_roll, m_config.start_pitch);
                    RCLCPP_INFO(this->get_logger(), "APPLIED CONFIGURED START POSE: x=%.2f y=%.2f z=%.2f yaw=%.2f",
                                m_config.start_x, m_config.start_y, m_config.start_z, m_config.start_yaw);
                }
            }
            else
                RCLCPP_WARN(this->get_logger(), "FAILED TO AUTO-LOAD MAP: %s (call /relocalize to load one manually)", m_config.map_path.c_str());
        }

        m_reloc_srv = this->create_service<slam_interfaces::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<slam_interfaces::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        // RViz "2D Pose Estimate" and other operator-driven re-seeding, reusing whatever map is already loaded.
        m_initialpose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10, std::bind(&LocalizerNode::initialPoseCB, this, std::placeholders::_1));

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);
        // Absolute (leading "/") so it lands on /amcl_pose regardless of node namespace, matching Nav2's AMCL convention.
        m_pose_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10);

        // ICP realignment is expensive; only rerun it at update_hz. TF/pose are
        // instead broadcast from syncCB at sensor rate (see there) so map->odom
        // always carries a fresh, strictly-increasing timestamp instead of the
        // same stamp being re-sent to tf2 between recomputes (which is what
        // makes tf2 log "old"/"repeated" data warnings for that edge).
        const double recompute_hz = m_config.update_hz > 0.0 ? m_config.update_hz : 1.0;
        m_timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / recompute_hz), std::bind(&LocalizerNode::timerCB, this));
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

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
        m_config.update_hz = config["update_hz"].as<double>();
        m_config.map_path = config["map_path"] ? config["map_path"].as<std::string>() : m_config.map_path;
        if (config["start_x"])
        {
            m_config.has_start_pose = true;
            m_config.start_x = config["start_x"].as<double>();
            m_config.start_y = config["start_y"] ? config["start_y"].as<double>() : 0.0;
            m_config.start_z = config["start_z"] ? config["start_z"].as<double>() : 0.0;
            m_config.start_yaw = config["start_yaw"] ? config["start_yaw"].as<double>() : 0.0;
            m_config.start_roll = config["start_roll"] ? config["start_roll"].as<double>() : 0.0;
            m_config.start_pitch = config["start_pitch"] ? config["start_pitch"].as<double>() : 0.0;
        }

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();
        m_localizer_config.rough_vgicp_resolution = config["rough_vgicp_resolution"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();
        m_localizer_config.refine_vgicp_resolution = config["refine_vgicp_resolution"].as<double>();
        m_localizer_config.num_threads = config["num_threads"].as<int>();

        m_config.max_offset_jump_dist = config["max_offset_jump_dist"] ?
            config["max_offset_jump_dist"].as<double>() : m_config.max_offset_jump_dist;
        m_config.max_offset_jump_yaw = config["max_offset_jump_yaw"] ?
            config["max_offset_jump_yaw"].as<double>() : m_config.max_offset_jump_yaw;
        m_config.transform_tolerance = config["transform_tolerance"] ?
            config["transform_tolerance"].as<double>() : m_config.transform_tolerance;
    }
    void timerCB()
    {
        if (!m_state.message_received)
            return;

        M4F initial_guess = M4F::Identity();
        if (m_state.service_received)
        {
            std::lock_guard<std::mutex> service_lock(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
            // m_state.service_received = false;
        }
        else
        {
            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
        }

        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        {
            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(m_state.last_cloud);
        }

        bool result = m_localizer->align(initial_guess);
        if (result)
        {
            m_state.consecutive_align_failures = 0;
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            M3D candidate_offset_r = map_body_r * current_local_r.transpose();
            V3D candidate_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;

            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);

            // Gate against a well-scoring but wrong ICP local minimum (long
            // corridors / flat walls / symmetric rooms are classic traps,
            // and this is most likely to bite while parked between goals).
            // Explicit /relocalize calls are exempt -- a jump is exactly what
            // they are for.
            const V3D offset_delta_t = candidate_offset_t - m_state.last_offset_t;
            const M3D offset_delta_r = candidate_offset_r * m_state.last_offset_r.transpose();
            const double jump_dist = offset_delta_t.norm();
            const double jump_yaw = std::abs(std::atan2(offset_delta_r(1, 0), offset_delta_r(0, 0)));

            const bool jump_too_large = m_state.has_aligned_once && !m_state.service_received &&
                (jump_dist > m_config.max_offset_jump_dist || jump_yaw > m_config.max_offset_jump_yaw);

            if (jump_too_large)
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Rejecting map->odom update: jump %.2f m / %.2f rad exceeds limit "
                    "(%.2f m / %.2f rad) -- keeping last accepted offset",
                    jump_dist, jump_yaw, m_config.max_offset_jump_dist, m_config.max_offset_jump_yaw);
            }
            else
            {
                m_state.last_offset_r = candidate_offset_r;
                m_state.last_offset_t = candidate_offset_t;
                m_state.has_aligned_once = true;
                m_state.last_fitness_score = m_localizer->lastFitnessScore();
            }

            if (!m_state.localize_success && m_state.service_received)
            {
                std::lock_guard<std::mutex> service_lock(m_state.service_mutex);
                m_state.localize_success = true;
                m_state.service_received = false;
            }
        }
        else
        {
            // Previously silent -- this is the only signal an operator gets
            // that ICP has stopped converging (e.g. drove outside the mapped
            // area, heavy occlusion). map->odom keeps holding the last
            // accepted offset rather than freezing or disappearing.
            m_state.consecutive_align_failures++;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "ICP alignment failed to converge (%d consecutive attempts) -- "
                "rough[converged=%s score=%.3f/%.3f] refine[converged=%s score=%.3f/%.3f] -- "
                "map->odom is holding the last accepted offset",
                m_state.consecutive_align_failures,
                m_localizer->lastRoughConverged() ? "y" : "n", m_localizer->lastRoughFitness(),
                m_localizer_config.rough_score_thresh,
                m_localizer->lastRefineConverged() ? "y" : "n", m_localizer->lastRefineFitness(),
                m_localizer_config.refine_score_thresh);
        }
        publishMapCloud(current_time);
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        // Captured once per new synced message so map->odom is broadcast with a
        // genuinely fresh, strictly-increasing stamp -- not re-sent unchanged
        // between ICP recomputes (see timerCB).
        builtin_interfaces::msg::Time stamp = cloud_msg->header.stamp;
        {
            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);

            pcl::fromROSMsg(*cloud_msg, *m_state.last_cloud);

            m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                                odom_msg->pose.pose.orientation.x,
                                                odom_msg->pose.pose.orientation.y,
                                                odom_msg->pose.pose.orientation.z)
                                 .toRotationMatrix();
            m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                                 odom_msg->pose.pose.position.y,
                                 odom_msg->pose.pose.position.z);
            m_state.last_message_time = stamp;
            if (!m_state.message_received)
            {
                m_state.message_received = true;
                m_config.local_frame = odom_msg->header.frame_id;
            }
        }
        sendBroadCastTF(stamp);
        publishMapPose(stamp);
    }

    // Composes the latest ICP offset (map->odom) with the latest odometry sample to get
    // the robot pose expressed directly in the map frame, and publishes it Nav2/AMCL-style.
    void publishMapPose(builtin_interfaces::msg::Time &time)
    {
        M3D map_r;
        V3D map_t;
        double fitness_score;
        {
            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
            map_r = m_state.last_offset_r * m_state.last_r;
            map_t = m_state.last_offset_r * m_state.last_t + m_state.last_offset_t;
            fitness_score = m_state.last_fitness_score;
        }
        Eigen::Quaterniond q(map_r);
        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.frame_id = m_config.map_frame;
        pose_msg.header.stamp = time;
        pose_msg.pose.pose.position.x = map_t.x();
        pose_msg.pose.pose.position.y = map_t.y();
        pose_msg.pose.pose.position.z = map_t.z();
        pose_msg.pose.pose.orientation.x = q.x();
        pose_msg.pose.pose.orientation.y = q.y();
        pose_msg.pose.pose.orientation.z = q.z();
        pose_msg.pose.pose.orientation.w = q.w();
        // Heuristic covariance: refine-stage VGICP fitness score is a mean
        // squared point-to-distribution residual (~m^2), so it's a rough
        // proxy for positional variance -- NOT a rigorous statistical
        // estimate the way AMCL's particle spread is, but strictly more
        // honest than the hardcoded zero (== "perfectly certain") this used
        // to publish. Falls back to a deliberately large variance before the
        // first successful alignment.
        const double pos_var = fitness_score >= 0.0 ? std::max(fitness_score, 1e-4) : 1.0;
        const double yaw_var = fitness_score >= 0.0 ? std::max(fitness_score * 4.0, 1e-4) : 1.0;
        pose_msg.pose.covariance[0] = pos_var;  // x
        pose_msg.pose.covariance[7] = pos_var;  // y
        pose_msg.pose.covariance[14] = pos_var; // z
        pose_msg.pose.covariance[21] = yaw_var; // roll (not independently tracked, reuse yaw scale)
        pose_msg.pose.covariance[28] = yaw_var; // pitch
        pose_msg.pose.covariance[35] = yaw_var; // yaw
        m_pose_pub->publish(pose_msg);
    }

    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        // AMCL-style transform_tolerance: stamp the TF slightly ahead of the
        // sensor capture time so a lookup at "now()" (always a little later,
        // due to processing/network latency) isn't rejected as stale even
        // though the offset itself only actually changes at update_hz.
        transformStamped.header.stamp = rclcpp::Time(time) + rclcpp::Duration::from_seconds(m_config.transform_tolerance);
        Eigen::Quaterniond q;
        V3D t;
        {
            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
            q = Eigen::Quaterniond(m_state.last_offset_r);
            t = m_state.last_offset_t;
        }
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);
    }

    // Shared by /initialpose and the yaml-configured start pose: seeds the next
    // alignment with an absolute map-frame guess instead of assuming odom==map.
    void applyInitialGuess(double x, double y, double z, double yaw, double roll, double pitch)
    {
        Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle(pitch, Eigen::Vector3d::UnitY());
        std::lock_guard<std::mutex> service_lock(m_state.service_mutex);
        m_state.initial_guess.setIdentity();
        m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
        m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
        m_state.service_received = true;
        m_state.localize_success = false;
    }

    // RViz "2D Pose Estimate" (or any other /initialpose publisher): re-seeds
    // ICP against the map that's already loaded, without needing a pcd_path.
    void initialPoseCB(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg)
    {
        if (!m_map_loaded)
        {
            RCLCPP_WARN(this->get_logger(), "Ignoring /initialpose: no map loaded yet (set map_path or call /relocalize first)");
            return;
        }
        double roll, pitch, yaw;
        Eigen::Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 0, 1); // yaw, roll, pitch to match applyInitialGuess's composition order
        yaw = euler[0];
        roll = euler[1];
        pitch = euler[2];
        applyInitialGuess(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, yaw, roll, pitch);
        RCLCPP_INFO(this->get_logger(), "Re-seeded from /initialpose: x=%.2f y=%.2f yaw=%.2f", msg->pose.pose.position.x, msg->pose.pose.position.y, yaw);
    }

    void relocCB(const std::shared_ptr<slam_interfaces::srv::Relocalize::Request> request, std::shared_ptr<slam_interfaces::srv::Relocalize::Response> response)
    {
        std::string pcd_path = request->pcd_path;
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        if (!std::filesystem::exists(pcd_path))
        {
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        bool load_flag = m_localizer->loadMap(pcd_path);
        if (!load_flag)
        {
            response->success = false;
            response->message = "load map failed";
            return;
        }
        m_map_loaded = true;
        applyInitialGuess(x, y, z, yaw, roll, pitch);

        response->success = true;
        response->message = "relocalize success";
        return;
    }

    void relocCheckCB(const std::shared_ptr<slam_interfaces::srv::IsValid::Request> request, std::shared_ptr<slam_interfaces::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex> service_lock(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_state.localize_success;
        return;
    }
    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        if (m_map_cloud_pub->get_subscription_count() < 1)
            return;
        CloudType::Ptr map_cloud = m_localizer->refineMap();
        if (map_cloud->size() < 1)
            return;
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

private:
    NodeConfig m_config;
    NodeState m_state;
    bool m_map_loaded = false;

    ICPConfig m_localizer_config;
    std::shared_ptr<ICPLocalizer> m_localizer;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<slam_interfaces::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<slam_interfaces::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_initialpose_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_pose_pub;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
