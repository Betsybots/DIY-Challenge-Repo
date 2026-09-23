#include <queue>
#include <deque>
#include <mutex>
#include <atomic>
#include <filesystem>
#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "slam_interfaces/srv/relocalize.hpp"
#include "slam_interfaces/srv/is_valid.hpp"
#include "slam_interfaces/msg/localization_status.hpp"
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

// Map-frame seed pose for auto-recovery, derived from the loaded map's own
// point cloud (see computeRecoveryHypotheses()) -- no external waypoints
// file to maintain, and it auto-adapts to whatever map gets loaded (startup
// map_path or a later /relocalize with a different pcd).
struct RecoveryHypothesis
{
    double x = 0.0;
    double y = 0.0;
};

// One synced (cloud, odometry pose) sample kept for scan accumulation (see
// buildAccumulatedCloud()). The cloud is a fresh object per callback (never
// mutated in place), so aliasing between history entries and last_cloud is
// safe.
struct ScanRecord
{
    CloudType::Ptr cloud;
    M3D r = M3D::Identity();
    V3D t = V3D::Zero();
};

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
    // Time constant for blending the broadcast map->odom transform toward a
    // new ICP correction. Set to <= 0 to retain immediate updates.
    double offset_smoothing_time_constant = 0.5;
    // Attempt a map-derived grid-hypothesis sweep (see computeRecoveryHypotheses())
    // every time consecutive_align_failures reaches a multiple of this count.
    // 0 disables auto-recovery.
    int recovery_after_failures = 0;
    // Spacing (meters) between candidate grid points covering the loaded
    // map's footprint. Smaller = more thorough but slower sweeps.
    double recovery_grid_spacing = 2.0;
    // Evenly-spaced yaw candidates tried at each grid point (e.g. 4 = every
    // 90 deg). Grid points carry no heading information, unlike a hand-picked
    // waypoint, so more samples are needed here to cover orientation.
    int recovery_yaw_samples = 4;
    // Merge this many of the most recent synced scans (via their own paired
    // odometry) into one cloud before each ICP alignment, instead of aligning
    // a single scan against the map. 1 (default) = disabled, this repo's
    // original behavior. Higher values (2-5) give ICP a denser, more
    // geometrically-constrained input -- useful given the QT64's ~1m blind
    // zone -- at a small added per-cycle transform cost. Clamped to [1, 10].
    int accumulate_scans = 1;
};

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    // Touched from both the ICP-timer callback group and the default group
    // (services/subscriptions) once timerCB runs on its own callback group
    // (see LocalizerNode's constructor) -- atomic so those cross-group reads
    // and writes aren't a data race.
    std::atomic<bool> message_received{false};
    std::atomic<bool> service_received{false};
    std::atomic<bool> localize_success{false};
    builtin_interfaces::msg::Time last_message_time;
    CloudType::Ptr last_cloud = std::make_shared<CloudType>();
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    // Bounded to accumulate_scans entries (see syncCB), most recent last --
    // consumed by buildAccumulatedCloud().
    std::deque<ScanRecord> scan_history;
    M3D last_offset_r = M3D::Identity(); // Smoothed, broadcast map->odom rotation.
    V3D last_offset_t = V3D::Zero();     // Smoothed, broadcast map->odom translation.
    M3D target_offset_r = M3D::Identity();
    V3D target_offset_t = V3D::Zero();
    rclcpp::Time last_smooth_time;
    rclcpp::Time last_align_time;
    M4F initial_guess = M4F::Identity();
    // Odometry pose at the moment the current /relocalize or /initialpose
    // request was received -- lets timerCB's service_received branch keep
    // tracking the robot's actual motion on every retry, instead of reusing
    // the exact requested pose forever if the first attempt fails.
    M3D service_ref_r = M3D::Identity();
    V3D service_ref_t = V3D::Zero();
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
            std::lock_guard<std::mutex> localizer_lock(m_localizer_mutex);
            if (std::filesystem::exists(m_config.map_path) && m_localizer->loadMap(m_config.map_path))
            {
                m_map_loaded = true;
                RCLCPP_INFO(this->get_logger(), "AUTO-LOADED MAP: %s", m_config.map_path.c_str());
                computeRecoveryHypotheses();
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
        // Lets other nodes (e.g. motion_controller) throttle speed or react
        // when map->odom stops being refreshed, instead of only finding out
        // indirectly via a pose jump.
        m_status_pub = this->create_publisher<slam_interfaces::msg::LocalizationStatus>("localization_status", 10);

        // timerCB (ICP) runs in its own callback group so a slow align() or
        // recovery sweep can't delay syncCB's TF broadcast / the smoothing
        // tick, which stay in the node's default group -- requires
        // MultiThreadedExecutor in main() to actually run concurrently.
        // m_localizer_mutex (see below) protects the one thing genuinely
        // shared between the two groups: m_localizer itself.
        m_icp_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_timer = this->create_wall_timer(10ms, std::bind(&LocalizerNode::timerCB, this), m_icp_callback_group);
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
        m_config.offset_smoothing_time_constant = config["offset_smoothing_time_constant"] ?
            config["offset_smoothing_time_constant"].as<double>() :
            m_config.offset_smoothing_time_constant;
        m_config.recovery_after_failures = config["recovery_after_failures"] ?
            config["recovery_after_failures"].as<int>() : m_config.recovery_after_failures;
        m_config.recovery_grid_spacing = config["recovery_grid_spacing"] ?
            config["recovery_grid_spacing"].as<double>() : m_config.recovery_grid_spacing;
        m_config.recovery_yaw_samples = config["recovery_yaw_samples"] ?
            config["recovery_yaw_samples"].as<int>() : m_config.recovery_yaw_samples;
        m_config.accumulate_scans = config["accumulate_scans"] ?
            config["accumulate_scans"].as<int>() : m_config.accumulate_scans;
        m_config.accumulate_scans = std::clamp(m_config.accumulate_scans, 1, 10);
    }

    // Derives map-frame seed hypotheses for auto-recovery directly from the
    // loaded map's own point cloud: lays a grid over its XY footprint at
    // recovery_grid_spacing, keeping only cells with actual map structure
    // nearby (an occupancy check against the map cloud voxelized at half that
    // spacing) so hypotheses aren't wasted on open space outside the mapped
    // area's footprint.
    // Precondition: caller holds m_localizer_mutex (this touches m_localizer
    // and m_recovery_hypotheses, both shared with the ICP callback group).
    void computeRecoveryHypotheses()
    {
        m_recovery_hypotheses.clear();
        CloudType::Ptr map_cloud = m_localizer->roughMap();
        if (!map_cloud || map_cloud->empty())
            return;

        const double occ_res = std::max(m_config.recovery_grid_spacing * 0.5, 0.1);
        constexpr int64_t kOffset = 1'000'000;
        constexpr int64_t kStride = 4'000'000;
        auto cell_key = [&](int64_t ix, int64_t iy) {
            return (ix + kOffset) * kStride + (iy + kOffset);
        };

        std::unordered_set<int64_t> occupied_cells;
        occupied_cells.reserve(map_cloud->size());
        double min_x = std::numeric_limits<double>::max();
        double max_x = std::numeric_limits<double>::lowest();
        double min_y = std::numeric_limits<double>::max();
        double max_y = std::numeric_limits<double>::lowest();
        for (const auto &pt : map_cloud->points)
        {
            min_x = std::min(min_x, static_cast<double>(pt.x));
            max_x = std::max(max_x, static_cast<double>(pt.x));
            min_y = std::min(min_y, static_cast<double>(pt.y));
            max_y = std::max(max_y, static_cast<double>(pt.y));
            occupied_cells.insert(cell_key(
                static_cast<int64_t>(std::floor(pt.x / occ_res)),
                static_cast<int64_t>(std::floor(pt.y / occ_res))));
        }

        auto has_structure_nearby = [&](double x, double y) {
            const int64_t cx = static_cast<int64_t>(std::floor(x / occ_res));
            const int64_t cy = static_cast<int64_t>(std::floor(y / occ_res));
            for (int64_t dx = -1; dx <= 1; ++dx)
                for (int64_t dy = -1; dy <= 1; ++dy)
                    if (occupied_cells.count(cell_key(cx + dx, cy + dy)))
                        return true;
            return false;
        };

        const double spacing = std::max(m_config.recovery_grid_spacing, 0.1);
        for (double x = min_x; x <= max_x; x += spacing)
        {
            for (double y = min_y; y <= max_y; y += spacing)
            {
                if (has_structure_nearby(x, y))
                    m_recovery_hypotheses.push_back({x, y});
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Computed %zu auto-recovery grid hypothesis(es) from the loaded map (spacing=%.1fm)",
            m_recovery_hypotheses.size(), spacing);
    }

    // Merges scan_history entries into one cloud expressed in target_r/target_t's
    // body frame (same-frame composition math as the map->odom offset elsewhere
    // in this file), instead of aligning ICP against a single, possibly
    // feature-thin scan (QT64 has a ~1m blind zone). No-op (returns the most
    // recent scan verbatim, zero extra cost) when accumulate_scans <= 1 or
    // fewer than 2 scans are available yet.
    CloudType::Ptr buildAccumulatedCloud(const std::vector<ScanRecord> &history, const M3D &target_r, const V3D &target_t)
    {
        if (history.empty())
            return std::make_shared<CloudType>();
        if (history.size() == 1 || m_config.accumulate_scans <= 1)
            return history.back().cloud;

        CloudType::Ptr accumulated = std::make_shared<CloudType>(*history.back().cloud);
        for (size_t i = 0; i + 1 < history.size(); ++i)
        {
            const ScanRecord &hist = history[i];
            const M3D rel_r = target_r.transpose() * hist.r;
            const V3D rel_t = target_r.transpose() * (hist.t - target_t);
            M4F transform = M4F::Identity();
            transform.block<3, 3>(0, 0) = rel_r.cast<float>();
            transform.block<3, 1>(0, 3) = rel_t.cast<float>();
            CloudType transformed;
            pcl::transformPointCloud(*hist.cloud, transformed, transform);
            accumulated->operator+=(transformed);
        }
        return accumulated;
    }
    void timerCB()
    {
        if (!m_state.message_received)
            return;

        advanceSmoothedOffset();

        const rclcpp::Time now = this->now();
        const double align_period = 1.0 / std::max(m_config.update_hz, 1e-6);
        if (m_state.last_align_time.nanoseconds() != 0 &&
            (now - m_state.last_align_time).seconds() < align_period)
        {
            return;
        }
        m_state.last_align_time = now;

        // Single snapshot of the odometry pose/cloud/stamp that will actually be
        // aligned, taken once under one lock -- both the ICP warm-start guess
        // below and the offset computed after align() are derived from this same
        // capture, so a syncCB update landing mid-timerCB can't pair the guess
        // with different odometry than what was actually aligned.
        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
        CloudType::Ptr current_cloud;
        M3D current_target_offset_r;
        V3D current_target_offset_t;
        std::vector<ScanRecord> history_snapshot;
        {
            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            current_target_offset_r = m_state.target_offset_r;
            current_target_offset_t = m_state.target_offset_t;
            history_snapshot.assign(m_state.scan_history.begin(), m_state.scan_history.end());
        }
        current_cloud = buildAccumulatedCloud(history_snapshot, current_local_r, current_local_t);

        // Guards every m_localizer call/m_recovery_hypotheses access below --
        // m_localizer is also touched by relocCB/the constructor (default
        // callback group) via loadMap(). Held through publishLocalizationStatus()/
        // publishMapCloud() at the end of this function (both also read
        // m_localizer and are only ever called from here).
        std::lock_guard<std::mutex> localizer_lock(m_localizer_mutex);
        m_localizer->setInput(current_cloud);

        M4F initial_guess = M4F::Identity();
        if (m_state.service_received)
        {
            M3D requested_r;
            V3D requested_t;
            M3D service_ref_r;
            V3D service_ref_t;
            {
                std::lock_guard<std::mutex> service_lock(m_state.service_mutex);
                requested_r = m_state.initial_guess.block<3, 3>(0, 0).cast<double>();
                requested_t = m_state.initial_guess.block<3, 1>(0, 3).cast<double>();
                service_ref_r = m_state.service_ref_r;
                service_ref_t = m_state.service_ref_t;
            }
            // Track the robot's own odometry motion since the request instead of
            // retrying the exact same static requested pose forever: service_received
            // is only ever cleared on a successful alignment (see below), so without
            // this, a failed first attempt would keep re-seeding from an
            // increasingly stale guess for the rest of the run.
            const M3D req_offset_r = requested_r * service_ref_r.transpose();
            const V3D req_offset_t = -req_offset_r * service_ref_t + requested_t;
            initial_guess.block<3, 3>(0, 0) = (req_offset_r * current_local_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (req_offset_r * current_local_t + req_offset_t).cast<float>();
        }
        else
        {
            initial_guess.block<3, 3>(0, 0) = (current_target_offset_r * current_local_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (current_target_offset_r * current_local_t + current_target_offset_t).cast<float>();
        }

        bool result = m_localizer->align(initial_guess);
        if (result)
        {
            if (m_state.consecutive_align_failures > 0)
            {
                RCLCPP_INFO(
                    this->get_logger(),
                    "ICP map alignment RECOVERED after %d consecutive failure(s) (rough fitness=%.3f, refine fitness=%.3f)",
                    m_state.consecutive_align_failures,
                    m_localizer->lastRoughFitness(), m_localizer->lastRefineFitness());
            }
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
            const V3D offset_delta_t = candidate_offset_t - m_state.target_offset_t;
            const M3D offset_delta_r = candidate_offset_r * m_state.target_offset_r.transpose();
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
                m_state.target_offset_r = candidate_offset_r;
                m_state.target_offset_t = candidate_offset_t;
                m_state.has_aligned_once = true;
                m_state.last_fitness_score = m_localizer->lastFitnessScore();
            }

            if (!m_state.localize_success && m_state.service_received)
            {
                std::lock_guard<std::mutex> service_lock(m_state.service_mutex);
                m_state.localize_success = true;
                m_state.service_received = false;
                // An explicit /relocalize or /initialpose has no prior localized
                // state to stay continuous with -- snap instead of blending.
                m_state.last_offset_r = m_state.target_offset_r;
                m_state.last_offset_t = m_state.target_offset_t;
                // This is the log line that actually confirms localization is
                // trustworthy -- the /relocalize service response (see relocCB)
                // only ever confirmed the map file loaded.
                RCLCPP_INFO(
                    this->get_logger(),
                    "Map lock ACQUIRED: map->odom translation=(%.3f, %.3f, %.3f)",
                    m_state.target_offset_t.x(), m_state.target_offset_t.y(), m_state.target_offset_t.z());
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

            if (m_config.recovery_after_failures > 0 && !m_recovery_hypotheses.empty() &&
                m_state.consecutive_align_failures % m_config.recovery_after_failures == 0)
            {
                // attemptRecoverySweep() also touches m_localizer -- relies on
                // localizer_lock (above) still being held here, does not lock
                // itself (see its own precondition comment).
                attemptRecoverySweep(current_local_r, current_local_t);
            }
        }
        publishLocalizationStatus(current_time);
        publishMapCloud(current_time);
    }

    // Sweeps map-derived grid hypotheses (see computeRecoveryHypotheses()) trying
    // to re-lock from scratch, for when ICP has been failing against the
    // last-known offset for a while (e.g. the robot was picked up/got lost
    // beyond the offset's convergence basin). Blocking (like the regular
    // align() call this augments) -- runs on the same timer thread, so a full
    // sweep briefly delays the next TF/status publish; acceptable since the
    // alternative is staying lost indefinitely. Returns on the first
    // hypothesis that converges rather than exhaustively scoring all of them.
    // Precondition: m_localizer->setInput() has already been called with this
    // scan (true for its only caller, timerCB's failure branch) -- not
    // repeated here since fast_gicp caches by cloud pointer identity anyway.
    // Precondition: caller holds m_localizer_mutex (true for its only caller,
    // timerCB's failure branch) -- not acquired here to avoid deadlocking on
    // a non-recursive mutex already held by that caller.
    bool attemptRecoverySweep(const M3D &local_r, const V3D &local_t)
    {
        const int yaw_samples = std::max(1, m_config.recovery_yaw_samples);
        RCLCPP_WARN(
            this->get_logger(),
            "Auto-recovery: %d consecutive alignment failures -- sweeping %zu grid hypothesis(es) "
            "x %d yaw sample(s) to attempt re-lock",
            m_state.consecutive_align_failures, m_recovery_hypotheses.size(), yaw_samples);

        for (size_t i = 0; i < m_recovery_hypotheses.size(); ++i)
        {
            const RecoveryHypothesis &hyp = m_recovery_hypotheses[i];
            for (int k = 0; k < yaw_samples; ++k)
            {
                const double yaw = (2.0 * M_PI * k) / yaw_samples;
                M4F guess = M4F::Identity();
                Eigen::AngleAxisd yaw_angle(yaw, Eigen::Vector3d::UnitZ());
                guess.block<3, 3>(0, 0) = yaw_angle.toRotationMatrix().cast<float>();
                guess.block<3, 1>(0, 3) = V3F(hyp.x, hyp.y, 0.0f);
                if (!m_localizer->align(guess))
                    continue;

                const M3D map_body_r = guess.block<3, 3>(0, 0).cast<double>();
                const V3D map_body_t = guess.block<3, 1>(0, 3).cast<double>();
                const M3D recovered_offset_r = map_body_r * local_r.transpose();
                const V3D recovered_offset_t = -map_body_r * local_r.transpose() * local_t + map_body_t;

                std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
                m_state.target_offset_r = recovered_offset_r;
                m_state.target_offset_t = recovered_offset_t;
                // No prior localized state worth staying continuous with -- snap.
                m_state.last_offset_r = recovered_offset_r;
                m_state.last_offset_t = recovered_offset_t;
                m_state.has_aligned_once = true;
                m_state.localize_success = true;
                m_state.last_fitness_score = m_localizer->lastFitnessScore();
                m_state.consecutive_align_failures = 0;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Auto-recovery: Map lock RE-ACQUIRED from grid hypothesis #%zu (x=%.2f y=%.2f) yaw=%.2f -- "
                    "map->odom translation=(%.3f, %.3f, %.3f)",
                    i, hyp.x, hyp.y, yaw,
                    recovered_offset_t.x(), recovered_offset_t.y(), recovered_offset_t.z());
                return true;
            }
        }
        RCLCPP_WARN(
            this->get_logger(),
            "Auto-recovery: none of %zu grid hypothesis(es) x %d yaw sample(s) converged -- still lost, "
            "will retry after another %d consecutive failure(s)",
            m_recovery_hypotheses.size(), yaw_samples, m_config.recovery_after_failures);
        return false;
    }

    // Precondition: caller holds m_localizer_mutex (true for its only caller,
    // timerCB, via the lock held for the whole ICP section -- not acquired
    // here to avoid deadlocking on a non-recursive mutex already held).
    void publishLocalizationStatus(builtin_interfaces::msg::Time &time)
    {
        slam_interfaces::msg::LocalizationStatus status;
        status.header.stamp = time;
        status.header.frame_id = m_config.map_frame;
        status.localized = m_state.localize_success;
        status.consecutive_align_failures = m_state.consecutive_align_failures;
        status.rough_fitness = m_localizer->lastRoughFitness();
        status.refine_fitness = m_localizer->lastRefineFitness();
        m_status_pub->publish(status);
    }

    void advanceSmoothedOffset()
    {
        std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
        const rclcpp::Time now = this->now();
        if (m_state.last_smooth_time.nanoseconds() == 0)
        {
            m_state.last_smooth_time = now;
            return;
        }

        const double dt = (now - m_state.last_smooth_time).seconds();
        m_state.last_smooth_time = now;
        if (dt <= 0.0 || dt > 1.0)
            return;

        if (m_config.offset_smoothing_time_constant <= 0.0)
        {
            m_state.last_offset_r = m_state.target_offset_r;
            m_state.last_offset_t = m_state.target_offset_t;
            return;
        }

        const double alpha = 1.0 - std::exp(-dt / m_config.offset_smoothing_time_constant);
        m_state.last_offset_t = (1.0 - alpha) * m_state.last_offset_t + alpha * m_state.target_offset_t;
        Eigen::Quaterniond current(m_state.last_offset_r);
        Eigen::Quaterniond target(m_state.target_offset_r);
        current.normalize();
        target.normalize();
        m_state.last_offset_r = current.slerp(alpha, target).toRotationMatrix();
    }
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg, const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        // Captured once per new synced message so map->odom is broadcast with a
        // genuinely fresh, strictly-increasing stamp -- not re-sent unchanged
        // between ICP recomputes (see timerCB).
        builtin_interfaces::msg::Time stamp = cloud_msg->header.stamp;
        {
            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);

            // Fresh object per callback (not reused in place) so history entries
            // pushed below stay valid/unmutated once earlier scans are appended.
            CloudType::Ptr fresh_cloud = std::make_shared<CloudType>();
            pcl::fromROSMsg(*cloud_msg, *fresh_cloud);
            m_state.last_cloud = fresh_cloud;

            m_state.last_r = Eigen::Quaterniond(odom_msg->pose.pose.orientation.w,
                                                odom_msg->pose.pose.orientation.x,
                                                odom_msg->pose.pose.orientation.y,
                                                odom_msg->pose.pose.orientation.z)
                                 .toRotationMatrix();
            m_state.last_t = V3D(odom_msg->pose.pose.position.x,
                                 odom_msg->pose.pose.position.y,
                                 odom_msg->pose.pose.position.z);
            m_state.last_message_time = stamp;

            m_state.scan_history.push_back({fresh_cloud, m_state.last_r, m_state.last_t});
            const size_t max_history = static_cast<size_t>(std::max(1, m_config.accumulate_scans));
            while (m_state.scan_history.size() > max_history)
                m_state.scan_history.pop_front();

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
        // Snapshot current odometry as the reference for tracking live motion on
        // retries (see timerCB); falls back to identity if no odometry has
        // arrived yet -- no worse than always-frozen until the first scan.
        M3D service_ref_r = M3D::Identity();
        V3D service_ref_t = V3D::Zero();
        {
            std::lock_guard<std::mutex> message_lock(m_state.message_mutex);
            if (m_state.message_received)
            {
                service_ref_r = m_state.last_r;
                service_ref_t = m_state.last_t;
            }
        }
        std::lock_guard<std::mutex> service_lock(m_state.service_mutex);
        m_state.initial_guess.setIdentity();
        m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
        m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
        m_state.service_ref_r = service_ref_r;
        m_state.service_ref_t = service_ref_t;
        m_state.service_received = true;
        m_state.localize_success = false;
        m_state.consecutive_align_failures = 0;
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

        RCLCPP_INFO(
            this->get_logger(),
            "Received /relocalize request: pcd_path=%s initial pose=(x=%.2f, y=%.2f, z=%.2f, yaw=%.2f, roll=%.2f, pitch=%.2f)",
            pcd_path.c_str(), x, y, z, yaw, roll, pitch);

        if (!std::filesystem::exists(pcd_path))
        {
            RCLCPP_ERROR(this->get_logger(), "relocalize REJECTED: pcd file not found: %s", pcd_path.c_str());
            response->success = false;
            response->message = "pcd file not found";
            return;
        }

        bool load_flag;
        {
            std::lock_guard<std::mutex> localizer_lock(m_localizer_mutex);
            load_flag = m_localizer->loadMap(pcd_path);
            if (load_flag)
                computeRecoveryHypotheses();
        }
        if (!load_flag)
        {
            RCLCPP_ERROR(this->get_logger(), "relocalize REJECTED: failed to load pcd map: %s", pcd_path.c_str());
            response->success = false;
            response->message = "load map failed";
            return;
        }
        m_map_loaded = true;
        applyInitialGuess(x, y, z, yaw, roll, pitch);

        RCLCPP_INFO(
            this->get_logger(),
            "relocalize ACCEPTED: map loaded, will attempt ICP alignment every %.2fs (tracked "
            "against live odometry) until it converges -- watch for a 'Map lock ACQUIRED' or "
            "repeated alignment-failed warnings to know whether this actually took effect (this "
            "response alone only confirms the map file loaded, not that ICP has converged)",
            1.0 / std::max(m_config.update_hz, 1e-6));

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
    // Precondition: caller holds m_localizer_mutex (true for its only caller,
    // timerCB, via the lock held for the whole ICP section).
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
    // Guards every m_localizer call and m_recovery_hypotheses access -- both
    // are reachable from the ICP callback group (timerCB) and the default
    // group (relocCB/constructor via loadMap()).
    std::mutex m_localizer_mutex;
    std::vector<RecoveryHypothesis> m_recovery_hypotheses;
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
    rclcpp::Publisher<slam_interfaces::msg::LocalizationStatus>::SharedPtr m_status_pub;
    rclcpp::CallbackGroup::SharedPtr m_icp_callback_group;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizerNode>();
    // MultiThreadedExecutor is required for timerCB's dedicated callback
    // group (see the constructor) to actually run concurrently with syncCB/
    // the services, instead of the default single-threaded executor still
    // serializing everything regardless of group.
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
