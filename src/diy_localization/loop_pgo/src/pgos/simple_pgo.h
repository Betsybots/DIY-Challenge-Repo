#pragma once
#include "commons.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <fast_gicp/gicp/fast_gicp.hpp>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include "../scancontext/Scancontext.h"

struct KeyPoseWithCloud
{
    M3D r_local;
    V3D t_local;
    M3D r_global;
    V3D t_global;
    double time;
    // Cumulative odometry path length (sum of |t_between| up to and including
    // this keyframe) -- distinguishes a genuine revisit (traveled away and
    // back) from merely being spatially close in the CURRENT estimate after
    // idling/slow-driving/rotating in place nearby, which time or keyframe
    // count alone can't tell apart from a real loop.
    double path_length;
    CloudType::Ptr body_cloud;
};
struct LoopPair
{
    size_t source_id;
    size_t target_id;
    M3D r_offset;
    V3D t_offset;
    double score; // FastGICP fitness score, used as the loop factor's isotropic noise floor
    // ICP's Gauss-Newton Hessian (JtJ) at convergence for this pair's best-fitness
    // guess, tangent order [rot xyz, trans xyz] -- see conditionLoopInformation()
    // in simple_pgo.cpp. Only its eigen-DIRECTIONS and RELATIVE eigenvalue ratios
    // are used (never its absolute magnitudes directly), so a miscalibrated
    // Hessian scale can only ever loosen trust in a direction, never make the
    // factor more confident than the fitness-based floor alone would allow.
    Eigen::Matrix<double, 6, 6> hessian = Eigen::Matrix<double, 6, 6>::Identity();
};

struct Config
{
    double key_pose_delta_deg = 10;
    double key_pose_delta_trans = 1.0;
    double loop_search_radius = 1.0;
    double loop_score_tresh = 0.15;
    // Scan Context's keyframe-gap exclusion window (see NUM_EXCLUDE_RECENT in
    // Scancontext.h). Below this many keyframes of separation, the
    // descriptor-based fallback never attempts a match -- keep in sync with
    // how short your actual test/production loops are expected to be.
    int num_exclude_recent = 50;
    // Scan Context descriptor-distance acceptance gate (see SC_DIST_THRES in
    // Scancontext.h). In a small/symmetric room, different physical poses can
    // produce near-identical descriptors (perceptual aliasing), so a loose
    // threshold here accepts false candidates well before an actual revisit.
    // Tighten (lower) if you see "[Loop found]" against an early keyframe
    // that the robot has not actually returned to.
    double sc_dist_thres = 0.13;
    int loop_submap_half_range = 5;
    double submap_resolution = 0.1;
    double min_loop_detect_duration = 10.0;
    int loop_consistency_count = 3;
    // Between-factor (sequential odometry edge) noise, scaled by how far the
    // robot's own odometry says it actually moved/turned between the two
    // keyframes it links (see addKeyPose()). Previously this was a single
    // fixed, very tight variance regardless of delta size -- that makes every
    // odometry edge in the graph almost rigid, so even a correctly-detected
    // loop closure can only nudge the trajectory a little, leaving a visible
    // seam/duplicated geometry (e.g. double walls at corners) in the saved
    // map instead of the loop fully smoothing it out. Tune these to your
    // odometry's real drift rate if ghosting persists; the fixed floors (see
    // addKeyPose()) only guard the near-zero-delta edge case.
    double odom_trans_noise_per_meter = 0.05; // 1-sigma meters of drift per meter of edge translation
    double odom_rot_noise_per_rad = 0.05;     // 1-sigma radians of drift per radian of edge rotation

    // ISAM2 incremental solver tuning (was hardcoded in SimplePGO's ctor).
    double isam2_relinearize_threshold = 0.01;
    int isam2_relinearize_skip = 1;

    // FastGICP loop-closure ICP tuning (was hardcoded in SimplePGO's ctor).
    // Max correspondence distance should roughly match the submap's real
    // extent (loop_submap_half_range * key_pose_delta_trans), otherwise ICP
    // can accept correspondences across open space to the wrong structure.
    double icp_max_correspondence_distance = 3.0;
    int icp_correspondence_randomness = 20;
    int icp_max_iterations = 50;
    double icp_transformation_epsilon = 1e-6;

    // Hybrid loop-factor noise model (see conditionLoopInformation() in
    // simple_pgo.cpp): the ICP fitness score sets an isotropic variance FLOOR
    // for all 6 DOF (matches LIO-SAM/SC-A-LOAM's own approach, never more
    // confident than that baseline). The Hessian's eigen-directions are used
    // ONLY to detect genuinely degenerate directions (eigenvalue ratio to the
    // strongest direction below this threshold) and loosen those specific
    // directions further -- it can never tighten a direction below the
    // fitness-based floor, so a miscalibrated/overconfident raw Hessian can't
    // silently make the solver over-trust a bad loop closure.
    double hessian_degeneracy_ratio_threshold = 0.05;
    // Upper bound on the loosened variance for a degenerate direction, so a
    // near-zero eigenvalue ratio can't blow up to a numerically unusable
    // value in ISAM2.
    double hessian_max_variance = 4.0;
};

class SimplePGO
{
public:
    SimplePGO(const Config &config);

    bool isKeyPose(const PoseWithTime &pose);

    bool addKeyPose(const CloudWithPose &cloud_with_pose);


    void searchForLoopPairs();

    void smoothAndUpdate();

    CloudType::Ptr getSubMap(int idx, int half_range, double resolution);
    std::vector<std::pair<size_t, size_t>> &historyPairs() { return m_history_pairs; }
    std::vector<KeyPoseWithCloud> &keyPoses() { return m_key_poses; }

    M3D offsetR() { return m_r_offset; }
    V3D offsetT() { return m_t_offset; }

private:
    Config m_config;
    std::vector<KeyPoseWithCloud> m_key_poses;
    std::vector<std::pair<size_t, size_t>> m_history_pairs;
    std::vector<LoopPair> m_cache_pairs;
    M3D m_r_offset;
    V3D m_t_offset;
    std::shared_ptr<gtsam::ISAM2> m_isam2;
    gtsam::Values m_initial_values;
    gtsam::NonlinearFactorGraph m_graph;
    fast_gicp::FastGICP<PointType, PointType> m_icp;
    SCManager m_sc_manager; // descriptor-based loop candidates, complements the KD-tree radius search
    size_t m_pending_loop_source = 0;
    int m_pending_loop_target = -1;
    int m_pending_loop_count = 0;
    // Correction implied by the most recent pending candidate, used to
    // require consecutive candidates to imply a consistent correction (not
    // just a consistent target keyframe index) -- see searchForLoopPairs().
    M3D m_pending_loop_r_offset = M3D::Identity();
    V3D m_pending_loop_t_offset = V3D::Zero();
};