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
    double score; // FastGICP fitness score, used directly as the loop factor's noise variance
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
};

class SimplePGO
{
public:
    SimplePGO(const Config &config);

    bool isKeyPose(const PoseWithTime &pose);

    bool addKeyPose(const CloudWithPose &cloud_with_pose);

    bool hasLoop(){return m_cache_pairs.size() > 0;}

    void searchForLoopPairs();

    void smoothAndUpdate();

    CloudType::Ptr getSubMap(int idx, int half_range, double resolution);
    std::vector<std::pair<size_t, size_t>> &historyPairs() { return m_history_pairs; }
    std::vector<KeyPoseWithCloud> &keyPoses() { return m_key_poses; }

    M3D offsetR() { return m_r_offset; }
    V3D offsetT() { return m_t_offset; }

    // Source/target submaps (both already in the CURRENT global frame, same
    // as getSubMap()'s output) from the most recent candidate that made it
    // far enough through searchForLoopPairs() to actually run ICP -- set
    // whether that candidate was ultimately accepted or rejected by a later
    // gate. Lets pgo_node publish a live "what is loop closure comparing
    // right now" view (this package had no visibility into proximity/loop
    // candidates before a closure was actually accepted, which is useless
    // for diagnosing why closures aren't firing).
    bool hasCandidateCloudsThisCycle() const { return m_have_candidate_clouds; }
    CloudType::Ptr candidateTargetCloud() const { return m_candidate_target_cloud; }
    CloudType::Ptr candidateSourceCloud() const { return m_candidate_source_cloud_aligned; }
    void clearCandidateCloudsFlag() { m_have_candidate_clouds = false; }

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

    // See hasCandidateCloudsThisCycle()/candidateTargetCloud()/candidateSourceCloud().
    bool m_have_candidate_clouds = false;
    CloudType::Ptr m_candidate_target_cloud;
    CloudType::Ptr m_candidate_source_cloud_aligned;
};