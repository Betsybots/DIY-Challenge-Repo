#include "simple_pgo.h"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigenvalues>

namespace
{
// Hybrid loop-factor noise model. Two ingredients:
//  1. base_variance (from ICP fitness score, same as LIO-SAM/SC-A-LOAM's own
//     approach) sets an ISOTROPIC FLOOR applied to every one of the 6 DOF --
//     this is exactly what the plain fitness-based model already does, and
//     is never made more confident than that here.
//  2. The Hessian's eigen-DIRECTIONS and RELATIVE eigenvalue ratios (never
//     its absolute magnitudes) are used only to detect genuinely degenerate
//     directions (e.g. along a corridor's own axis) and loosen those specific
//     directions beyond the floor, capped at hessian_max_variance.
// Because a direction can only ever end up >= base_variance (never lower),
// a miscalibrated/overconfident raw Hessian can't make the solver trust a
// bad loop closure more than the vetted fitness-based baseline would --
// this was the main risk with the original, now-removed
// conditionLoopInformation() that used the Hessian's absolute eigenvalues
// directly as the information matrix.
Eigen::Matrix<double, 6, 6> conditionLoopInformation(
    const Eigen::Matrix<double, 6, 6> &hessian, double base_variance, const Config &config)
{
    const Eigen::Matrix<double, 6, 6> symmetric_hessian = 0.5 * (hessian + hessian.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(symmetric_hessian);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
    {
        // Can't safely evaluate degeneracy -- fall back to the plain
        // isotropic fitness-based floor everywhere.
        return Eigen::Matrix<double, 6, 6>::Identity() * (1.0 / base_variance);
    }

    const Eigen::Matrix<double, 6, 1> eigenvalues = solver.eigenvalues();
    const double max_eigenvalue = eigenvalues.maxCoeff();
    Eigen::Matrix<double, 6, 1> variances;
    for (int i = 0; i < 6; ++i)
    {
        const double ratio = (max_eigenvalue > 1e-9) ? (eigenvalues(i) / max_eigenvalue) : 0.0;
        if (ratio < config.hessian_degeneracy_ratio_threshold)
        {
            const double safe_ratio = std::max(ratio, 1e-6);
            variances(i) = std::min(base_variance / safe_ratio, config.hessian_max_variance);
        }
        else
        {
            variances(i) = base_variance;
        }
    }

    return solver.eigenvectors() * variances.cwiseInverse().asDiagonal() * solver.eigenvectors().transpose();
}
} // namespace

SimplePGO::SimplePGO(const Config &config) : m_config(config)
{
    m_sc_manager.setNumExcludeRecent(config.num_exclude_recent);
    m_sc_manager.setSCDistThres(config.sc_dist_thres);
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = config.isam2_relinearize_threshold;
    isam2_params.relinearizeSkip = config.isam2_relinearize_skip;
    m_isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    m_initial_values.clear();
    m_graph.resize(0);
    m_r_offset.setIdentity();
    m_t_offset.setZero();

    // Was 10m -- far larger than the submap itself (loop_submap_half_range=5
    // keyframes * key_pose_delta_trans=0.5m spacing, a ~2.5m radius), which let
    // ICP accept correspondences across open space to the wrong nearby
    // structure (e.g. the wrong corner) instead of only genuinely close
    // geometry. Tightened to roughly match the submap's real extent.
    m_icp.setMaxCorrespondenceDistance(config.icp_max_correspondence_distance);
    m_icp.setNumThreads(0); // 0 = use all available threads
    m_icp.setCorrespondenceRandomness(config.icp_correspondence_randomness);
    m_icp.setRegularizationMethod(fast_gicp::RegularizationMethod::PLANE);
    m_icp.setMaximumIterations(config.icp_max_iterations);
    m_icp.setTransformationEpsilon(config.icp_transformation_epsilon);
}

bool SimplePGO::isKeyPose(const PoseWithTime &pose)
{
    if (m_key_poses.size() == 0)
        return true;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    double delta_trans = (pose.t - last_item.t_local).norm();
    double delta_deg = Eigen::Quaterniond(pose.r).angularDistance(Eigen::Quaterniond(last_item.r_local)) * 57.324;
    if (delta_trans > m_config.key_pose_delta_trans || delta_deg > m_config.key_pose_delta_deg)
        return true;
    return false;
}
bool SimplePGO::addKeyPose(const CloudWithPose &cloud_with_pose)
{
    bool is_key_pose = isKeyPose(cloud_with_pose.pose);
    if (!is_key_pose)
        return false;
    size_t idx = m_key_poses.size();
    M3D init_r = m_r_offset * cloud_with_pose.pose.r;
    V3D init_t = m_r_offset * cloud_with_pose.pose.t + m_t_offset;
    // 添加初始值
    m_initial_values.insert(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)));
    if (idx == 0)
    {
        // 添加先验约束
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
        m_graph.add(gtsam::PriorFactor<gtsam::Pose3>(idx, gtsam::Pose3(gtsam::Rot3(init_r), gtsam::Point3(init_t)), noise));
    }
    else
    {
        const KeyPoseWithCloud &last_item = m_key_poses.back();
        M3D r_between = last_item.r_local.transpose() * cloud_with_pose.pose.r;
        V3D t_between = last_item.r_local.transpose() * (cloud_with_pose.pose.t - last_item.t_local);
        // Scale by the *actual* measured delta for this edge (not the
        // isKeyPose() spacing target) so a longer stretch between keyframes
        // -- more room for odometry to have drifted -- gets a proportionally
        // looser edge, letting loop-closure corrections actually propagate
        // back through it instead of fighting a near-rigid prior.
        const double dtrans = t_between.norm();
        const double drot = Eigen::Quaterniond(r_between).angularDistance(Eigen::Quaterniond::Identity());
        // Floors are fixed, not exposed: they only guard the near-zero-delta
        // edge case (a keyframe added by rotation alone with ~0 translation,
        // or vice versa) from collapsing to an under-constrained variance~0
        // edge -- not something that needs retuning per course, unlike the
        // per_meter/per_rad slopes below which describe this odometry's own
        // real drift rate.
        constexpr double kOdomTransNoiseFloor = 0.01; // meters
        constexpr double kOdomRotNoiseFloor = 0.01;   // radians
        constexpr double kOdomZNoiseFloor = 0.001;    // meters, ground robot
        const double trans_sigma = std::max(kOdomTransNoiseFloor, m_config.odom_trans_noise_per_meter * dtrans);
        const double rot_sigma = std::max(kOdomRotNoiseFloor, m_config.odom_rot_noise_per_rad * drot);
        const double z_sigma = kOdomZNoiseFloor;
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << rot_sigma * rot_sigma, rot_sigma * rot_sigma, rot_sigma * rot_sigma,
                                 trans_sigma * trans_sigma, trans_sigma * trans_sigma, z_sigma * z_sigma).finished());
        m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(idx - 1, idx, gtsam::Pose3(gtsam::Rot3(r_between), gtsam::Point3(t_between)), noise));
    }
    KeyPoseWithCloud item;
    item.time = cloud_with_pose.pose.second;
    item.r_local = cloud_with_pose.pose.r;
    item.t_local = cloud_with_pose.pose.t;
    item.body_cloud = cloud_with_pose.cloud;
    item.r_global = init_r;
    item.t_global = init_t;
    item.path_length = (idx == 0) ? 0.0 : m_key_poses.back().path_length + (cloud_with_pose.pose.t - m_key_poses.back().t_local).norm();
    m_key_poses.push_back(item);
    m_sc_manager.makeAndSaveScancontextAndKeys(*cloud_with_pose.cloud); // index-aligned with m_key_poses
    return true;
}

CloudType::Ptr SimplePGO::getSubMap(int idx, int half_range, double resolution)
{
    assert(idx >= 0 && idx < static_cast<int>(m_key_poses.size()));
    int min_idx = std::max(0, idx - half_range);
    int max_idx = std::min(static_cast<int>(m_key_poses.size()) - 1, idx + half_range);

    CloudType::Ptr ret(new CloudType);
    for (int i = min_idx; i <= max_idx; i++)
    {

        CloudType::Ptr body_cloud = m_key_poses[i].body_cloud;
        CloudType::Ptr global_cloud(new CloudType);
        pcl::transformPointCloud(*body_cloud, *global_cloud, m_key_poses[i].t_global, Eigen::Quaterniond(m_key_poses[i].r_global));
        *ret += *global_cloud;
    }
    if (resolution > 0)
    {
        pcl::VoxelGrid<PointType> voxel_grid;
        voxel_grid.setLeafSize(resolution, resolution, resolution);
        voxel_grid.setInputCloud(ret);
        voxel_grid.filter(*ret);
    }
    return ret;
}

void SimplePGO::searchForLoopPairs()
{
    if (m_key_poses.size() < 10)
        return;
    if (m_config.min_loop_detect_duration > 0.0)
    {
        if (m_history_pairs.size() > 0)
        {
            double current_time = m_key_poses.back().time;
            double last_time = m_key_poses[m_history_pairs.back().second].time;
            if (current_time - last_time < m_config.min_loop_detect_duration)
                return;
        }
    }

    size_t cur_idx = m_key_poses.size() - 1;
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    pcl::PointXYZ last_pose_pt;
    last_pose_pt.x = last_item.t_global(0);
    last_pose_pt.y = last_item.t_global(1);
    last_pose_pt.z = last_item.t_global(2);

    // Exclude recent keyframes from the radius search itself (mirrors
    // m_sc_manager's own num_exclude_recent for the ScanContext fallback
    // below). Without this, consecutive keyframes (only ~key_pose_delta_trans
    // apart, e.g. 0.5m) are ALWAYS within loop_search_radius (e.g. 1.0m) of
    // each other during ordinary continuous motion -- confirmed live: every
    // single cycle picked candidate=cur_idx-1 (or another keyframe just a
    // few steps back), which the travelled-path-length gate below then
    // (correctly) rejects every time. The real bug this caused: since that
    // trivial recent neighbor always satisfies "found a candidate"
    // (loop_idx != -1), the ScanContext fallback -- specifically built for
    // "revisits the geometric radius search misses due to accumulated drift"
    // -- was gated behind `if (loop_idx == -1)` and so almost NEVER actually
    // ran, even after a genuine drift-affected revisit. Now only keyframes
    // old enough to plausibly be a real revisit are eligible for the radius
    // search at all; ScanContext gets a real chance to fire otherwise.
    const bool have_old_keyframe = cur_idx > static_cast<size_t>(m_config.num_exclude_recent);
    const size_t max_idx_for_radius = have_old_keyframe ? (cur_idx - static_cast<size_t>(m_config.num_exclude_recent)) : 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; have_old_keyframe && i < max_idx_for_radius; i++)
    {
        pcl::PointXYZ pt;
        pt.x = m_key_poses[i].t_global(0);
        pt.y = m_key_poses[i].t_global(1);
        pt.z = m_key_poses[i].t_global(2);
        key_poses_cloud->push_back(pt);
    }
    int loop_idx = -1;
    if (!key_poses_cloud->empty())
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(key_poses_cloud);
        std::vector<int> ids;
        std::vector<float> sqdists;
        kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);

        // Candidate SELECTION here only needs to pick one of the spatially-nearby
        // keyframes the radius search returns; whether it's actually a valid loop
        // (not just a recently-visited spot) is decided uniformly below by the
        // path-length check, not by picking a specific candidate here. Prefer the
        // oldest (smallest index) as the most conservative choice.
        for (size_t i = 0; i < ids.size(); i++)
        {
            if (loop_idx == -1 || ids[i] < loop_idx)
                loop_idx = ids[i];
        }
    }

    // fallback: descriptor-based candidate (Scan Context), for revisits the geometric radius
    // search misses due to accumulated drift (no reliance on an accurate global position estimate)
    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    if (loop_idx == -1)
    {
        std::pair<int, float> sc_result = m_sc_manager.detectLoopClosureID();
        if (sc_result.first != -1)
        {
            loop_idx = sc_result.first;
            initial_guess.block<3, 3>(0, 0) = Eigen::AngleAxisf(sc_result.second, Eigen::Vector3f::UnitZ()).toRotationMatrix();
        }
    }

    // Neither the radius search above (gates on CURRENT spatial proximity +
    // wall-clock time) nor ScanContext's own NUM_EXCLUDE_RECENT (gates on
    // keyframe COUNT) actually verify the robot travelled away from this
    // candidate and back -- both stay satisfied trivially while the robot
    // idles, drives slowly, or rotates in place near the candidate (rotation
    // alone adds new keyframes via isKeyPose()'s angle check with ~zero
    // translation, so they're still co-located and pass a radius check
    // instantly). Require actual cumulative path length travelled since the
    // candidate to meaningfully exceed the search radius itself -- otherwise
    // the robot never really left this candidate's vicinity in the first
    // place, and accepting it just forces ISAM2 to warp the whole trajectory
    // to satisfy a constraint that wasn't a genuine revisit.
    if (loop_idx != -1)
    {
        const double travelled = m_key_poses[cur_idx].path_length - m_key_poses[loop_idx].path_length;
        if (travelled < 3.0 * m_config.loop_search_radius)
        {
            loop_idx = -1;
        }
    }

    if (loop_idx == -1)
        return;

    CloudType::Ptr target_cloud = getSubMap(loop_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
    CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0, m_config.submap_resolution);

    // Minimum-overlap sanity gate (matches LIO-SAM's detectLoopClosureDistance()/
    // performLoopClosure() in mapOptmization.cpp, which rejects a candidate
    // outright if either cloud is too sparse: "cureKeyframeCloud->size() < 300
    // || prevKeyframeCloud->size() < 1000"). Fixed, not exposed as yaml
    // parameters -- LIO-SAM hardcodes its equivalent directly in source too;
    // these are already scaled down from that for the QT64's sparser,
    // shorter-range scans and aren't course-specific.
    constexpr int kLoopMinSourcePoints = 50;
    constexpr int kLoopMinTargetPoints = 200;
    if (static_cast<int>(source_cloud->size()) < kLoopMinSourcePoints ||
        static_cast<int>(target_cloud->size()) < kLoopMinTargetPoints)
    {
        return;
    }

    m_icp.setInputSource(source_cloud);
    m_icp.setInputTarget(target_cloud);

    // NOTE: deliberately NOT gating on m_icp.hasConverged() here. For
    // fast_gicp (and PCL ICP in general), hasConverged() only reflects
    // whether the per-iteration transformation delta shrank below
    // transformationEpsilon within maximumIterations -- it is NOT a
    // fit-quality signal and can actively disagree with getFitnessScore()
    // (confirmed live: a candidate with fitness=0.0027, ~30x better than the
    // 0.08 threshold, was rejected solely because hasConverged()==false).
    // getFitnessScore() alone is the correct acceptance gate.
    CloudType::Ptr align_cloud(new CloudType);
    m_icp.align(*align_cloud, initial_guess);
    const double fitness = m_icp.getFitnessScore();
    M4F loop_transform = m_icp.getFinalTransformation();
    // Captured for the hybrid noise model in smoothAndUpdate() -- see
    // conditionLoopInformation().
    const Eigen::Matrix<double, 6, 6> best_hessian = m_icp.getFinalHessian();

    if (fitness > m_config.loop_score_tresh)
    {
        return;
    }

    // Compute the correction this candidate implies BEFORE gating, so the
    // consistency check below can compare it to the previous candidate's
    // correction (not just keyframe indices).
    M3D r_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].r_global;
    V3D t_refined = loop_transform.block<3, 3>(0, 0).cast<double>() * m_key_poses[cur_idx].t_global + loop_transform.block<3, 1>(0, 3).cast<double>();
    M3D r_offset = m_key_poses[loop_idx].r_global.transpose() * r_refined;
    V3D t_offset = m_key_poses[loop_idx].r_global.transpose() * (t_refined - m_key_poses[loop_idx].t_global);

    // Index-proximity alone (matching nearby target keyframes across
    // consecutive source keyframes) is not enough to rule out Scan Context
    // perceptual aliasing: a robot moving along a repeating/symmetric
    // structure (e.g. a small rectangular room) can keep matching the WRONG
    // wall consistently for several consecutive keyframes too, since the
    // aliasing itself tracks the robot's motion. This is the same failure
    // mode addressed by sequence-based place recognition (Milford & Wyeth,
    // "SeqSLAM", 2012) and by relative-pose/geometric consistency checks in
    // robust pose-graph pipelines: a GENUINE revisit implies a pose
    // correction (r_offset/t_offset) that stays essentially constant across
    // consecutive detections, because the real drift being corrected for
    // hasn't changed between two keyframes 1 spacing apart. An aliased match
    // against unrelated geometry has no reason to reproduce the same
    // correction each time. Tolerance is derived from the keyframe spacing
    // itself (a few multiples of it), not a new standalone tunable.
    const double trans_tolerance = 3.0 * m_config.key_pose_delta_trans;
    const double rot_tolerance_rad = 3.0 * m_config.key_pose_delta_deg * M_PI / 180.0;
    // Index tolerance is a secondary check (the trans/rot offset comparison
    // below is what actually verifies consistency); fixed since it's just a
    // "did we jump to a wildly different target keyframe" sanity bound, not
    // something that needs retuning per course.
    constexpr int kLoopConsistencyIndexTolerance = 2;
    // Was `cur_idx == m_pending_loop_source + 1` -- required the LITERAL
    // immediately-next keyframe to independently pass every upstream gate
    // (radius/ScanContext candidate found, path-length, submap size, ICP
    // fitness, ICP-vs-guess deviation) with zero misses, 3 times in a row,
    // before a loop closure could ever be accepted. Real sensor noise means
    // ICP fitness routinely fluctuates a little above/below threshold
    // frame-to-frame even for a genuine revisit -- a single missed keyframe
    // reset the whole count back to 1, which in practice meant closures
    // almost never fired even when the robot plainly returned to its start.
    // Allow a bounded gap of a few keyframes between hits instead (RTAB-Map/
    // pose-graph SLAM systems accumulate evidence for a revisit over a local
    // window, not literal adjacent-frame agreement) -- still requires the
    // SAME target (index tolerance) and a consistent implied correction
    // (trans/rot tolerance below), so this doesn't reopen the aliasing gap
    // this mechanism exists for, it just tolerates a few individual misses.
    constexpr size_t kLoopConsistencyMaxKeyframeGap = 4;
    bool candidate_is_consistent =
        m_pending_loop_count > 0 &&
        (cur_idx - m_pending_loop_source) <= kLoopConsistencyMaxKeyframeGap &&
        std::abs(loop_idx - m_pending_loop_target) <= kLoopConsistencyIndexTolerance;
    if (candidate_is_consistent)
    {
        const double trans_delta = (t_offset - m_pending_loop_t_offset).norm();
        const double rot_delta = Eigen::Quaterniond(r_offset).angularDistance(Eigen::Quaterniond(m_pending_loop_r_offset));
        candidate_is_consistent = trans_delta <= trans_tolerance && rot_delta <= rot_tolerance_rad;
    }
    m_pending_loop_count = candidate_is_consistent ? m_pending_loop_count + 1 : 1;
    m_pending_loop_source = cur_idx;
    m_pending_loop_target = loop_idx;
    m_pending_loop_r_offset = r_offset;
    m_pending_loop_t_offset = t_offset;
    if (m_pending_loop_count < m_config.loop_consistency_count)
    {
        return;
    }
    m_pending_loop_count = 0;

    LoopPair one_pair;
    one_pair.source_id = cur_idx;
    one_pair.target_id = loop_idx;
    one_pair.score = fitness;
    one_pair.r_offset = r_offset;
    one_pair.t_offset = t_offset;
    one_pair.hessian = best_hessian;
    m_cache_pairs.push_back(one_pair);
    m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
    RCLCPP_WARN(rclcpp::get_logger("loop_pgo"),
                "[Loop found] source=%d target=%d fitness=%.4f", static_cast<int>(cur_idx), loop_idx, one_pair.score);
}

void SimplePGO::smoothAndUpdate()
{
    bool has_loop = !m_cache_pairs.empty();
    // 添加回环因子
    if (has_loop)
    {
        for (LoopPair &pair : m_cache_pairs)
        {
            // Hybrid noise model -- see conditionLoopInformation(): fitness
            // score sets an isotropic floor everywhere (same baseline
            // LIO-SAM/SC-A-LOAM use), the Hessian's eigen-directions only
            // loosen genuinely degenerate directions beyond that floor, never
            // tighten below it.
            const double base_variance = std::max(pair.score, 1e-4);
            const Eigen::Matrix<double, 6, 6> information =
                conditionLoopInformation(pair.hessian, base_variance, m_config);
            auto gaussian_noise = gtsam::noiseModel::Gaussian::Information(information);
            // 1.345 is the standard Huber constant (95% efficiency under
            // Gaussian noise) -- a textbook value, not a per-course tunable.
            auto huber_loss = gtsam::noiseModel::mEstimator::Huber::Create(1.345);
            auto robust_noise = gtsam::noiseModel::Robust::Create(huber_loss, gaussian_noise);
            m_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(pair.target_id, pair.source_id,
                                                           gtsam::Pose3(gtsam::Rot3(pair.r_offset),
                                                                        gtsam::Point3(pair.t_offset)),
                                                           robust_noise));
        }
        std::vector<LoopPair>().swap(m_cache_pairs);
    }
    // smooth and mapping
    m_isam2->update(m_graph, m_initial_values);
    m_isam2->update();
    if (has_loop)
    {
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
        m_isam2->update();
    }
    m_graph.resize(0);
    m_initial_values.clear();

    // update key poses
    gtsam::Values estimate_values = m_isam2->calculateBestEstimate();
    for (size_t i = 0; i < m_key_poses.size(); i++)
    {
        gtsam::Pose3 pose = estimate_values.at<gtsam::Pose3>(i);
        m_key_poses[i].r_global = pose.rotation().matrix().cast<double>();
        m_key_poses[i].t_global = pose.translation().matrix().cast<double>();
    }
    // update offset
    const KeyPoseWithCloud &last_item = m_key_poses.back();
    m_r_offset = last_item.r_global * last_item.r_local.transpose();
    m_t_offset = last_item.t_global - m_r_offset * last_item.t_local;
}