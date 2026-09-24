#include "simple_pgo.h"

namespace
{
Eigen::Matrix<double, 6, 6> conditionLoopInformation(
    const Eigen::Matrix<double, 6, 6> &hessian, const Config &config)
{
    const Eigen::Matrix<double, 6, 6> symmetric_hessian = 0.5 * (hessian + hessian.transpose());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(symmetric_hessian);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
        return Eigen::Matrix<double, 6, 6>::Identity() * config.hessian_min_information;

    Eigen::Matrix<double, 6, 1> eigenvalues = solver.eigenvalues();
    const double max_eigenvalue = eigenvalues.maxCoeff();
    if (!std::isfinite(max_eigenvalue) || max_eigenvalue <= config.hessian_min_information)
        return Eigen::Matrix<double, 6, 6>::Identity() * config.hessian_min_information;

    for (int i = 0; i < eigenvalues.size(); ++i)
    {
        if (eigenvalues(i) <= 0.0)
        {
            eigenvalues(i) = config.hessian_min_information;
        }
        else if (eigenvalues(i) / max_eigenvalue < config.hessian_eigen_ratio_threshold)
        {
            eigenvalues(i) = std::max(config.hessian_min_information,
                                      eigenvalues(i) * config.hessian_degenerate_scale);
        }
    }

    return solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
}
}

SimplePGO::SimplePGO(const Config &config) : m_config(config)
{
    m_sc_manager.setNumExcludeRecent(config.num_exclude_recent);
    m_sc_manager.setSCDistThres(config.sc_dist_thres);
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
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
    m_icp.setMaxCorrespondenceDistance(3.0);
    m_icp.setNumThreads(0); // 0 = use all available threads
    m_icp.setCorrespondenceRandomness(20);
    m_icp.setRegularizationMethod(fast_gicp::RegularizationMethod::PLANE);
    m_icp.setMaximumIterations(50);
    m_icp.setTransformationEpsilon(1e-6);
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
        // 添加里程计约束
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
        const double trans_sigma = std::max(m_config.odom_trans_noise_floor, m_config.odom_trans_noise_per_meter * dtrans);
        const double rot_sigma = std::max(m_config.odom_rot_noise_floor, m_config.odom_rot_noise_per_rad * drot);
        const double z_sigma = m_config.odom_z_noise_floor;
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr key_poses_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < m_key_poses.size() - 1; i++)
    {
        pcl::PointXYZ pt;
        pt.x = m_key_poses[i].t_global(0);
        pt.y = m_key_poses[i].t_global(1);
        pt.z = m_key_poses[i].t_global(2);
        key_poses_cloud->push_back(pt);
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(key_poses_cloud);
    std::vector<int> ids;
    std::vector<float> sqdists;
    kdtree.radiusSearch(last_pose_pt, m_config.loop_search_radius, ids, sqdists);

    int loop_idx = -1;
    for (size_t i = 0; i < ids.size(); i++)
    {
        int idx = ids[i];
        if (std::abs(last_item.time - m_key_poses[idx].time) > m_config.loop_time_tresh)
        {
            loop_idx = idx;
            break;
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
            const M3D r_guess = Eigen::AngleAxisd(
                static_cast<double>(sc_result.second), Eigen::Vector3d::UnitZ()).toRotationMatrix();
            initial_guess.block<3, 3>(0, 0) = r_guess.cast<float>();
            // Unlike the radius-search branch above (whose candidate is, by
            // construction, within loop_search_radius of last_item in the
            // CURRENT global estimate, so identity is a fair starting guess),
            // a Scan Context candidate exists precisely BECAUSE drift has
            // pushed these two keyframes' current global positions apart --
            // leaving the translation at identity/zero assumes source and
            // target already overlap in world coordinates, which is normally
            // false here. ICP then only finds whatever sparse correspondences
            // happen to fall within setMaxCorrespondenceDistance of that
            // wrong start and can still "converge" with a passable fitness
            // score despite the two submaps not actually overlapping --
            // exactly what shows up downstream as an accepted loop whose
            // start/end scans visibly don't align. Seed translation from the
            // current global position delta between the two keyframes
            // (target - R*source, consistent with how align()'s guess is
            // applied: p_target ~= R*p_source + t).
            const V3D t_guess = m_key_poses[loop_idx].t_global - r_guess * last_item.t_global;
            initial_guess.block<3, 1>(0, 3) = t_guess.cast<float>();
        }
    }

    if (loop_idx == -1)
        return;

    CloudType::Ptr target_cloud = getSubMap(loop_idx, m_config.loop_submap_half_range, m_config.submap_resolution);
    CloudType::Ptr source_cloud = getSubMap(m_key_poses.size() - 1, 0, m_config.submap_resolution);

    // Minimum-overlap sanity gate (matches LIO-SAM's detectLoopClosureDistance()/
    // performLoopClosure() in mapOptmization.cpp, which rejects a candidate
    // outright if either cloud is too sparse: "cureKeyframeCloud->size() < 300
    // || prevKeyframeCloud->size() < 1000"). Without this, a near-empty or
    // feature-poor submap can still report ICP convergence with a passable
    // fitness score purely because it has very few, trivially-satisfied
    // correspondences -- indistinguishable downstream from a genuine match.
    if (static_cast<int>(source_cloud->size()) < m_config.loop_min_source_points ||
        static_cast<int>(target_cloud->size()) < m_config.loop_min_target_points)
        return;

    CloudType::Ptr align_cloud(new CloudType);

    m_icp.setInputSource(source_cloud);
    m_icp.setInputTarget(target_cloud);
    m_icp.align(*align_cloud, initial_guess);

    if (!m_icp.hasConverged() || m_icp.getFitnessScore() > m_config.loop_score_tresh)
        return;

    // Compute the correction this candidate implies BEFORE gating, so the
    // consistency check below can compare it to the previous candidate's
    // correction (not just keyframe indices).
    M4F loop_transform = m_icp.getFinalTransformation();
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
    bool candidate_is_consistent =
        m_pending_loop_count > 0 && cur_idx == m_pending_loop_source + 1 &&
        std::abs(loop_idx - m_pending_loop_target) <= m_config.loop_consistency_target_tolerance;
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
        return;
    m_pending_loop_count = 0;

    LoopPair one_pair;
    one_pair.source_id = cur_idx;
    one_pair.target_id = loop_idx;
    one_pair.score = m_icp.getFitnessScore();
    one_pair.information = conditionLoopInformation(m_icp.getFinalHessian(), m_config);
    one_pair.r_offset = r_offset;
    one_pair.t_offset = t_offset;
    m_cache_pairs.push_back(one_pair);
    m_history_pairs.emplace_back(one_pair.target_id, one_pair.source_id);
}

void SimplePGO::smoothAndUpdate()
{
    bool has_loop = !m_cache_pairs.empty();
    // 添加回环因子
    if (has_loop)
    {
        for (LoopPair &pair : m_cache_pairs)
        {
            auto gaussian_noise = gtsam::noiseModel::Gaussian::Information(pair.information);
            auto huber_loss = gtsam::noiseModel::mEstimator::Huber::Create(m_config.loop_huber_k);
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