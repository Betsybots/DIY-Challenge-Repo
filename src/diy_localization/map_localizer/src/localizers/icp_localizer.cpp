#include "icp_localizer.h"

ICPLocalizer::ICPLocalizer(const ICPConfig &config) : m_config(config)
{
    m_refine_inp.reset(new CloudType);
    m_refine_tgt.reset(new CloudType);
    m_rough_inp.reset(new CloudType);
    m_rough_tgt.reset(new CloudType);

    m_refine_icp.setNumThreads(m_config.num_threads);
    m_refine_icp.setResolution(m_config.refine_vgicp_resolution);
    m_refine_icp.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);

    m_rough_icp.setNumThreads(m_config.num_threads);
    m_rough_icp.setResolution(m_config.rough_vgicp_resolution);
    m_rough_icp.setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
}
bool ICPLocalizer::loadMap(const std::string &path)
{
    if (!std::filesystem::exists(path))
    {
        std::cerr << "Map file not found: " << path << std::endl;
        return false;
    }
    pcl::PCDReader reader;
    CloudType::Ptr cloud(new CloudType);
    reader.read(path, *cloud);
    // Fresh buffers each load -- FastGICP caches its KdTree/covariances by
    // pointer identity, so reusing the same Ptr while its contents change
    // (e.g. reloading via /relocalize) would leave it searching a stale index.
    m_refine_tgt = std::make_shared<CloudType>();
    m_rough_tgt = std::make_shared<CloudType>();
    if (m_config.refine_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_map_resolution, m_config.refine_map_resolution, m_config.refine_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_tgt);
    }

    if (m_config.rough_map_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_map_resolution, m_config.rough_map_resolution, m_config.rough_map_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_tgt);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_tgt);
    }
    return true;
}
void ICPLocalizer::setInput(const CloudType::Ptr &cloud)
{
    // Same reasoning as loadMap(): a new Ptr per frame forces FastGICP to see
    // a changed input and rebuild its KdTree, instead of reusing a stale one
    // sized for whatever point count the previous scan downsampled to.
    m_refine_inp = std::make_shared<CloudType>();
    m_rough_inp = std::make_shared<CloudType>();
    if (m_config.refine_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.refine_scan_resolution, m_config.refine_scan_resolution, m_config.refine_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_refine_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_refine_inp);
    }

    if (m_config.rough_scan_resolution > 0)
    {
        m_voxel_filter.setLeafSize(m_config.rough_scan_resolution, m_config.rough_scan_resolution, m_config.rough_scan_resolution);
        m_voxel_filter.setInputCloud(cloud);
        m_voxel_filter.filter(*m_rough_inp);
    }
    else
    {
        pcl::copyPointCloud(*cloud, *m_rough_inp);
    }
}

bool ICPLocalizer::align(M4F &guess)
{
    CloudType::Ptr aligned_cloud(new CloudType);
    m_last_refine_converged = false;
    m_last_refine_fitness = -1.0;
    if (m_refine_tgt->size() == 0 || m_rough_tgt->size() == 0)
    {
        m_last_rough_converged = false;
        m_last_rough_fitness = -1.0;
        return false;
    }
    m_rough_icp.setMaximumIterations(m_config.rough_max_iteration);
    m_rough_icp.setInputSource(m_rough_inp);
    m_rough_icp.setInputTarget(m_rough_tgt);
    m_rough_icp.align(*aligned_cloud, guess);
    m_last_rough_converged = m_rough_icp.hasConverged();
    m_last_rough_fitness = m_rough_icp.getFitnessScore();
    // hasConverged() reflects fast_gicp's internal iteration-delta epsilon,
    // not fit quality -- observed in practice to actively disagree with the
    // fitness score (a "converged" run scoring worse than a "not converged"
    // one on the very next scan). Gate on fitness score alone; converged
    // flags are still recorded above for diagnostics/logging.
    if (m_last_rough_fitness > m_config.rough_score_thresh)
        return false;
    m_refine_icp.setMaximumIterations(m_config.refine_max_iteration);
    m_refine_icp.setInputSource(m_refine_inp);
    m_refine_icp.setInputTarget(m_refine_tgt);
    m_refine_icp.align(*aligned_cloud, m_rough_icp.getFinalTransformation());
    m_last_refine_converged = m_refine_icp.hasConverged();
    m_last_refine_fitness = m_refine_icp.getFitnessScore();
    if (m_last_refine_fitness > m_config.refine_score_thresh)
        return false;
    guess = m_refine_icp.getFinalTransformation();
    m_last_fitness_score = m_last_refine_fitness;
    return true;
}
