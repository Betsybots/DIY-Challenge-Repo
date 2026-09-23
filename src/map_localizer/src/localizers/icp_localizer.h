#pragma once
#include "commons.h"
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <fast_gicp/gicp/fast_vgicp.hpp>

struct ICPConfig
{
    double refine_scan_resolution = 0.1;
    double refine_map_resolution = 0.1;
    double refine_score_thresh = 0.1;
    int refine_max_iteration = 10;
    // VGICP's own internal voxel covariance grid, separate from the scan/map
    // downsample resolution above.
    double refine_vgicp_resolution = 0.5;

    double rough_scan_resolution = 0.25;
    double rough_map_resolution = 0.25;
    double rough_score_thresh = 0.2;
    int rough_max_iteration = 5;
    double rough_vgicp_resolution = 1.0;

    int num_threads = 4;
};

class ICPLocalizer
{
public:
    ICPLocalizer(const ICPConfig &config);

    bool loadMap(const std::string &path);

    void setInput(const CloudType::Ptr &cloud);

    bool align(M4F &guess);
    ICPConfig &config() { return m_config; }
    CloudType::Ptr roughMap() { return m_rough_tgt; }
    CloudType::Ptr refineMap() { return m_refine_tgt; }
    // Fitness scores from the most recent align() call (rough/refine ICP's
    // getFitnessScore(), lower is better), regardless of whether that call
    // returned true. -1.0 means the corresponding ICP stage never ran (e.g.
    // no map loaded yet, or the rough stage already failed). Exposed so
    // callers can log *why* an alignment failed instead of only the bool.
    double lastRoughFitness() const { return m_last_rough_fitness; }
    double lastRefineFitness() const { return m_last_refine_fitness; }

private:
    ICPConfig m_config;
    double m_last_rough_fitness = -1.0;
    double m_last_refine_fitness = -1.0;
    pcl::VoxelGrid<PointType> m_voxel_filter;
    fast_gicp::FastVGICP<PointType, PointType> m_refine_icp;
    fast_gicp::FastVGICP<PointType, PointType> m_rough_icp;
    CloudType::Ptr m_refine_inp;
    CloudType::Ptr m_rough_inp;
    CloudType::Ptr m_refine_tgt;
    CloudType::Ptr m_rough_tgt;
    std::string m_pcd_path;
};
