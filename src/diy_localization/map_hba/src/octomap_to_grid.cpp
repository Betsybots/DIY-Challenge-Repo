// Offline batch tool: keyframe patches + poses -> OctoMap -> 2D occupancy grid.
//
// Consumes the exact output of loop_pgo's `/loop_pgo/save_maps` service
// (called with save_patches:true):
//   <maps_path>/patches/<i>.pcd   body-frame keyframe scans
//   <maps_path>/poses.txt         "<file_name> tx ty tz qw qx qy qz" per line
// or map_hba's own `/map_hba/save_poses` output (refined_poses.txt, same format,
// same patches/ directory, BA-optimized translations/rotations).
//
// Each patch is ray-cast into an octomap::OcTree from its keyframe's sensor
// origin, so free space is genuinely carved out (not just "occupied where a
// point landed") and repeated observations of the same voxel are fused
// probabilistically, which is more robust to sensor noise than a plain
// point-count threshold. The tree is then projected, within a configurable
// height band, into a 2D occupancy grid and written as a nav2_map_server-
// compatible map.pgm + map.yaml.
//
// Usage:
//   ros2 run map_hba octomap_to_grid --config src/map_hba/config/octomap_to_grid.yaml
//
//   All settings can also be passed/overridden as CLI flags, e.g. for quick
//   one-off tuning without editing the yaml:
//   ros2 run map_hba octomap_to_grid --config <yaml> --resolution 0.1 --z-max 0.8
//
//   Or fully via CLI, no yaml at all:
//   ros2 run map_hba octomap_to_grid --maps-path src/map --out src/map/global_map

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace
{

struct KeyframePose
{
    std::string file_name;
    Eigen::Vector3d t;
    Eigen::Matrix3d r;
};

struct Options
{
    std::string maps_path;
    std::string poses_file = "poses.txt";
    std::string out_prefix;
    double resolution = 0.05;
    double z_min = -0.3;
    double z_max = 0.5;
    bool save_bt = false;
};

enum class CellState : uint8_t
{
    UNKNOWN,
    FREE,
    OCCUPIED
};

void printUsage(const char *prog)
{
    std::cerr << "Usage: " << prog << " [--config <yaml>] [--maps-path <dir>] [--out <prefix>] "
              << "[--poses-file poses.txt] [--resolution 0.05] "
              << "[--z-min -0.3] [--z-max 0.5] [--save-bt]\n"
              << "  --config loads defaults from a yaml file (see "
              << "config/octomap_to_grid.yaml); any CLI flag overrides it.\n";
}

// Loads whichever of maps_path/poses_file/out_prefix/resolution/z_min/z_max/
// save_bt are present in the yaml, leaving Options' existing values (built-in
// defaults, or values set by an earlier --config) untouched otherwise.
bool loadYamlConfig(const std::string &path, Options &opt)
{
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(path);
    }
    catch (const std::exception &e)
    {
        std::cerr << "[ERROR] failed to load config " << path << ": " << e.what() << "\n";
        return false;
    }
    if (!config)
    {
        std::cerr << "[ERROR] config file is empty or invalid: " << path << "\n";
        return false;
    }
    if (config["maps_path"])
        opt.maps_path = config["maps_path"].as<std::string>();
    if (config["poses_file"])
        opt.poses_file = config["poses_file"].as<std::string>();
    if (config["out_prefix"])
        opt.out_prefix = config["out_prefix"].as<std::string>();
    if (config["resolution"])
        opt.resolution = config["resolution"].as<double>();
    if (config["z_min"])
        opt.z_min = config["z_min"].as<double>();
    if (config["z_max"])
        opt.z_max = config["z_max"].as<double>();
    if (config["save_bt"])
        opt.save_bt = config["save_bt"].as<bool>();
    return true;
}

bool parseArgs(int argc, char **argv, Options &opt)
{
    // Phase 1: if --config is present, load it first so phase 2's CLI flags
    // can still override individual values for quick one-off testing.
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]) == "--config")
        {
            if (i + 1 >= argc)
            {
                std::cerr << "[ERROR] missing value for --config\n";
                return false;
            }
            if (!loadYamlConfig(argv[i + 1], opt))
                return false;
            break;
        }
    }

    // Phase 2: CLI flags override whatever came from the yaml (or the
    // built-in defaults in Options if there was no --config at all).
    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        auto next = [&](const char *name) -> std::string
        {
            if (i + 1 >= argc)
            {
                std::cerr << "[ERROR] missing value for " << name << "\n";
                std::exit(1);
            }
            return argv[++i];
        };
        if (arg == "--config")
            next("--config"); // already handled in phase 1, just skip its value
        else if (arg == "--maps-path")
            opt.maps_path = next("--maps-path");
        else if (arg == "--poses-file")
            opt.poses_file = next("--poses-file");
        else if (arg == "--out")
            opt.out_prefix = next("--out");
        else if (arg == "--resolution")
            opt.resolution = std::stod(next("--resolution"));
        else if (arg == "--z-min")
            opt.z_min = std::stod(next("--z-min"));
        else if (arg == "--z-max")
            opt.z_max = std::stod(next("--z-max"));
        else if (arg == "--save-bt")
            opt.save_bt = true;
        else if (arg == "-h" || arg == "--help")
        {
            printUsage(argv[0]);
            std::exit(0);
        }
        else
        {
            std::cerr << "[ERROR] unknown argument: " << arg << "\n";
            return false;
        }
    }
    if (opt.maps_path.empty() || opt.out_prefix.empty())
    {
        std::cerr << "[ERROR] maps_path and out_prefix are required "
                  << "(set them in --config yaml, or pass --maps-path/--out)\n";
        return false;
    }
    if (opt.z_max <= opt.z_min)
    {
        std::cerr << "[ERROR] z_max must be greater than z_min\n";
        return false;
    }
    return true;
}

// Parses one poses.txt / refined_poses.txt line, matching the format written
// by loop_pgo/map_hba: "<file_name> tx ty tz qw qx qy qz"
bool parsePoseLine(const std::string &line, KeyframePose &kp)
{
    std::stringstream ss(line);
    std::vector<std::string> tokens;
    std::string tok;
    while (std::getline(ss, tok, ' '))
        if (!tok.empty())
            tokens.push_back(tok);
    if (tokens.size() != 8)
        return false;
    kp.file_name = tokens[0];
    kp.t = Eigen::Vector3d(std::stod(tokens[1]), std::stod(tokens[2]), std::stod(tokens[3]));
    Eigen::Quaterniond q(std::stod(tokens[4]), std::stod(tokens[5]), std::stod(tokens[6]), std::stod(tokens[7]));
    kp.r = q.normalized().toRotationMatrix();
    return true;
}

} // namespace

int main(int argc, char **argv)
{
    Options opt;
    if (!parseArgs(argc, argv, opt))
    {
        printUsage(argv[0]);
        return 1;
    }

    const fs::path maps_dir(opt.maps_path);
    const fs::path patches_dir = maps_dir / "patches";
    const fs::path poses_path = fs::path(opt.poses_file).is_absolute()
                                     ? fs::path(opt.poses_file)
                                     : maps_dir / opt.poses_file;

    if (!fs::exists(patches_dir))
    {
        std::cerr << "[ERROR] " << patches_dir << " does not exist. Run loop_pgo's "
                  << "/loop_pgo/save_maps service with save_patches:true first.\n";
        return 1;
    }
    if (!fs::exists(poses_path))
    {
        std::cerr << "[ERROR] " << poses_path << " does not exist.\n";
        return 1;
    }

    std::ifstream ifs(poses_path.string());
    if (!ifs)
    {
        std::cerr << "[ERROR] failed to open " << poses_path << "\n";
        return 1;
    }

    octomap::OcTree tree(opt.resolution);

    std::string line;
    size_t keyframe_count = 0;
    size_t point_count = 0;
    while (std::getline(ifs, line))
    {
        if (line.empty())
            continue;
        KeyframePose kp;
        if (!parsePoseLine(line, kp))
        {
            std::cerr << "[WARN] skipping malformed pose line: " << line << "\n";
            continue;
        }

        const fs::path pcd_file = patches_dir / kp.file_name;
        if (!fs::exists(pcd_file))
        {
            std::cerr << "[WARN] missing patch, skipping: " << pcd_file << "\n";
            continue;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile(pcd_file.string(), *cloud) != 0)
        {
            std::cerr << "[WARN] failed to load " << pcd_file << ", skipping\n";
            continue;
        }

        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.linear() = kp.r;
        transform.translation() = kp.t;
        pcl::PointCloud<pcl::PointXYZI>::Ptr world_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*cloud, *world_cloud, transform);

        octomap::Pointcloud scan;
        scan.reserve(world_cloud->size());
        for (const auto &p : world_cloud->points)
        {
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;
            scan.push_back(p.x, p.y, p.z);
        }

        const octomap::point3d origin(static_cast<float>(kp.t.x()),
                                       static_cast<float>(kp.t.y()),
                                       static_cast<float>(kp.t.z()));
        tree.insertPointCloud(scan, origin);

        ++keyframe_count;
        point_count += scan.size();
        std::cout << "[INFO] inserted keyframe " << kp.file_name << " (" << scan.size() << " pts)\n";
    }

    if (keyframe_count == 0)
    {
        std::cerr << "[ERROR] no keyframes were inserted; nothing to write\n";
        return 1;
    }

    tree.updateInnerOccupancy();
    std::cout << "[INFO] " << keyframe_count << " keyframes, " << point_count
              << " points ray-cast into octree (resolution " << opt.resolution << " m)\n";

    if (opt.save_bt)
    {
        const std::string bt_path = opt.out_prefix + ".bt";
        tree.writeBinary(bt_path);
        std::cout << "[INFO] wrote octree: " << bt_path << "\n";
    }

    // ---- project to 2D occupancy grid within [z_min, z_max] ----
    double min_x, min_y, min_z, max_x, max_y, max_z;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);

    const double res = opt.resolution;
    const int width = std::max(1, static_cast<int>(std::ceil((max_x - min_x) / res)) + 1);
    const int height = std::max(1, static_cast<int>(std::ceil((max_y - min_y) / res)) + 1);

    std::vector<CellState> grid(static_cast<size_t>(width) * static_cast<size_t>(height), CellState::UNKNOWN);
    auto cellIndex = [&](int gx, int gy)
    { return static_cast<size_t>(gy) * static_cast<size_t>(width) + static_cast<size_t>(gx); };

    // Iterate actual (possibly pruned/coarser) leafs and splat each one across
    // every grid cell it covers -- this handles octomap's multi-resolution
    // leafs correctly without needing tree.expand() (which would blow up
    // memory for a large outdoor map at fine resolution).
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it)
    {
        const double z = it.getZ();
        if (z < opt.z_min || z > opt.z_max)
            continue;

        const bool occupied = tree.isNodeOccupied(*it);
        const double half = it.getSize() / 2.0;
        int gx0 = static_cast<int>(std::floor((it.getX() - half - min_x) / res));
        int gx1 = static_cast<int>(std::floor((it.getX() + half - min_x) / res));
        int gy0 = static_cast<int>(std::floor((it.getY() - half - min_y) / res));
        int gy1 = static_cast<int>(std::floor((it.getY() + half - min_y) / res));
        gx0 = std::clamp(gx0, 0, width - 1);
        gx1 = std::clamp(gx1, 0, width - 1);
        gy0 = std::clamp(gy0, 0, height - 1);
        gy1 = std::clamp(gy1, 0, height - 1);

        for (int gy = gy0; gy <= gy1; ++gy)
        {
            for (int gx = gx0; gx <= gx1; ++gx)
            {
                CellState &cell = grid[cellIndex(gx, gy)];
                if (occupied)
                    cell = CellState::OCCUPIED; // occupied always wins
                else if (cell != CellState::OCCUPIED)
                    cell = CellState::FREE;
            }
        }
    }

    size_t n_occ = 0, n_free = 0, n_unknown = 0;
    for (auto c : grid)
    {
        if (c == CellState::OCCUPIED)
            ++n_occ;
        else if (c == CellState::FREE)
            ++n_free;
        else
            ++n_unknown;
    }
    std::cout << "[INFO] grid " << width << "x" << height << " @ " << res
              << " m/cell -> occupied=" << n_occ << " free=" << n_free
              << " unknown=" << n_unknown << "\n";

    const fs::path out_prefix(opt.out_prefix);
    if (out_prefix.has_parent_path())
        fs::create_directories(out_prefix.parent_path());

    // ---- write PGM (binary P5). Image row 0 = top = max Y, matching the
    // nav2_map_server convention that `origin` is the map-frame pose of the
    // image's bottom-left pixel. ----
    const std::string pgm_path = opt.out_prefix + ".pgm";
    std::ofstream pgm(pgm_path, std::ios::binary);
    if (!pgm)
    {
        std::cerr << "[ERROR] failed to open " << pgm_path << " for writing\n";
        return 1;
    }
    pgm << "P5\n" << width << " " << height << "\n255\n";
    for (int gy = height - 1; gy >= 0; --gy)
    {
        for (int gx = 0; gx < width; ++gx)
        {
            uint8_t pixel;
            switch (grid[cellIndex(gx, gy)])
            {
            case CellState::OCCUPIED:
                pixel = 0; // black
                break;
            case CellState::FREE:
                pixel = 254; // near-white
                break;
            default:
                pixel = 205; // gray = unknown
                break;
            }
            pgm.put(static_cast<char>(pixel));
        }
    }
    pgm.close();

    // ---- write YAML (nav2_map_server schema) ----
    const std::string yaml_path = opt.out_prefix + ".yaml";
    std::ofstream yaml(yaml_path);
    if (!yaml)
    {
        std::cerr << "[ERROR] failed to open " << yaml_path << " for writing\n";
        return 1;
    }
    yaml << "image: " << out_prefix.filename().string() << ".pgm\n"
         << "resolution: " << res << "\n"
         << "origin: [" << min_x << ", " << min_y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";
    yaml.close();

    std::cout << "[INFO] wrote " << pgm_path << " and " << yaml_path << "\n";
    return 0;
}
