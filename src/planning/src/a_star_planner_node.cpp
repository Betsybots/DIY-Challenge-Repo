// Fast A* global planner for the DIY Challenge robot.
//
// Nav2 GlobalPlanner plugin implementation. See a_star_planner_node.hpp.

#include "planning/a_star_planner_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace diy_astar_planner
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = node_.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".robot_clearance", rclcpp::ParameterValue(robot_clearance_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".occupied_threshold", rclcpp::ParameterValue(occupied_threshold_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".unknown_is_occupied", rclcpp::ParameterValue(unknown_is_occupied_));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".visited_publish_interval",
    rclcpp::ParameterValue(visited_publish_interval_));

  node->get_parameter(name_ + ".robot_clearance", robot_clearance_);
  node->get_parameter(name_ + ".occupied_threshold", occupied_threshold_);
  node->get_parameter(name_ + ".unknown_is_occupied", unknown_is_occupied_);
  node->get_parameter(name_ + ".visited_publish_interval", visited_publish_interval_);

  if (visited_publish_interval_ < 1) {
    visited_publish_interval_ = 1;
  }

  visited_map_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    name_ + "/visited_map", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(logger_, "Configured A* planner \"%s\"", name_.c_str());
  RCLCPP_INFO(logger_, "  robot_clearance = %.2f m", robot_clearance_);
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up A* planner \"%s\"", name_.c_str());
  visited_map_pub_.reset();
  safe_grid_.clear();
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating A* planner \"%s\"", name_.c_str());
  if (visited_map_pub_) {
    visited_map_pub_->on_activate();
  }
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating A* planner \"%s\"", name_.c_str());
  if (visited_map_pub_) {
    visited_map_pub_->on_deactivate();
  }
}

// ================================================================
// Precompute inflated obstacle map from the live costmap
// ================================================================

void AStarPlanner::buildSafeGrid()
{
  const int width = static_cast<int>(costmap_->getSizeInCellsX());
  const int height = static_cast<int>(costmap_->getSizeInCellsY());
  const size_t total_cells = static_cast<size_t>(width) * height;

  clearance_cells_ = static_cast<int>(
    std::ceil(robot_clearance_ / costmap_->getResolution()));

  safe_grid_.assign(total_cells, true);

  std::vector<std::pair<int, int>> blocked_cells;

  for (int y = 0; y < height; ++y) {
    const int row_offset = y * width;
    for (int x = 0; x < width; ++x) {
      const unsigned char value = costmap_->getCost(x, y);

      bool blocked = false;
      if (value == nav2_costmap_2d::NO_INFORMATION) {
        if (unknown_is_occupied_) {
          blocked = true;
        }
      } else if (value >= occupied_threshold_) {
        blocked = true;
      }

      if (blocked) {
        safe_grid_[row_offset + x] = false;
        blocked_cells.emplace_back(x, y);
      }
    }
  }

  // Precompute circular inflation offsets once.
  const int radius = clearance_cells_;
  const int radius_squared = radius * radius;
  std::vector<std::pair<int, int>> inflation_offsets;

  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      if (dx * dx + dy * dy <= radius_squared) {
        inflation_offsets.emplace_back(dx, dy);
      }
    }
  }

  // Inflate every blocked cell.
  for (const auto & obstacle : blocked_cells) {
    for (const auto & offset : inflation_offsets) {
      const int nx = obstacle.first + offset.first;
      const int ny = obstacle.second + offset.second;

      if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
        safe_grid_[ny * width + nx] = false;
      }
    }
  }

  // Treat borders inside clearance radius as unsafe (outside-map space is blocked).
  if (radius > 0) {
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (x < radius || y < radius || x >= width - radius || y >= height - radius) {
          safe_grid_[y * width + x] = false;
        }
      }
    }
  }
}

// ================================================================
// Fast A*
// ================================================================

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = clock_->now();

  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      logger_, "Start/goal frame must be '%s' (got '%s' / '%s')",
      global_frame_.c_str(), start.header.frame_id.c_str(), goal.header.frame_id.c_str());
    return path;
  }

  buildSafeGrid();

  visited_map_.header.frame_id = global_frame_;
  visited_map_.info.width = costmap_->getSizeInCellsX();
  visited_map_.info.height = costmap_->getSizeInCellsY();
  visited_map_.info.resolution = costmap_->getResolution();
  visited_map_.info.origin.position.x = costmap_->getOriginX();
  visited_map_.info.origin.position.y = costmap_->getOriginY();
  visited_map_.data.assign(safe_grid_.size(), 0);

  const GraphNode start_node = worldToGrid(start.pose);
  const GraphNode goal_node = worldToGrid(goal.pose);

  if (!poseOnMap(start_node)) {
    RCLCPP_ERROR(logger_, "Start outside map");
    return path;
  }
  if (!poseOnMap(goal_node)) {
    RCLCPP_ERROR(logger_, "Goal outside map");
    return path;
  }
  if (!isSafe(start_node)) {
    RCLCPP_ERROR(logger_, "Start is inside inflated obstacle region");
    return path;
  }
  if (!isSafe(goal_node)) {
    RCLCPP_ERROR(logger_, "Goal is inside inflated obstacle region");
    return path;
  }

  static const std::vector<std::tuple<int, int, double>> directions = {
    {-1, 0, 1.0}, {1, 0, 1.0}, {0, -1, 1.0}, {0, 1, 1.0},
    {-1, -1, std::sqrt(2.0)}, {-1, 1, std::sqrt(2.0)},
    {1, -1, std::sqrt(2.0)}, {1, 1, std::sqrt(2.0)},
  };

  const int width = static_cast<int>(costmap_->getSizeInCellsX());

  const auto key_of = [width](int x, int y) {
      return static_cast<int64_t>(y) * width + x;
    };

  const int64_t start_key = key_of(start_node.x, start_node.y);
  const int64_t goal_key = key_of(goal_node.x, goal_node.y);

  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater> open_heap;

  std::unordered_map<int64_t, double> g_score;
  std::unordered_map<int64_t, int64_t> came_from;
  std::unordered_set<int64_t> closed_set;

  g_score[start_key] = 0.0;

  uint64_t heap_counter = 0;
  const double start_h = octileXY(start_node.x, start_node.y, goal_node.x, goal_node.y);
  open_heap.push({start_h, heap_counter, start_node.x, start_node.y});

  int expanded_count = 0;
  bool goal_reached = false;

  while (!open_heap.empty() && rclcpp::ok()) {
    const OpenEntry current = open_heap.top();
    open_heap.pop();

    const int64_t current_key = key_of(current.x, current.y);

    if (closed_set.count(current_key)) {
      continue;
    }
    closed_set.insert(current_key);
    ++expanded_count;

    visited_map_.data[current.y * width + current.x] = 50;
    if (expanded_count % visited_publish_interval_ == 0) {
      publishVisitedMap();
    }

    if (current_key == goal_key) {
      goal_reached = true;
      break;
    }

    const double current_g = g_score[current_key];

    for (const auto & direction : directions) {
      const int dx = std::get<0>(direction);
      const int dy = std::get<1>(direction);
      const double movement_cost = std::get<2>(direction);

      const GraphNode neighbor{current.x + dx, current.y + dy};

      if (!poseOnMap(neighbor) || !isSafe(neighbor)) {
        continue;
      }

      // Diagonal corner cutting prevention.
      if (dx != 0 && dy != 0) {
        const GraphNode horizontal{current.x + dx, current.y};
        const GraphNode vertical{current.x, current.y + dy};
        if (!isSafe(horizontal) || !isSafe(vertical)) {
          continue;
        }
      }

      const int64_t neighbor_key = key_of(neighbor.x, neighbor.y);
      const double tentative_g = current_g + movement_cost;

      const auto existing_it = g_score.find(neighbor_key);
      const double existing_g =
        existing_it != g_score.end() ? existing_it->second : std::numeric_limits<double>::infinity();

      if (tentative_g >= existing_g) {
        continue;
      }

      came_from[neighbor_key] = current_key;
      g_score[neighbor_key] = tentative_g;

      const double heuristic = octileXY(neighbor.x, neighbor.y, goal_node.x, goal_node.y);
      const double f_score = tentative_g + heuristic;

      ++heap_counter;
      open_heap.push({f_score, heap_counter, neighbor.x, neighbor.y});
    }
  }

  publishVisitedMap();
  RCLCPP_INFO(logger_, "A* expanded %d cells", expanded_count);

  if (!goal_reached) {
    return path;
  }

  // Reconstruct path.
  std::vector<int64_t> cells;
  int64_t current_key = goal_key;
  cells.push_back(current_key);

  while (current_key != start_key) {
    const auto it = came_from.find(current_key);
    if (it == came_from.end()) {
      RCLCPP_ERROR(logger_, "Broken A* parent chain");
      return nav_msgs::msg::Path();
    }
    current_key = it->second;
    cells.push_back(current_key);
  }

  std::reverse(cells.begin(), cells.end());

  for (const int64_t key : cells) {
    const int x = static_cast<int>(key % width);
    const int y = static_cast<int>(key / width);

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = path.header;
    pose_stamped.pose = gridToWorldXY(x, y);
    path.poses.push_back(pose_stamped);
  }

  return path;
}

// ================================================================
// FAST safety lookup
// ================================================================

bool AStarPlanner::isSafe(const GraphNode & node) const
{
  if (!poseOnMap(node)) {
    return false;
  }
  const size_t index =
    static_cast<size_t>(node.y) * costmap_->getSizeInCellsX() + static_cast<size_t>(node.x);
  return safe_grid_[index];
}

// ================================================================
// Visited map
// ================================================================

void AStarPlanner::publishVisitedMap()
{
  if (!visited_map_pub_ || !visited_map_pub_->is_activated()) {
    return;
  }

  visited_map_.header.stamp = clock_->now();
  visited_map_pub_->publish(visited_map_);
}

// ================================================================
// Heuristic
// ================================================================

double AStarPlanner::octileXY(int x1, int y1, int x2, int y2)
{
  const int dx = std::abs(x1 - x2);
  const int dy = std::abs(y1 - y2);
  const int minimum = std::min(dx, dy);
  const int maximum = std::max(dx, dy);
  return maximum + (std::sqrt(2.0) - 1.0) * minimum;
}

// ================================================================
// Map helpers
// ================================================================

bool AStarPlanner::poseOnMap(const GraphNode & node) const
{
  return node.x >= 0 && node.x < static_cast<int>(costmap_->getSizeInCellsX()) &&
         node.y >= 0 && node.y < static_cast<int>(costmap_->getSizeInCellsY());
}

GraphNode AStarPlanner::worldToGrid(const geometry_msgs::msg::Pose & pose) const
{
  GraphNode node;
  node.x = static_cast<int>(std::floor(
      (pose.position.x - costmap_->getOriginX()) / costmap_->getResolution()));
  node.y = static_cast<int>(std::floor(
      (pose.position.y - costmap_->getOriginY()) / costmap_->getResolution()));
  return node;
}

geometry_msgs::msg::Pose AStarPlanner::gridToWorldXY(int x, int y) const
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = (x + 0.5) * costmap_->getResolution() + costmap_->getOriginX();
  pose.position.y = (y + 0.5) * costmap_->getResolution() + costmap_->getOriginY();
  pose.position.z = 0.0;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = 1.0;
  return pose;
}

}  // namespace diy_astar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(diy_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)
