// Fast A* global planner for the DIY Challenge robot.
//
// Nav2 GlobalPlanner plugin: loaded by nav2_planner's planner_server via
// pluginlib. Reads obstacle data from the Nav2 costmap and returns a path
// synchronously from createPlan().

#ifndef PLANNING__A_STAR_PLANNER_NODE_HPP_
#define PLANNING__A_STAR_PLANNER_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_astar_planner
{

struct GraphNode
{
  int x = 0;
  int y = 0;
};

// Open-set entry for the priority queue: (f_score, insertion counter, x, y).
// The counter is a deterministic tie breaker matching Python's heapq usage.
struct OpenEntry
{
  double f = 0.0;
  uint64_t counter = 0;
  int x = 0;
  int y = 0;
};

struct OpenEntryGreater
{
  bool operator()(const OpenEntry & a, const OpenEntry & b) const
  {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    return a.counter > b.counter;
  }
};

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  void buildSafeGrid();

  bool isSafe(const GraphNode & node) const;

  void publishVisitedMap();

  static double octileXY(int x1, int y1, int x2, int y2);

  bool poseOnMap(const GraphNode & node) const;
  GraphNode worldToGrid(const geometry_msgs::msg::Pose & pose) const;
  geometry_msgs::msg::Pose gridToWorldXY(int x, int y) const;

  // Nav2 plugin context
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_ = nullptr;
  std::string name_;
  std::string global_frame_;
  rclcpp::Logger logger_{rclcpp::get_logger("AStarPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  // Parameters (declared under the plugin name, e.g. "GridPlanner.robot_clearance")
  double robot_clearance_ = 0.40;
  int occupied_threshold_ = 253;
  bool unknown_is_occupied_ = true;
  int visited_publish_interval_ = 250;

  // Debug publisher
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr visited_map_pub_;

  // State
  std::vector<bool> safe_grid_;
  nav_msgs::msg::OccupancyGrid visited_map_;
  int clearance_cells_ = 0;
};

}  // namespace nav2_astar_planner

#endif  // PLANNING__A_STAR_PLANNER_NODE_HPP_
