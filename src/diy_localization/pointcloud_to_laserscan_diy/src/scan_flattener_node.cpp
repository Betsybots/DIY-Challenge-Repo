// Flattens a spinning-lidar 3D PointCloud2 into a synthetic 2D LaserScan, for
// testing AMCL (which requires a 2D scan) against a raw sensor-frame cloud.
//
// A ring range + elevation-angle window is used to select the "scan lines"
// that make up the synthetic 2D slice, since a spinning mechanical lidar's
// rings are each captured at a fixed elevation angle regardless of range --
// a height-band filter (as in the stock pointcloud_to_laserscan package)
// would not reproduce that correctly. If the input cloud has no 'ring'
// field (some recordings/drivers omit it), ring filtering is skipped and
// only the vertical FOV window is applied.

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace scan_flattener
{

class ScanFlattenerNode : public rclcpp::Node
{
public:
  ScanFlattenerNode()
  : Node("scan_flattener_node")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/lidar_points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/scan");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "");

    ring_min_ = declare_parameter<int>("ring_min", 10);
    ring_max_ = declare_parameter<int>("ring_max", 41);
    vertical_fov_min_deg_ = declare_parameter<double>("vertical_fov_min_deg", -40.0);
    vertical_fov_max_deg_ = declare_parameter<double>("vertical_fov_max_deg", 20.0);

    angle_increment_ = declare_parameter<double>("angle_increment", 0.0035);
    range_min_ = declare_parameter<double>("range_min", 0.5);
    range_max_ = declare_parameter<double>("range_max", 30.0);
    use_inf_ = declare_parameter<bool>("use_inf", true);

    if (ring_min_ > ring_max_) {
      throw std::invalid_argument("ring_min must be <= ring_max");
    }
    if (vertical_fov_min_deg_ > vertical_fov_max_deg_) {
      throw std::invalid_argument("vertical_fov_min_deg must be <= vertical_fov_max_deg");
    }
    if (angle_increment_ <= 0.0 || range_min_ <= 0.0 || range_max_ <= range_min_) {
      throw std::invalid_argument("angle_increment/range_min/range_max are invalid");
    }

    vertical_fov_min_rad_ = vertical_fov_min_deg_ * M_PI / 180.0;
    vertical_fov_max_rad_ = vertical_fov_max_deg_ * M_PI / 180.0;
    num_bins_ = static_cast<size_t>(std::ceil((2.0 * M_PI) / angle_increment_));

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, 10);
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ScanFlattenerNode::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "scan_flattener_node: %s -> %s, ring [%d, %d], vertical FOV [%.1f, %.1f] deg, "
      "%zu bins, range [%.2f, %.2f] m",
      input_topic_.c_str(), output_topic_.c_str(), ring_min_, ring_max_,
      vertical_fov_min_deg_, vertical_fov_max_deg_, num_bins_, range_min_, range_max_);
  }

private:
  // Bins one already-filtered-by-ring point into `ranges` by elevation/range/angle.
  void binPoint(float x, float y, float z, std::vector<float> & ranges) const
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      return;
    }

    const double horizontal_dist = std::hypot(static_cast<double>(x), static_cast<double>(y));
    const double elevation = std::atan2(static_cast<double>(z), horizontal_dist);
    if (elevation < vertical_fov_min_rad_ || elevation > vertical_fov_max_rad_) {
      return;
    }

    const double range = std::hypot(horizontal_dist, static_cast<double>(z));
    if (range < range_min_ || range > range_max_) {
      return;
    }

    const double angle = std::atan2(static_cast<double>(y), static_cast<double>(x));
    // Normalize to [0, 2*pi) so it maps directly onto a [-pi, pi] scan's bin index.
    const double angle_from_min = angle - (-M_PI);
    size_t bin = static_cast<size_t>(std::floor(angle_from_min / angle_increment_));
    if (bin >= num_bins_) {
      bin = num_bins_ - 1;
    }

    if (static_cast<float>(range) < ranges[bin]) {
      ranges[bin] = static_cast<float>(range);
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
  {
    const float empty_range = use_inf_
      ? std::numeric_limits<float>::infinity()
      : static_cast<float>(range_max_) + 1.0f;
    std::vector<float> ranges(num_bins_, empty_range);

    uint8_t ring_datatype = 0;
    for (const auto & field : msg->fields) {
      if (field.name == "ring") {
        ring_datatype = field.datatype;
        break;
      }
    }
    if (ring_datatype == 0) {
      RCLCPP_WARN_ONCE(get_logger(),
        "Input cloud on '%s' has no 'ring' field -- ring-based scan-line filtering is "
        "disabled; only the vertical FOV window is applied.", input_topic_.c_str());
    } else if (
      ring_datatype != sensor_msgs::msg::PointField::UINT16 &&
      ring_datatype != sensor_msgs::msg::PointField::UINT8)
    {
      RCLCPP_WARN_ONCE(get_logger(),
        "Input cloud's 'ring' field has an unsupported datatype (%u) -- ring-based "
        "scan-line filtering is disabled; only the vertical FOV window is applied.",
        ring_datatype);
      ring_datatype = 0;
    }

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    if (ring_datatype == sensor_msgs::msg::PointField::UINT16) {
      sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg, "ring");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_ring) {
        if (*iter_ring < static_cast<uint16_t>(ring_min_) ||
          *iter_ring > static_cast<uint16_t>(ring_max_))
        {
          continue;
        }
        binPoint(*iter_x, *iter_y, *iter_z, ranges);
      }
    } else if (ring_datatype == sensor_msgs::msg::PointField::UINT8) {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_ring(*msg, "ring");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_ring) {
        if (*iter_ring < static_cast<uint8_t>(ring_min_) ||
          *iter_ring > static_cast<uint8_t>(ring_max_))
        {
          continue;
        }
        binPoint(*iter_x, *iter_y, *iter_z, ranges);
      }
    } else {
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        binPoint(*iter_x, *iter_y, *iter_z, ranges);
      }
    }

    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = msg->header.stamp;
    scan.header.frame_id = output_frame_id_.empty() ? msg->header.frame_id : output_frame_id_;
    scan.angle_min = -M_PI;
    scan.angle_max = -M_PI + static_cast<double>(num_bins_ - 1) * angle_increment_;
    scan.angle_increment = angle_increment_;
    scan.time_increment = 0.0;
    scan.scan_time = 0.0;
    scan.range_min = static_cast<float>(range_min_);
    scan.range_max = static_cast<float>(range_max_);
    scan.ranges = std::move(ranges);

    scan_pub_->publish(scan);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;

  int ring_min_;
  int ring_max_;
  double vertical_fov_min_deg_;
  double vertical_fov_max_deg_;
  double vertical_fov_min_rad_;
  double vertical_fov_max_rad_;

  double angle_increment_;
  double range_min_;
  double range_max_;
  bool use_inf_;
  size_t num_bins_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
};

}  // namespace scan_flattener

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_flattener::ScanFlattenerNode>());
  rclcpp::shutdown();
  return 0;
}
