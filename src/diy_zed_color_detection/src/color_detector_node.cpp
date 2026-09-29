// ROS 2 node: subscribes to a colour image, detects RED/GREEN objects on a blue
// background (the blob must be surrounded by blue). Publishes one std_msgs/Bool topic per colour (~/red, ~/green): true when that
// colour is seen in the frame. Also publishes an annotated debug image.
// With use_depth, only pixels between min_distance_m and max_distance_m (ZED depth) count.
// All thresholds are ROS parameters (see config/params.yaml) and can be changed live
// with `ros2 param set`.
#include "diy_zed_color_detection/color_detector.hpp"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>  // Iron and newer
#else
#include <cv_bridge/cv_bridge.h>  // Humble
#endif
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#if __has_include(<message_filters/subscriber.hpp>)
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#else
#include <message_filters/subscriber.h>  // Humble
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#endif

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <functional>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace zcd {

class ColorDetectorNode : public rclcpp::Node {
 public:
  explicit ColorDetectorNode(const rclcpp::NodeOptions& options)
      : Node("color_detector", options), detector_(DetectorConfig{}) {
    declareParameters();
    detector_.setConfig(configFromParameters());
    std::tie(min_distance_m_, max_distance_m_) = distanceRangeFromParameters();

    // Rebuild the detector whenever a parameter changes (e.g. `ros2 param set`); invalid values are rejected.
    on_set_handle_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      try {
        DetectorConfig cfg = configFromParameters(params);
        const auto range = distanceRangeFromParameters(params);
        std::lock_guard<std::mutex> lock(mutex_);
        detector_.setConfig(std::move(cfg));
        std::tie(min_distance_m_, max_distance_m_) = range;
      } catch (const std::exception& e) {
        result.successful = false;
        result.reason = e.what();
      }
      return result;
    });

    publish_debug_ = get_parameter("publish_debug_image").as_bool();
    publish_masks_ = get_parameter("publish_masks").as_bool();

    // One Bool topic per colour, named after it in lower case: RED -> ~/red, GREEN -> ~/green.
    for (const auto& c : detector_.config().colors) {
      std::string topic = c.name;
      std::transform(topic.begin(), topic.end(), topic.begin(), [](unsigned char ch) { return std::tolower(ch); });
      color_pubs_.push_back(create_publisher<std_msgs::msg::Bool>("~/" + topic, 10));
    }
    if (publish_debug_) debug_pub_ = image_transport::create_publisher(this, "~/debug_image");
    if (publish_masks_) {
      for (const auto& c : detector_.config().colors) {
        mask_pubs_.push_back(image_transport::create_publisher(this, "~/mask/" + c.name));
      }
    }

    // Subscribe to the topic named by `image_topic`. The ZED wrapper publishes RELIABLE; a
    // best-effort subscriber (the default) accepts both reliable and best-effort publishers.
    image_topic_ = get_parameter("image_topic").as_string();
    const std::string transport = get_parameter("image_transport").as_string();
    const bool reliable = get_parameter("qos_reliable").as_bool();
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(get_parameter("qos_queue_size").as_int())));
    if (reliable) qos.reliable(); else qos.best_effort();

    use_depth_ = get_parameter("use_depth").as_bool();
    if (use_depth_) {
      // Colour + depth, paired by timestamp. Depth is the ZED registered depth (same view as the
      // left colour image): 32FC1 in metres, or 16UC1 in millimetres.
      depth_topic_ = get_parameter("depth_topic").as_string();
      if (transport != "raw") {
        RCLCPP_WARN(get_logger(), "image_transport '%s' is ignored when use_depth is true; using raw", transport.c_str());
      }
      const rmw_qos_profile_t profile = qos.get_rmw_qos_profile();
      color_filter_.subscribe(this, image_topic_, profile);
      depth_filter_.subscribe(this, depth_topic_, profile);
      sync_ = std::make_shared<Sync>(SyncPolicy(static_cast<uint32_t>(get_parameter("qos_queue_size").as_int())),
                                     color_filter_, depth_filter_);
      sync_->registerCallback(std::bind(&ColorDetectorNode::onImage, this, std::placeholders::_1, std::placeholders::_2));
      image_topic_ = color_filter_.getSubscriber()->get_topic_name();
      depth_topic_ = depth_filter_.getSubscriber()->get_topic_name();
      RCLCPP_INFO(get_logger(), "subscribed to '%s' + depth '%s' (qos=%s), range %.2f-%.2f m, detecting %zu colour(s)",
                  image_topic_.c_str(), depth_topic_.c_str(), reliable ? "reliable" : "best_effort",
                  min_distance_m_, max_distance_m_, detector_.config().colors.size());
    } else {
      auto cb = [this](const sensor_msgs::msg::Image::ConstSharedPtr& msg) { onImage(msg, nullptr); };
      if (transport == "raw") {
        raw_sub_ = create_subscription<sensor_msgs::msg::Image>(image_topic_, qos, cb);
        image_topic_ = raw_sub_->get_topic_name();
      } else {
        image_sub_ = image_transport::create_subscription(this, image_topic_, cb, transport, qos.get_rmw_qos_profile());
        image_topic_ = image_sub_.getTopic();
      }
      RCLCPP_INFO(get_logger(), "subscribed to '%s' (transport=%s, qos=%s), no depth limit, detecting %zu colour(s)",
                  image_topic_.c_str(), transport.c_str(), reliable ? "reliable" : "best_effort",
                  detector_.config().colors.size());
    }

    // If nothing arrives, say so and list the image topics that do exist.
    watchdog_ = create_wall_timer(std::chrono::seconds(5), [this]() { checkInput(); });
  }

  void checkInput() {
    if (frames_ > 0) {
      if (!reported_first_frame_) {
        RCLCPP_INFO(get_logger(), "receiving images on '%s'", image_topic_.c_str());
        reported_first_frame_ = true;
      }
      watchdog_->cancel();
      return;
    }
    if (use_depth_ && count_publishers(depth_topic_) == 0) {
      RCLCPP_WARN(get_logger(), "nobody publishes depth topic '%s'. Set depth_topic, check the ZED depth is "
                  "enabled, or set use_depth:=false", depth_topic_.c_str());
    } else if (use_depth_ && count_publishers(image_topic_) > 0) {
      RCLCPP_WARN(get_logger(), "colour and depth are both published but no synchronised pair arrived yet. "
                  "Check both topics are publishing (ros2 topic hz) or try qos_reliable true/false");
      return;
    }
    const size_t publishers = count_publishers(image_topic_);
    std::string candidates;
    for (const auto& [name, types] : get_topic_names_and_types()) {
      for (const auto& t : types) {
        if (t == "sensor_msgs/msg/Image") candidates += "\n    " + name;
      }
    }
    if (publishers == 0) {
      RCLCPP_WARN(get_logger(),
                  "no image received on '%s' and nobody publishes it. Set image_topic to one of:%s",
                  image_topic_.c_str(), candidates.empty() ? " (no Image topics found; is the camera running?)" : candidates.c_str());
    } else {
      RCLCPP_WARN(get_logger(),
                  "'%s' has %zu publisher(s) but no image arrived yet. Likely a QoS mismatch (try qos_reliable "
                  "true/false) or an image_transport mismatch (try image_transport:=compressed)",
                  image_topic_.c_str(), publishers);
    }
  }

 private:
  void declareParameters() {
    declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image");
    declare_parameter("image_transport", "raw");
    declare_parameter("qos_reliable", false);
    declare_parameter("qos_queue_size", 10);
    declare_parameter("use_depth", true);
    declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered");
    declare_parameter("min_distance_m", 0.0);
    declare_parameter("max_distance_m", 2.0);
    declare_parameter("publish_debug_image", true);
    declare_parameter("publish_masks", false);
    declare_parameter("blur_kernel", 5);
    declare_parameter("morph_kernel", 5);
    declare_parameter("min_area", 400.0);
    declare_parameter("max_area", 0.0);

    declare_parameter("background.enabled", true);
    declare_parameter("background.hue_range", std::vector<int64_t>{95, 130});
    declare_parameter("background.sat_range", std::vector<int64_t>{60, 255});
    declare_parameter("background.val_range", std::vector<int64_t>{40, 255});
    declare_parameter("background.require_surround", true);
    declare_parameter("background.surround_gap_px", 3);
    declare_parameter("background.surround_width_px", 15);
    declare_parameter("background.surround_min_ratio", 0.5);
    declare_parameter("background.min_frame_ratio", 0.0);

    const auto names = declare_parameter("colors", std::vector<std::string>{"RED", "GREEN"});
    for (const auto& n : names) {
      const bool red = n == "RED";
      declare_parameter(n + ".hue_ranges", red ? std::vector<int64_t>{0, 10, 170, 179} : std::vector<int64_t>{40, 85});
      declare_parameter(n + ".sat_range", std::vector<int64_t>{red ? 100 : 80, 255});
      declare_parameter(n + ".val_range", std::vector<int64_t>{red ? 70 : 50, 255});
      declare_parameter(n + ".draw_bgr", red ? std::vector<int64_t>{0, 0, 255} : std::vector<int64_t>{0, 255, 0});
    }
    color_names_ = names;
  }

  // Builds a config from current parameters, with `overrides` (pending changes) taking precedence.
  DetectorConfig configFromParameters(const std::vector<rclcpp::Parameter>& overrides = {}) {
    auto get = [&](const std::string& name) {
      for (const auto& p : overrides) {
        if (p.get_name() == name) return p;
      }
      return get_parameter(name);
    };
    auto pair = [&](const std::string& name, int max_value) {
      const auto v = get(name).as_integer_array();
      if (v.size() != 2) throw std::runtime_error(name + " must be [min, max]");
      if (v[0] < 0 || v[1] > max_value || v[0] > v[1]) {
        throw std::runtime_error(name + " must satisfy 0 <= min <= max <= " + std::to_string(max_value));
      }
      return std::pair<int, int>(static_cast<int>(v[0]), static_cast<int>(v[1]));
    };

    for (const auto& p : overrides) {
      if (p.get_name() == "colors") throw std::runtime_error("'colors' can only be set at startup");
    }

    DetectorConfig cfg;
    cfg.blur_kernel = static_cast<int>(get("blur_kernel").as_int());
    cfg.morph_kernel = static_cast<int>(get("morph_kernel").as_int());
    cfg.min_area = get("min_area").as_double();
    cfg.max_area = get("max_area").as_double();

    cfg.background.enabled = get("background.enabled").as_bool();
    std::tie(cfg.background.hue.hue_min, cfg.background.hue.hue_max) = pair("background.hue_range", 179);
    std::tie(cfg.background.sat_min, cfg.background.sat_max) = pair("background.sat_range", 255);
    std::tie(cfg.background.val_min, cfg.background.val_max) = pair("background.val_range", 255);
    cfg.background.require_surround = get("background.require_surround").as_bool();
    cfg.background.surround_gap_px = static_cast<int>(get("background.surround_gap_px").as_int());
    cfg.background.surround_width_px = static_cast<int>(get("background.surround_width_px").as_int());
    cfg.background.surround_min_ratio = get("background.surround_min_ratio").as_double();
    cfg.background.min_frame_ratio = get("background.min_frame_ratio").as_double();
    if (cfg.background.surround_gap_px < 0 || cfg.background.surround_width_px < 1) {
      throw std::runtime_error("background.surround_gap_px must be >= 0 and surround_width_px >= 1");
    }
    for (double r : {cfg.background.surround_min_ratio, cfg.background.min_frame_ratio}) {
      if (r < 0.0 || r > 1.0) throw std::runtime_error("background ratios must be between 0.0 and 1.0");
    }

    for (const auto& n : color_names_) {
      ColorSpec spec;
      spec.name = n;
      const auto hues = get(n + ".hue_ranges").as_integer_array();
      if (hues.empty() || hues.size() % 2 != 0) throw std::runtime_error(n + ".hue_ranges must be pairs");
      for (size_t i = 0; i < hues.size(); i += 2) {
        if (hues[i] < 0 || hues[i + 1] > 179 || hues[i] > hues[i + 1]) {
          throw std::runtime_error(n + ".hue_ranges values must satisfy 0 <= min <= max <= 179");
        }
        spec.hue_ranges.push_back({static_cast<int>(hues[i]), static_cast<int>(hues[i + 1])});
      }
      std::tie(spec.sat_min, spec.sat_max) = pair(n + ".sat_range", 255);
      std::tie(spec.val_min, spec.val_max) = pair(n + ".val_range", 255);
      const auto bgr = get(n + ".draw_bgr").as_integer_array();
      if (bgr.size() == 3) spec.draw_bgr = cv::Scalar(bgr[0], bgr[1], bgr[2]);
      cfg.colors.push_back(std::move(spec));
    }
    return cfg;
  }

  std::pair<double, double> distanceRangeFromParameters(const std::vector<rclcpp::Parameter>& overrides = {}) {
    auto get = [&](const std::string& name) {
      for (const auto& p : overrides) {
        if (p.get_name() == name) return p.as_double();
      }
      return get_parameter(name).as_double();
    };
    const double lo = get("min_distance_m"), hi = get("max_distance_m");
    if (lo < 0.0 || hi <= lo) throw std::runtime_error("need 0 <= min_distance_m < max_distance_m");
    return {lo, hi};
  }

  // Non-zero where the depth is finite and inside [min, max] metres; resized to the colour image.
  static cv::Mat depthMask(const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg, const cv::Size& size,
                           double min_m, double max_m) {
    cv_bridge::CvImageConstPtr d = cv_bridge::toCvShare(depth_msg);
    cv::Mat metres;
    if (d->image.type() == CV_16UC1) {
      d->image.convertTo(metres, CV_32F, 0.001);  // millimetres -> metres; 0 means no data
    } else if (d->image.type() == CV_32FC1) {
      metres = d->image;
    } else {
      throw std::runtime_error("unsupported depth encoding '" + depth_msg->encoding + "' (need 32FC1 or 16UC1)");
    }
    // NaN and +/-inf fail both comparisons, so they are excluded; so is 0 when min_m > 0.
    cv::Mat mask = (metres >= std::max(min_m, 1e-6)) & (metres <= max_m);
    if (mask.size() != size) cv::resize(mask, mask, size, 0, 0, cv::INTER_NEAREST);
    return mask;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg,
               const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg) {
    ++frames_;
    cv_bridge::CvImagePtr cv;
    try {
      // Converts bgra8 (ZED), rgb8, mono etc. to bgr8.
      cv = cv_bridge::toCvCopy(msg, "bgr8");
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }

    DetectionResult result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cv::Mat in_range;
      if (depth_msg) {
        try {
          in_range = depthMask(depth_msg, cv->image.size(), min_distance_m_, max_distance_m_);
        } catch (const std::exception& e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "depth: %s", e.what());
          return;
        }
      }
      result = detector_.detect(cv->image, in_range);
      if (publish_debug_ && debug_pub_.getNumSubscribers() > 0) {
        if (!in_range.empty()) {
          // Darken everything outside the distance range so the limit is visible.
          cv::Mat dark = cv->image * 0.3;
          dark.copyTo(cv->image, ~in_range);
        }
        detector_.draw(cv->image, result);
        debug_pub_.publish(cv->toImageMsg());
      }
    }

    for (size_t i = 0; i < color_pubs_.size() && i < result.masks.size(); ++i) {
      const std::string& name = color_names_[i];
      std_msgs::msg::Bool seen;
      seen.data = std::any_of(result.detections.begin(), result.detections.end(),
                              [&](const Detection& d) { return d.color == name; });
      color_pubs_[i]->publish(seen);
    }

    for (size_t i = 0; i < mask_pubs_.size() && i < result.masks.size(); ++i) {
      if (mask_pubs_[i].getNumSubscribers() == 0) continue;
      mask_pubs_[i].publish(cv_bridge::CvImage(msg->header, "mono8", result.masks[i]).toImageMsg());
    }
  }

  std::mutex mutex_;
  ColorDetector detector_;
  std::vector<std::string> color_names_;
  bool publish_debug_ = true;
  bool publish_masks_ = false;

  std::string image_topic_;
  std::atomic<size_t> frames_{0};
  bool reported_first_frame_ = false;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;
  bool use_depth_ = false;
  std::string depth_topic_;
  double min_distance_m_ = 0.0, max_distance_m_ = 2.0;
  message_filters::Subscriber<sensor_msgs::msg::Image> color_filter_, depth_filter_;
  std::shared_ptr<Sync> sync_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher debug_pub_;
  std::vector<image_transport::Publisher> mask_pubs_;
  std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> color_pubs_;
  OnSetParametersCallbackHandle::SharedPtr on_set_handle_;
};

}  // namespace zcd

RCLCPP_COMPONENTS_REGISTER_NODE(zcd::ColorDetectorNode)
