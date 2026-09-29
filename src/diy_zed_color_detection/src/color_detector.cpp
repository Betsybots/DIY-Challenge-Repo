#include "diy_zed_color_detection/color_detector.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <climits>
#include <stdexcept>

namespace zcd {
namespace {

int readInt(const cv::FileNode& node, const std::string& key, int fallback) {
  const cv::FileNode n = node[key];
  return n.empty() ? fallback : static_cast<int>(n);
}

void checkRange(const std::string& what, int lo, int hi, int max_value) {
  if (lo < 0 || hi > max_value || lo > hi) {
    throw std::runtime_error(what + ": expected 0 <= min <= max <= " + std::to_string(max_value) +
                             ", got [" + std::to_string(lo) + ", " + std::to_string(hi) + "]");
  }
}

cv::Mat inRangeHsv(const cv::Mat& hsv, const HsvRange& h, int smin, int smax, int vmin, int vmax) {
  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(h.hue_min, smin, vmin), cv::Scalar(h.hue_max, smax, vmax), mask);
  return mask;
}

// Share of blue pixels in a ring around the blob. Only the blob's neighbourhood is processed.
double surroundRatio(const std::vector<cv::Point>& contour, const cv::Rect& box, const cv::Mat& blue,
                     const BackgroundSpec& b) {
  const int outer = b.surround_gap_px + b.surround_width_px;
  const cv::Rect roi = (box + cv::Size(2 * outer, 2 * outer) - cv::Point(outer, outer)) & cv::Rect(0, 0, blue.cols, blue.rows);
  cv::Mat blob = cv::Mat::zeros(roi.size(), CV_8U);
  cv::drawContours(blob, std::vector<std::vector<cv::Point>>{contour}, 0, 255, cv::FILLED, cv::LINE_8, cv::noArray(),
                   INT_MAX, -roi.tl());
  auto grow = [&](int r) {
    if (r <= 0) return blob.clone();
    cv::Mat out;
    cv::dilate(blob, out, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2 * r + 1, 2 * r + 1)));
    return out;
  };
  const cv::Mat ring = grow(outer) & ~grow(b.surround_gap_px);
  const int ring_px = cv::countNonZero(ring);
  if (ring_px == 0) return 0.0;
  return static_cast<double>(cv::countNonZero(ring & blue(roi))) / ring_px;
}

}  // namespace

DetectorConfig DetectorConfig::load(const std::string& path) {
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) throw std::runtime_error("cannot open config file: " + path);

  DetectorConfig cfg;
  const cv::FileNode root = fs.root();
  cfg.blur_kernel = readInt(root, "blur_kernel", cfg.blur_kernel);
  cfg.morph_kernel = readInt(root, "morph_kernel", cfg.morph_kernel);
  cfg.min_area = readInt(root, "min_area", static_cast<int>(cfg.min_area));
  cfg.max_area = readInt(root, "max_area", static_cast<int>(cfg.max_area));

  const cv::FileNode bg = fs["background"];
  if (!bg.empty()) {
    auto& b = cfg.background;
    b.enabled = readInt(bg, "enabled", 1) != 0;
    b.hue.hue_min = readInt(bg, "hue_min", b.hue.hue_min);
    b.hue.hue_max = readInt(bg, "hue_max", b.hue.hue_max);
    b.sat_min = readInt(bg, "sat_min", b.sat_min);
    b.sat_max = readInt(bg, "sat_max", b.sat_max);
    b.val_min = readInt(bg, "val_min", b.val_min);
    b.val_max = readInt(bg, "val_max", b.val_max);
    b.require_surround = readInt(bg, "require_surround", 1) != 0;
    b.surround_gap_px = readInt(bg, "surround_gap_px", b.surround_gap_px);
    b.surround_width_px = readInt(bg, "surround_width_px", b.surround_width_px);
    if (!bg["surround_min_ratio"].empty()) b.surround_min_ratio = static_cast<double>(bg["surround_min_ratio"]);
    if (!bg["min_frame_ratio"].empty()) b.min_frame_ratio = static_cast<double>(bg["min_frame_ratio"]);
    checkRange("background hue", b.hue.hue_min, b.hue.hue_max, 179);
    checkRange("background saturation", b.sat_min, b.sat_max, 255);
    checkRange("background value", b.val_min, b.val_max, 255);
  }

  const cv::FileNode colors = fs["colors"];
  if (colors.type() != cv::FileNode::SEQ || colors.size() == 0) {
    throw std::runtime_error("config needs a non-empty 'colors' list");
  }
  for (const cv::FileNode& c : colors) {
    ColorSpec spec;
    spec.name = static_cast<std::string>(c["name"]);
    if (spec.name.empty()) throw std::runtime_error("every colour needs a 'name'");

    std::vector<int> hues;
    c["hue_ranges"] >> hues;
    if (hues.empty() || hues.size() % 2 != 0) {
      throw std::runtime_error(spec.name + ": 'hue_ranges' must be pairs [min, max, ...]");
    }
    for (size_t i = 0; i < hues.size(); i += 2) {
      checkRange(spec.name + " hue", hues[i], hues[i + 1], 179);
      spec.hue_ranges.push_back({hues[i], hues[i + 1]});
    }
    spec.sat_min = readInt(c, "sat_min", spec.sat_min);
    spec.sat_max = readInt(c, "sat_max", spec.sat_max);
    spec.val_min = readInt(c, "val_min", spec.val_min);
    spec.val_max = readInt(c, "val_max", spec.val_max);
    checkRange(spec.name + " saturation", spec.sat_min, spec.sat_max, 255);
    checkRange(spec.name + " value", spec.val_min, spec.val_max, 255);

    std::vector<int> bgr;
    c["draw_bgr"] >> bgr;
    if (bgr.size() == 3) spec.draw_bgr = cv::Scalar(bgr[0], bgr[1], bgr[2]);
    cfg.colors.push_back(std::move(spec));
  }
  return cfg;
}

ColorDetector::ColorDetector(DetectorConfig cfg) : cfg_(std::move(cfg)) {}

DetectionResult ColorDetector::detect(const cv::Mat& bgr, const cv::Mat& valid_mask) const {
  DetectionResult out;
  if (bgr.empty()) return out;

  cv::Mat work = bgr;
  if (cfg_.blur_kernel > 1) {
    const int k = cfg_.blur_kernel | 1;  // force odd
    cv::GaussianBlur(bgr, work, cv::Size(k, k), 0);
  }
  cv::Mat hsv;
  cv::cvtColor(work, hsv, cv::COLOR_BGR2HSV);

  if (cfg_.background.enabled) {
    const auto& b = cfg_.background;
    out.background_mask = inRangeHsv(hsv, b.hue, b.sat_min, b.sat_max, b.val_min, b.val_max);
    out.background_fraction =
        static_cast<double>(cv::countNonZero(out.background_mask)) / out.background_mask.total();
  }

  const auto& bgs = cfg_.background;
  const bool check_surround = bgs.enabled && bgs.require_surround && bgs.surround_width_px > 0;
  const bool frame_is_blue = !bgs.enabled || out.background_fraction >= bgs.min_frame_ratio;

  cv::Mat kernel;
  if (cfg_.morph_kernel > 1) {
    kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(cfg_.morph_kernel, cfg_.morph_kernel));
  }

  for (const ColorSpec& spec : cfg_.colors) {
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8U);
    for (const HsvRange& h : spec.hue_ranges) {
      mask |= inRangeHsv(hsv, h, spec.sat_min, spec.sat_max, spec.val_min, spec.val_max);
    }
    if (!out.background_mask.empty()) mask &= ~out.background_mask;
    if (!valid_mask.empty()) mask &= valid_mask;
    if (!kernel.empty()) {
      cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
      cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
      const double area = cv::contourArea(contour);
      if (area < cfg_.min_area) continue;
      if (cfg_.max_area > 0 && area > cfg_.max_area) continue;
      const cv::Moments m = cv::moments(contour);
      Detection d;
      d.color = spec.name;
      d.box = cv::boundingRect(contour);
      d.area = area;
      d.center = m.m00 > 0 ? cv::Point2f(m.m10 / m.m00, m.m01 / m.m00)
                           : cv::Point2f(d.box.x + d.box.width / 2.f, d.box.y + d.box.height / 2.f);
      if (check_surround) d.surround_ratio = surroundRatio(contour, d.box, out.background_mask, bgs);
      const bool on_blue = frame_is_blue && (!check_surround || d.surround_ratio >= bgs.surround_min_ratio);
      (on_blue ? out.detections : out.rejected).push_back(std::move(d));
    }
    out.masks.push_back(std::move(mask));
  }

  std::sort(out.detections.begin(), out.detections.end(),
            [](const Detection& a, const Detection& b) { return a.area > b.area; });
  return out;
}

void ColorDetector::draw(cv::Mat& bgr, const DetectionResult& result) const {
  // Rejected blobs (not on blue): thin grey box with the blue share around them.
  for (const Detection& d : result.rejected) {
    cv::rectangle(bgr, d.box, {160, 160, 160}, 1);
    const std::string label = d.color + " not on blue " + std::to_string(static_cast<int>(d.surround_ratio * 100)) + "%";
    cv::putText(bgr, label, {d.box.x, std::max(15, d.box.y - 6)}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {160, 160, 160}, 1);
  }
  for (const Detection& d : result.detections) {
    cv::Scalar color(255, 255, 255);
    for (const ColorSpec& s : cfg_.colors) {
      if (s.name == d.color) color = s.draw_bgr;
    }
    cv::rectangle(bgr, d.box, color, 2);
    cv::circle(bgr, d.center, 4, color, cv::FILLED);
    const std::string label = d.color + " " + std::to_string(static_cast<int>(d.area));
    cv::putText(bgr, label, {d.box.x, std::max(15, d.box.y - 6)}, cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
  }
  if (cfg_.background.enabled) {
    const std::string bg = "blue bg: " + std::to_string(static_cast<int>(result.background_fraction * 100)) + "%";
    cv::putText(bgr, bg, {10, bgr.rows - 12}, cv::FONT_HERSHEY_SIMPLEX, 0.6, {255, 255, 255}, 2);
  }
}

}  // namespace zcd
