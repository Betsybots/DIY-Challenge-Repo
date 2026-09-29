#pragma once

#include <opencv2/core.hpp>

#include <string>
#include <vector>

namespace zcd {

struct HsvRange {
  int hue_min = 0, hue_max = 179;
};

struct ColorSpec {
  std::string name;
  std::vector<HsvRange> hue_ranges;
  int sat_min = 0, sat_max = 255;
  int val_min = 0, val_max = 255;
  cv::Scalar draw_bgr{255, 255, 255};
};

struct BackgroundSpec {
  bool enabled = true;
  HsvRange hue{95, 130};
  int sat_min = 60, sat_max = 255;
  int val_min = 40, val_max = 255;

  // Object must sit on blue: of the pixels in a ring `surround_width_px` wide around the blob
  // (starting `surround_gap_px` outside it), at least `surround_min_ratio` must be blue.
  bool require_surround = true;
  int surround_gap_px = 3;
  int surround_width_px = 15;
  double surround_min_ratio = 0.5;
  // Reject everything unless at least this share of the whole frame is blue (0 disables).
  double min_frame_ratio = 0.0;
};

struct DetectorConfig {
  int blur_kernel = 5;
  int morph_kernel = 5;
  double min_area = 400.0;
  double max_area = 0.0;  // 0 = unlimited
  BackgroundSpec background;
  std::vector<ColorSpec> colors;

  // Throws std::runtime_error on a missing file or invalid values.
  static DetectorConfig load(const std::string& path);
};

struct Detection {
  std::string color;
  cv::Rect box;
  cv::Point2f center;
  double area = 0.0;
  double surround_ratio = 1.0;  // share of blue pixels around the blob
};

struct DetectionResult {
  std::vector<Detection> detections;  // accepted: right colour, on blue, in range
  std::vector<Detection> rejected;    // right colour and size but not on a blue background
  std::vector<cv::Mat> masks;      // one binary mask per ColorSpec, same order
  cv::Mat background_mask;         // empty when background is disabled
  double background_fraction = 0;  // share of the frame classed as blue background
};

class ColorDetector {
 public:
  explicit ColorDetector(DetectorConfig cfg);

  void setConfig(DetectorConfig cfg) { cfg_ = std::move(cfg); }
  const DetectorConfig& config() const { return cfg_; }

  // `valid_mask` (optional, CV_8U, same size as bgr): only non-zero pixels can be detected,
  // e.g. pixels within the allowed depth range.
  DetectionResult detect(const cv::Mat& bgr, const cv::Mat& valid_mask = cv::Mat()) const;

  // Draws boxes and labels for every detection onto `bgr`.
  void draw(cv::Mat& bgr, const DetectionResult& result) const;

 private:
  DetectorConfig cfg_;
};

}  // namespace zcd
