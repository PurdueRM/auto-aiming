#ifndef _OPENCVARMORDETECTOR_HPP
#define _OPENCVARMORDETECTOR_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <math.h>

// Detector Constants
#define LIGHT_BAR_ANGLE_LIMIT 10.0
#define LIGHT_BAR_ASPECT_RATIO_LOWER_LIMIT 1.0
#define LIGHT_BAR_WIDTH_LOWER_LIMIT 2.0
#define LIGHT_BAR_HEIGHT_LOWER_LIMIT 5.0
#define ARMOR_ANGLE_DIFF_LIMIT 5.0
#define ARMOR_LIGHT_BAR_ASPECT_RATIO_RATIO_LIMIT 5.0
#define ARMOR_Y_DIFF_LIMIT 0.5
#define ARMOR_HEIGHT_RATIO_LIMIT 1.5
#define ARMOR_ASPECT_RATIO_LIMIT 7

#define WIDTH 1280
#define HEIGHT 1024

enum TargetColor
{
  BLUE,
  RED
};

/**
 * @brief Configuration struct for the OpenCVArmorDetector class.
 *
 * @param _target_color The target color to detect (BLUE or RED).
 * @param _hue_range_limit The hue range limit for the target color.
 * @param _saturation_lower_limit The lower saturation limit for the target color.
 * @param _value_lower_limit The lower value limit for the target color.
 * @param _max_missed_frames The maximum number of frames to miss before resetting the search area.
 * @param _reduce_search_area Whether to reduce the search area to the bounding box of the detected armor.
 */
struct DetectorConfig
{
  TargetColor _target_color;
  int _hue_range_limit;
  int _saturation_lower_limit;
  int _value_lower_limit;
  int _max_missed_frames;
  bool _reduce_search_area = true;
};

class OpenCVArmorDetector
{
public:
  OpenCVArmorDetector(const DetectorConfig &config) : _config(config) { setConfig(config); }
  ~OpenCVArmorDetector() {}

  // Class methods
  std::vector<_Float32> search(cv::Mat &frame);

  // Other class variables
  int _detected_frame = 0;
  int _frame_count = 0;

  // Helper methods
  bool isLightBar(cv::RotatedRect &rect);
  bool isArmor(cv::RotatedRect &left_rect, cv::RotatedRect &right_rect);
  void drawRotatedRect(cv::Mat &frame, cv::RotatedRect &rect);

  // Setters
  void setConfig(DetectorConfig config);

  // Getters
  DetectorConfig getConfig() { return _config; }
  bool getResetSearchArea() { return _reset_search_area; }
  bool getMissedFrames() { return _missed_frames; }

private:
  // Class methods
  std::vector<cv::Point2f> detectArmorsInFrame(cv::Mat &frame);

  // Class variables
  DetectorConfig _config;
  int _search_area[4] = {0, 0, WIDTH, HEIGHT};
  bool _reset_search_area = true;
  int _missed_frames = 0;

  // Dynamic parameters
  TargetColor _targetColor;
  int _max_missed_frames;
  bool _reduce_search_area;

  // Helpers
  std::vector<cv::Point2f> rectToPoint(cv::RotatedRect &rect);

  // Color limits
  cv::Scalar _blue_lower_limit;
  cv::Scalar _blue_upper_limit;
  cv::Scalar _red_lower_limit_1;
  cv::Scalar _red_upper_limit_1;
  cv::Scalar _red_lower_limit_2;
  cv::Scalar _red_upper_limit_2;
};
#endif //_OPENCVARMORDETECTOR_HPP