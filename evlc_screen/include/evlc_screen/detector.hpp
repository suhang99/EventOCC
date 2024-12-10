#pragma once

#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <array>
#include <opencv2/opencv.hpp>
#include <optional>

#include "evlc_common/buffer.hpp"
#include "evlc_common/event.hpp"
#include "evlc_common/timer.hpp"
#include "evlc_common/types.hpp"
#include "evlc_screen/marker.hpp"
#include "evlc_screen/visualizer.hpp"

namespace evlc::screen {

/**
 * Detector for detecting marker spatially and temporally
 */
class Detector {
 public:
  using Ptr = std::shared_ptr<Detector>;

  /**
   * @brief status of detector
   */
  enum class Status { Initial, Temporal, Spatial, Success };

  /**
   *  Intermediate results for detection
   */
  class Result {
   public:
    double phase;   // the result of temporal detection
    ROI roi;        // top left corner and bottom right corner
    Status status;  // Status of current detector
  };

  Detector(ros::NodeHandle nh, ros::NodeHandle nh_private, SAE::Ptr sae,
           TemporalBuffer<Event>::Ptr event_buffer, ScreenMarker::Ptr marker,
           Visualizer::Ptr visualizer, Profiler::Ptr profiler);
  ~Detector() = default;

  /**
   * @brief Update one event for detection
   */
  void update(const Event& e);

  /**
   * @brief Detect marker
   * @param t current detect time
   * @return the result of detection
   */
  const Result& detect(Time t);

  /**
   * @brief Reset intermedieate results for next detection
   */
  void reset();

 private:
  /**
   * @brief Detect the spatial location of marker
   * @return region of interests
   */
  ROI detectSpatial();

  /**
   * @brief Detect the temporal location of marker
   * @param t Current temporal detection time
   * @return Estimated phase
   */
  double detectTemporal(const Time t);

  /**
   * @brief Check whether a set of points satisfy the requirements of locator
   * @param cv_points A set of points that may belong to a locator
   * @return 4 points that form a valid quad
   */
  ROI findValidROI(const std::vector<cv::Point>& cv_points);

  /**
   * @brief Use more accurate (slow) method to refine spatial detecction
   * @param cv_roi ROI to refine
   * @return refined ROI
   */
  ROI refineSpatial(ROI roi);

  ROI alignCorners(const ROI& corners);

  // parameters for detection
  double amplitude_thres_;
  double sigma_;                           // sigma of Gaussian weight to detect ROI
  const double length_thres_ = 30;         // Edge length threshold (in pixel)
  const double cos_thres_ = 0.258;         // Angle threshold for L-shape
  const double length_ratio_thres_ = 0.1;  // Ratio of adjacent edge length
  double frequency_, period_;
  double temporal_detector_period_;

  // event containers
  SAE::Ptr sae_;
  SAE::Ptr sae_delta_;  // pos stands for neg->pos, neg stands for pos->neg
  TemporalBuffer<Event>::Ptr event_buffer_;
  TemporalBuffer<BlinkEvent>::Ptr blink_events_;

  ScreenMarker::Ptr marker_;
  Result result_;
  Visualizer::Ptr visualizer_;
  Profiler::Ptr profiler_;
};
}  // namespace evlc::screen