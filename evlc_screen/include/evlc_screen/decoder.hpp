#pragma once

#include <ros/ros.h>

#include "evlc_common/event.hpp"
#include "evlc_common/types.hpp"
#include "evlc_screen/marker.hpp"
#include "evlc_screen/visualizer.hpp"

namespace evlc::screen {

class Decoder {
 public:
  using Ptr = std::shared_ptr<Decoder>;

  Decoder(ros::NodeHandle nh, ros::NodeHandle nh_private, SAE::Ptr sae, ScreenMarker::Ptr marker,
          Visualizer::Ptr visualizer);

  /**
   * @brief Decode data from events
   * @param trackers Tracker points of marker
   * @return Decoded data (empty if not ready)
   */
  std::string decode(const std::array<cv::Point2d, 8>& trackers);

  /**
   * @brief Decode data from events
   * @param locators Locator points of marker
   * @return Decoded data (empty if not ready)
   */
  std::string decode(const std::array<cv::Point2d, 4>& locators);

  /**
   * @brief Extract bits on markers by locator points
   * @param locators Locator points
   * @param period period for sae generation
   * @return Extracted BitMatrix
   */
  BitMatrix extractMarkerBits(const std::array<cv::Point2d, 4>& locators, double period);

  /**
   * @brief Find the number of counter-clockwise roll for marker towards standard orientation
   * @param bits input bits
   * @return number of counter-clockwise rolls
   */
  int getMarkerRoll(const BitMatrix& bits);

  /**
   * @brief Rotate the marker bits
   * @param roll the number of roll
   * @return rotated BitMatrix
   */
  BitMatrix alignMarkerBits(const BitMatrix& bits, int roll);

  const double& getCellThreshold() { return cell_thres_; };
  const double& getBinaryThreshold() { return binary_thres_; };

 private:
  SAE::Ptr sae_;
  ScreenMarker::Ptr marker_;
  Visualizer::Ptr visualizer_;
  double frequency_, period_;
  std::vector<BitMatrix> bits_;
  double cell_thres_;
  double binary_thres_;
  int decode_count_;
  int decode_times_;
};

}  // namespace evlc::screen