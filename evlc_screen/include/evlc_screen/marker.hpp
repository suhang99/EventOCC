#pragma once

#include "evlc_common/event.hpp"
#include "evlc_common/types.hpp"

namespace evlc::screen {

class ScreenMarker {
 public:
  using Ptr = std::shared_ptr<ScreenMarker>;

  ScreenMarker() = default;
  ScreenMarker(double marker_size, int payload_cells);

  /**
   * @brief compute homography matrix from tracker points
   * @param trackers tracker points
   * @return homography matrix
   */
  cv::Mat getHomography(const std::array<cv::Point2d, 8>& trackers);

  /**
   * @brief compute homography matrix from locator points
   * @param locators locator points
   * @return homography matrix
   */
  cv::Mat getHomography(const std::array<cv::Point2d, 4>& locators);

  /**
   * @brief Compute locators' position from trackers
   * @param trackers tracker points
   * @return locator points
   */
  std::array<cv::Point2d, 4> trackersToLocators(const std::array<cv::Point2d, 8>& trackers);

  /**
   * @brief Compute trackers' position from locators
   * @param locators locator points
   * @return tracker points
   */
  std::array<cv::Point2d, 8> locatorsToTrackers(const std::array<cv::Point2d, 4>& locators);

  std::array<cv::Point3d, 8> tracker_world_points;
  std::array<cv::Point2d, 8> tracker_canvas_points;
  std::array<cv::Point3d, 4> locator_world_points;
  std::array<cv::Point2d, 4> locator_canvas_points;
  size_t payload_cells, marker_cells;
  double marker_size, payload_size, cell_size;
  size_t inner_cells;
  size_t canvas_cell_size, locator_canvas_size, tracker_canvas_size;
  const size_t inner_locator_cells = 1, outer_locator_cells = 2;

 private:
};

}  // namespace evlc::screen