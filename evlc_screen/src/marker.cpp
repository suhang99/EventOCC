#include "evlc_screen/marker.hpp"

namespace evlc::screen {

ScreenMarker::ScreenMarker(double marker_size, int payload_cells)
    : payload_cells(payload_cells), marker_size(marker_size) {
  // Set cell size of different regions
  cell_size = marker_size / (payload_cells + 2 * (inner_locator_cells + outer_locator_cells));
  inner_cells = payload_cells + 2 * inner_locator_cells;
  marker_cells = inner_cells + 2 * outer_locator_cells;
  canvas_cell_size = 10;

  // Create tracker world points. Starting at upper left in clockwise
  tracker_world_points[0] = cv::Point3d(0, 0, 0);
  tracker_world_points[1] = cv::Point3d(marker_size, 0, 0);
  tracker_world_points[2] = cv::Point3d(marker_size, marker_size, 0);
  tracker_world_points[3] = cv::Point3d(0, marker_size, 0);
  tracker_world_points[4] = cv::Point3d(cell_size, cell_size, 0);
  tracker_world_points[5] = cv::Point3d(marker_size - cell_size, cell_size, 0);
  tracker_world_points[6] = cv::Point3d(marker_size - cell_size, marker_size - cell_size, 0);
  tracker_world_points[7] = cv::Point3d(cell_size, marker_size - cell_size, 0);

  // Create locator world points. Set the origin at the upper left corner of tracker points
  double outer_size = outer_locator_cells * cell_size;
  locator_world_points[0] = cv::Point3d(outer_size, outer_size, 0);
  locator_world_points[1] = cv::Point3d(marker_size - outer_size, outer_size, 0);
  locator_world_points[2] = cv::Point3d(marker_size - outer_size, marker_size - outer_size, 0);
  locator_world_points[3] = cv::Point3d(outer_size, marker_size - outer_size, 0);

  // Create locator canvas points (for homography estimation)
  // Manually set canvas cell size to 10 for convenience
  locator_canvas_size = canvas_cell_size * inner_cells;
  locator_canvas_points[0] = cv::Point2d(0, 0);
  locator_canvas_points[1] = cv::Point2d(locator_canvas_size - 1, 0);
  locator_canvas_points[2] = cv::Point2d(locator_canvas_size - 1, locator_canvas_size - 1);
  locator_canvas_points[3] = cv::Point2d(0, locator_canvas_size - 1);

  // Create tracker canvas points
  tracker_canvas_size = canvas_cell_size * marker_cells;
  auto margin = canvas_cell_size;
  tracker_canvas_points[0] = cv::Point2d(0, 0);
  tracker_canvas_points[1] = cv::Point2d(tracker_canvas_size - 1, 0);
  tracker_canvas_points[2] = cv::Point2d(tracker_canvas_size - 1, tracker_canvas_size - 1);
  tracker_canvas_points[3] = cv::Point2d(0, tracker_canvas_size - 1);
  tracker_canvas_points[4] = cv::Point2d(margin, margin);
  tracker_canvas_points[5] = cv::Point2d(tracker_canvas_size - margin - 1, margin);
  tracker_canvas_points[6] =
      cv::Point2d(tracker_canvas_size - margin - 1, tracker_canvas_size - margin - 1);
  tracker_canvas_points[7] = cv::Point2d(margin, tracker_canvas_size - margin - 1);
}

cv::Mat ScreenMarker::getHomography(const std::array<cv::Point2d, 8>& trackers) {
  return cv::findHomography(trackers, tracker_canvas_points);
}

cv::Mat ScreenMarker::getHomography(const std::array<cv::Point2d, 4>& locators) {
  return cv::findHomography(locators, locator_canvas_points);
}

std::array<cv::Point2d, 4> ScreenMarker::trackersToLocators(
    const std::array<cv::Point2d, 8>& trackers) {
  std::array<cv::Point2d, 4> locators;
  cv::Point2d center(0, 0);
  for (auto& p : trackers) {
    center += p;
  }
  center /= 8;
  double scale = 1.0 * inner_cells / marker_cells;
  for (int i = 0; i < 4; ++i) {
    locators[i] = center + scale * (trackers[i] - center);
  }
  return locators;
}

std::array<cv::Point2d, 8> ScreenMarker::locatorsToTrackers(const std::array<cv::Point2d, 4>& locators){
  std::array<cv::Point2d, 8> trackers;
  cv::Point2d center(0,0);
  for(auto& p: locators){
    center += p;
  }
  center /= 4;
  double scale1 = 1.0 * marker_cells / inner_cells;
  double scale2 = 1.0 * (marker_cells - outer_locator_cells) / inner_cells;
  for(int i = 0; i < 4; ++i){
    trackers[i] = center + scale1 * (locators[i] - center);
    trackers[i+4] = center + scale2 * (locators[i] - center);
  }
  return trackers;
}

}  // namespace evlc