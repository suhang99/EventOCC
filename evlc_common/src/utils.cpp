#include "evlc_common/utils.hpp"

namespace evlc {

void drawQuad(cv::Mat& image, const std::array<cv::Point2d, 4>& points) {
  std::vector<cv::Point> quad_points(5);
  quad_points[0] = points[0];
  quad_points[1] = points[1];
  quad_points[2] = points[2];
  quad_points[3] = points[3];
  quad_points[4] = points[0];
  const cv::Point* cv_points[1] = {quad_points.data()};
  int num_points[] = {static_cast<int>(quad_points.size())};
  cv::polylines(image, cv_points, num_points, 1, true, cv::Scalar(0, 255, 0), 3);
}

double computeIoU(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2) {
  std::vector<cv::Point> intersection;

  // Compute the intersection of two convex polygons
  cv::intersectConvexConvex(contour1, contour2, intersection, false);

  // Return 0 if empty intersection
  if (intersection.empty()) {
    return 0;
  }
  // Compute areas
  double area_intersection = cv::contourArea(intersection);
  double area_contour1 = cv::contourArea(contour1);
  double area_contour2 = cv::contourArea(contour2);

  // Compute the union area
  double area_union = area_contour1 + area_contour2 - area_intersection;

  // Compute IoU
  return area_intersection / area_union;
}

}  // namespace evlc