#pragma once

#include "evlc_common/types.hpp"

namespace evlc {

void drawQuad(cv::Mat& image, const std::array<cv::Point2d, 4>& points);

double computeIoU(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2);

inline Eigen::Vector3d transformPoint(const Eigen::Isometry3d& T, const Eigen::Vector3d& P) {
  return (T.matrix() * P.homogeneous()).hnormalized();
}

inline Eigen::Vector3d transformPoint(const Eigen::Isometry3d& T, const cv::Point3d& P) {
  return transformPoint(T, Eigen::Vector3d(P.x, P.y, P.z));
}



}  // namespace evlc