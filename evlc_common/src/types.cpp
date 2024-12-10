#include "evlc_common/types.hpp"

namespace evlc {

std::array<double, 8> Pose::convertToTUM() {
  std::array<double, 8> tum;
  Eigen::Quaterniond quat(T.rotation());
  quat.normalize();
  tum[0] = ts;
  tum[1] = T.translation()[0];
  tum[2] = T.translation()[1];
  tum[3] = T.translation()[2];
  tum[4] = quat.x();
  tum[5] = quat.y();
  tum[6] = quat.z();
  tum[7] = quat.w();
  return tum;
}

void ROI::setPoints(const std::array<cv::Point2d, 4>& points) {
  points_ = points;
  sort();
}

void ROI::setPoints(const std::vector<cv::Point2d>& points) {
  assert(points.size() == 4);
  for (int i = 0; i < 4; ++i) {
    points_[i] = points[i];
  }
  sort();
}

void ROI::setPoints(cv::Point2d p1, cv::Point2d p2, cv::Point2d p3, cv::Point2d p4) {
  points_[0] = p1;
  points_[1] = p2;
  points_[2] = p3;
  points_[3] = p4;
  sort();
}

void ROI::sort() {
  // Sort corners in x and y
  std::vector<double> points_x, points_y;
  for (int i = 0; i < 4; ++i) {
    points_x.emplace_back(points_[i].x);
    points_y.emplace_back(points_[i].y);
  }
  std::sort(points_x.begin(), points_x.end());
  std::sort(points_y.begin(), points_y.end());

  // Rotate corners in correct order
  cv::Point2d tl, tr, bl, br;
  bool tl_flag = false, tr_flag = false, bl_flag = false, br_flag = false;
  for (auto& point : points_) {
    if (point.x <= points_x[1] and point.y <= points_y[1] and not tl_flag) {
      tl.x = static_cast<float>(point.x);
      tl.y = static_cast<float>(point.y);
      tl_flag = true;
    } else if (point.x <= points_x[1] and point.y >= points_y[2] and not bl_flag) {
      bl.x = static_cast<float>(point.x);
      bl.y = static_cast<float>(point.y);
      bl_flag = true;
    } else if (point.x >= points_x[2] and point.y <= points_y[1] and not tr_flag) {
      tr.x = static_cast<float>(point.x);
      tr.y = static_cast<float>(point.y);
      tr_flag = true;
    } else if (point.x >= points_x[2] and point.y >= points_y[2] and not br_flag) {
      br.x = static_cast<float>(point.x);
      br.y = static_cast<float>(point.y);
      br_flag = true;
    }
  }
  points_ = {tl, tr, br, bl};
  valid_ = true;
}

}  // namespace evlc