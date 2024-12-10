#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <vector>

namespace evlc {

// C++ standards
using std::cout;
using std::endl;
const double EPSILON = 1e-5;

// Eigen
using BitMatrix = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// Types
using Time = double;
const bool EVENT_POS = true;
const bool EVENT_NEG = false;

class Pose {
 public:
  Pose(Time ts, Eigen::Isometry3d T) : ts(ts), T(T) {}

  std::array<double, 8> convertToTUM();

  Time ts;              // Timestamp
  Eigen::Isometry3d T;  // Transformation
};

class ROI {
 public:
  ROI() : valid_(false){};
  ROI(const std::array<cv::Point2d, 4>& points) { setPoints(std::move(points)); }
  ROI(const std::vector<cv::Point2d>& points) { setPoints(std::move(points)); };
  ROI(cv::Point2d p1, cv::Point2d p2, cv::Point2d p3, cv::Point2d p4) { setPoints(p1, p2, p3, p4); }

  void setPoints(const std::array<cv::Point2d, 4>& points);
  void setPoints(const std::vector<cv::Point2d>& points);
  void setPoints(cv::Point2d p1, cv::Point2d p2, cv::Point2d p3, cv::Point2d p4);
  const std::array<cv::Point2d, 4>& getPoints() const { return points_; };

  const cv::Point2d& getTopLeft() const { return points_[0]; }
  const cv::Point2d& getTopRight() const { return points_[1]; }
  const cv::Point2d& getBottomRight() const { return points_[2]; }
  const cv::Point2d& getBottomLeft() const { return points_[3]; }

  bool isValid() const { return valid_; }
  void setValid(bool valid) { valid_ = valid; }

 private:
  /**
   * @brief sort points in the following order: topleft, topright, bottomright, bottomleft
   * @return ordered points
   */
  void sort();

  std::array<cv::Point2d, 4> points_;  // Default order: topleft, topright, botttomright, bottomleft
  bool valid_;
};

}  // namespace evlc