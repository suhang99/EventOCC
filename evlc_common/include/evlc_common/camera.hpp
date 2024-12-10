#pragma once

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include "evlc_common/types.hpp"

namespace evlc {

class Camera {
 public:
  using Ptr = std::shared_ptr<Camera>;

  Camera(const ros::NodeHandle& nh);
  virtual ~Camera() = default;

  const int getHeight() const { return height_; }
  const int getWidth() const { return width_; }

  inline Eigen::Vector2d project(Eigen::Vector3d P) { return (K_ * P).hnormalized(); }
  inline Eigen::Vector3d backproject(Eigen::Vector2d p) { return K_inv_ * p.homogeneous(); }
  inline const cv::Mat& getCameraMatrix() const { return camera_matrix_; }
  inline const cv::Mat& getDistortion() const { return distortion_coeffs_; }
  inline const Eigen::Matrix3d& getK() const { return K_; }

 protected:
  int height_, width_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_coeffs_;
  Eigen::Matrix3d K_, K_inv_;
};

}  // namespace evlc