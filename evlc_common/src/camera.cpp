#include "evlc_common/camera.hpp"

#include <opencv2/core/eigen.hpp>
namespace evlc {

Camera::Camera(const ros::NodeHandle& nh) {
  nh.getParam("camera/image_width", width_);
  nh.getParam("camera/image_height", height_);
  ROS_INFO_STREAM("Camera resolution: " << width_ << " x " << height_);

  std::vector<double> camera_matrix_data;
  nh.getParam("camera/camera_matrix/data", camera_matrix_data);
  assert(camera_matrix_data.size() == 9);

  std::vector<double> distortion;
  nh.getParam("camera/distortion_coefficients/data", distortion);

  camera_matrix_ = cv::Mat(3, 3, CV_64F, camera_matrix_data.data()).clone();
  distortion_coeffs_ = cv::Mat(1, distortion.size(), CV_64F, distortion.data()).clone();
  cv::cv2eigen(camera_matrix_, K_);
  K_inv_ = K_.inverse();
}

}  // namespace evlc