#include "evlc_common/event.hpp"

#include <opencv2/core/eigen.hpp>

namespace evlc {

bool operator<(const Event& e1, const Event& e2) { return e1.t < e2.t; }

bool operator>(const Event& e1, const Event& e2) { return e1.t > e2.t; }

cv::Mat SAE::convertToGray(double span, bool polarity) {
  Eigen::MatrixXd mat;
  if (polarity) {
    mat = pos_map_;
  } else {
    mat = neg_map_;
  }
  double t_end = mat.maxCoeff();
  double t_start = t_end - span;
  mat = mat.unaryExpr([&t_start](double x) { return x > t_start ? x - t_start : 0; });
  mat /= (t_end - t_start);
  cv::Mat image;
  cv::eigen2cv(mat, image);
  image.convertTo(image, CV_32F);
  return image;
}

cv::Mat SAE::convertToRGB(double span, bool white_background) {
  cv::Mat image = cv::Mat::zeros(height_, width_, CV_8UC3);
  cv::Vec3b default_color;
  if (white_background) {
    default_color = cv::Vec3b(255, 255, 255);
  } else {
    default_color = cv::Vec3b(0, 0, 0);
  }
  const auto& pos = pos_map_;
  const auto& neg = neg_map_;
  double time = std::max(pos.maxCoeff() - span, 0.0);
  image.forEach<cv::Vec3b>([&](cv::Vec3b& pixel, const int* position) -> void {
    int row = position[0];
    int col = position[1];
    if (pos(row, col) > time and pos(row, col) >= neg(row, col)) {
      // positive event: red
      pixel = cv::Vec3b(0, 0, 255);
    } else if (neg(row, col) > time and neg(row, col) >= pos(row, col)) {
      // negative event: blue
      pixel = cv::Vec3b(255, 0, 0);
    } else {
      pixel = default_color;
    }
  });
  return image;
}

}  // namespace evlc