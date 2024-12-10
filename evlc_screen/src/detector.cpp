#include "evlc_screen/detector.hpp"

#include <cv_bridge/cv_bridge.h>
#include <ros/console.h>

#include <eigen3/unsupported/Eigen/FFT>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>

#include "evlc_common/timer.hpp"
#include "evlc_common/utils.hpp"

namespace evlc::screen {

Detector::Detector(ros::NodeHandle nh, ros::NodeHandle nh_private, SAE::Ptr sae,
                   TemporalBuffer<Event>::Ptr event_buffer, ScreenMarker::Ptr marker,
                   Visualizer::Ptr visualizer, Profiler::Ptr profiler)
    : sae_(sae),
      sae_delta_(std::make_shared<SAE>(sae->getWidth(), sae->getHeight())),
      marker_(marker),
      event_buffer_(event_buffer),
      blink_events_(std::make_shared<TemporalBuffer<BlinkEvent>>()),
      visualizer_(visualizer),
      profiler_(profiler) {
  result_.status = Status::Initial;
  nh_private.getParam("sender/frequency", frequency_);
  nh_private.getParam("receiver/detector/amplitude_threshold", amplitude_thres_);
  nh_private.getParam("receiver/detector/temporal_detector_period", temporal_detector_period_);
  period_ = 1.0 / frequency_;
  sigma_ = 0.1 * frequency_;
  blink_events_->setTimespan(2 * period_);
}

void Detector::update(const Event& e) {
  if (e.p == EVENT_POS) {
    // whether is neg->pos transition
    if (sae_->getPolarity(e.x, e.y) == EVENT_NEG) {
      // Get the time interval of pos-neg-pos event transition
      double t_prev = sae_delta_->getPosValue(e.x, e.y);
      blink_events_->push(BlinkEvent(e.x, e.y, e.t, e.t - t_prev));
      sae_delta_->update(e);
    }
  } else {
    // whether is pos->neg transition
    if (sae_->getPolarity(e.x, e.y) == EVENT_POS) {
      // Get the time interval of neg-pos-neg event transition
      double t_prev = sae_delta_->getNegValue(e.x, e.y);
      blink_events_->push(BlinkEvent(e.x, e.y, e.t, e.t - t_prev));
      sae_delta_->update(e);
    }
  }
}

const Detector::Result& Detector::detect(Time t) {
  blink_events_->clean();
  switch (result_.status) {
    case Status::Initial: {
      // 1. Detect ROI for temporal detector
      ROS_DEBUG("Detecting spatial pattern");
      ROI roi = detectSpatial();
      if (roi.isValid()) {
        result_.status = Status::Temporal;
        result_.roi = roi;
      }
      break;
    }
    case Status::Temporal: {
      // 2. Estimate phase from temporal detector
      ROS_DEBUG("Detecting temporal pattern");
      auto phase = detectTemporal(t);
      if (phase > 0) {
        result_.phase = phase;
#ifdef STATIC_SETUP
        result_.status = Status::Success;
#else
        result_.status = Status::Spatial;
#endif
      } else {
        result_.status = Status::Initial;
      }
      break;
    }
    case Status::Spatial: {
      ROS_DEBUG("Refining spatial detection");
      // 3. Refine marker's location
      auto roi = detectSpatial();
      roi = refineSpatial(roi);
      if (roi.isValid()) {
        result_.status = Status::Success;
        result_.roi = roi;
      }
      if (not roi.isValid()) {
        result_.status = Status::Initial;
        result_.phase = -1;
      }
      break;
    }
    default:
      break;
  }
  return result_;
}

ROI Detector::detectSpatial() {
  Eigen::MatrixXd evidence(sae_->getHeight(), sae_->getWidth());
  evidence.setZero();
  for (const auto& e : blink_events_->data()) {
    // Gaussian weight of each blink event
    double delta = 1.0 / e.dt - frequency_;
    evidence(e.y, e.x) += 1.0 / sigma_ * std::exp(-(std::pow(delta / sigma_, 2)) / 2.0);
  }

  // Normalize evidence matrix
  evidence /= evidence.maxCoeff();
  evidence *= 255;

  // Convert to opencv mat
  cv::Mat E;
  cv::eigen2cv(evidence, E);
  E.convertTo(E, CV_8U);

  cv::Mat frame = sae_->convertToRGB(period_);
  

  // Preprocessing for evidence map
  // cv::medianBlur(E, E, 3);
  cv::threshold(E, E, 50, 255, cv::THRESH_BINARY);

  // visualizer_->visualizeImage(E);

  // Extract Contours
  std::vector<std::vector<cv::Point>> cv_contours;
  cv::findContours(E, cv_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
  auto num_contours = cv_contours.size();

  cv::Mat image = E.clone();
  cv::RNG rng(42);

  // Extract convex hull
  cv::cvtColor(E, E, cv::COLOR_GRAY2RGB);
  std::vector<std::vector<cv::Point>> cv_polygons(num_contours), cv_hulls(num_contours);
  for (auto i = 0; i < num_contours; ++i) {
    cv::convexHull(cv_contours[i], cv_hulls[i]);
    cv::approxPolyDP(cv_hulls[i], cv_polygons[i], cv::arcLength(cv_hulls[i], true) * 0.05, true);
    cv::drawContours(image, cv_polygons, (int)i,
                     cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256)));
  }

  // Find the most possible ROI
  std::vector<ROI> roi_candidates;
  for (int i = 0; i < num_contours; ++i) {
    if (auto roi = findValidROI(cv_polygons[i]); roi.isValid()) {
      roi_candidates.emplace_back(roi);
    }
  }

  ROI best_roi;
  if (not roi_candidates.empty()) {
    // Choose the locator with the largest area.
    double max_area = 0;
    best_roi = roi_candidates.front();
    for (auto& roi : roi_candidates) {
      std::vector<cv::Point> points;
      points.reserve(roi.getPoints().size());
      for (auto& p : roi.getPoints()) {
        points.emplace_back(cvRound(p.x), cvRound(p.y));
      }
      double area = cv::contourArea(points);
      if (area > max_area) {
        best_roi = roi;
        max_area = area;
      }
    }
  }
  return best_roi;
}

ROI Detector::findValidROI(const std::vector<cv::Point>& cv_points) {
  if (cv_points.size() < 3) {
    return {};
  }

  // Find the length of each edge
  auto num_points = cv_points.size();
  std::vector<double> edge_length;
  for (auto i = 0; i < num_points - 1; ++i) {
    edge_length.emplace_back(cv::norm(cv_points[i] - cv_points[i + 1]));
  }
  edge_length.emplace_back(cv::norm(cv_points.back() - cv_points.front()));

  ROI selected_roi;

  for (auto i = 0; i < num_points; ++i) {
    // Length check of adjacent edges
    if (edge_length[i] < length_thres_ or edge_length[(i + 1) % num_points] < length_thres_) {
      continue;
    }

    // Angle check of adjacent edges
    cv::Point2d p1(cv_points[i]), p2(cv_points[(i + 1) % num_points]);
    cv::Point2d p3(cv_points[(i + 1) % num_points]), p4(cv_points[(i + 2) % num_points]);
    Eigen::Vector2d vec1(p2.x - p1.x, p2.y - p1.y);
    Eigen::Vector2d vec2(p4.x - p3.x, p4.y - p3.y);
    double cosine = vec1.dot(vec2) / (vec1.norm() * vec2.norm());
    if (abs(cosine) > cos_thres_) {
      continue;
    }

    // Ratio check of adjacent edges
    double len1 = edge_length[i];
    double len2 = edge_length[(i + 1) % num_points];
    if (std::abs(len1 - len2) / std::max(len1, len2) > length_ratio_thres_) {
      continue;
    }

    // Approximate corners by reflection
    if (p3 == p2) {
      p3 = (p1 + p4) - p2;
    } else if (p4 == p1) {
      p4 = (p2 + p3) - p1;
    } else {
      ROS_ERROR("Internal error in findValidQuad()");
    }

    selected_roi.setPoints(p1, p2, p3, p4);
    break;
  }
  return selected_roi;
}

double Detector::detectTemporal(Time t_detect) {
  event_buffer_->clean();
  double step = temporal_detector_period_ / (10 * frequency_);
  // Consider events in the last second
  double t_start = t_detect - temporal_detector_period_;

  // Distribute the event counts into bins
  int num_bins = static_cast<int>(std::ceil(temporal_detector_period_ / step));
  std::vector<double> event_count(num_bins, 0);

  const auto& top_left = result_.roi.getTopLeft();
  const auto& bottom_right = result_.roi.getBottomRight();

  for (const auto& e : event_buffer_->data()) {
    if (e.t > t_start) {
      // Note that we use positive events for fourier analysis
      if (e.p == EVENT_POS and e.x > top_left.x and e.x < bottom_right.x and e.y > top_left.y and
          e.y < bottom_right.y) {
        ++event_count[static_cast<int>(std::floor((e.t - t_start) / step))];
      }
    }
  }

  // Apply DFT to find phase
  std::vector<double> freqs;
  auto n = event_count.size();
  double t = step * n;
  if (n % 2 == 0) {
    for (int i = 0; i <= n / 2 - 1; i++) {
      freqs.emplace_back(i / t);
    }
    for (int i = -n / 2; i < 0; i++) {
      freqs.emplace_back(i / t);
    }
  } else {
    for (int i = 0; i <= (n - 1) / 2; i++) {
      freqs.emplace_back(i / t);
    }
    for (int i = -(n - 1) / 2; i < 0; i++) {
      freqs.emplace_back(i / t);
    }
  }

  Eigen::FFT<double> fft;
  std::vector<std::complex<double>> x_freq;
  fft.fwd(x_freq, event_count);

  double amplitude = std::abs(x_freq[static_cast<int>(frequency_)]) / num_bins;
  double phase = std::arg(x_freq[static_cast<int>(frequency_)]);
  double t_peak = (2 * M_PI - phase) / (2 * M_PI * frequency_) + t_start;
  if (amplitude > amplitude_thres_) {
    return t_peak;
  }
  return -1;
}

ROI Detector::refineSpatial(ROI roi) {
  std::vector<cv::Point> points;
  for (auto& p : roi.getPoints()) {
    points.emplace_back(p.x, p.y);
  }

  // Preprocess SAE frame
  cv::Mat frame = sae_->convertToGray(period_);
  frame *= 255;
  frame.convertTo(frame, CV_8UC3);

  cv::medianBlur(frame, frame, 3);
  cv::threshold(frame, frame, 50, 255, cv::THRESH_BINARY);

  // Extract contour from frame
  std::vector<std::vector<cv::Point>> cv_contours;
  std::vector<ROI> roi_candidates;
  cv::findContours(frame, cv_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  cv::Mat image = frame.clone();
  cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
  cv::Mat image2 = image.clone();
  cv::RNG rng(42);

  // Find valid contour as candidates
  int contour_idx = 0;
  for (const auto& contour : cv_contours) {
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, cv::arcLength(contour, true) * 0.02, true);

    auto color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
    cv::drawContours(image, cv_contours, contour_idx++, color);

    if (findValidROI(approx).isValid() and approx.size() == 4) {
      std::vector<std::vector<cv::Point>> temp{approx};
      cv::drawContours(image2, temp, 0, cv::Scalar(0, 255, 0), 3);
      roi_candidates.emplace_back(approx[0], approx[1], approx[2], approx[3]);
    }
  }

  if (roi_candidates.empty()) {
    return ROI();
  }

  // Find the roi that best aligned with the original one
  auto selected_roi = *std::max_element(
      roi_candidates.begin(), roi_candidates.end(), [this, &points](auto roi1, auto roi2) {
        std::vector<cv::Point> contour1, contour2;
        for (const auto& p : roi1.getPoints()) {
          contour1.emplace_back(p.x, p.y);
        }
        for (const auto& p : roi2.getPoints()) {
          contour2.emplace_back(p.x, p.y);
        }
        return computeIoU(points, contour1) < computeIoU(points, contour2);
      });
  return selected_roi;
}

void Detector::reset() {
  result_.phase = -1;
  result_.status = Status::Initial;
  result_.roi = ROI();
}

}  // namespace evlc::screen