#include "evlc_screen/tracker.hpp"

#include <ros/ros.h>

#include <opencv2/core/eigen.hpp>

#include "evlc_common/timer.hpp"
#include "evlc_common/utils.hpp"

namespace evlc::screen {

Tracker::Tracker(ros::NodeHandle nh, ros::NodeHandle nh_private, SAE::Ptr sae,
                 TemporalBuffer<Event>::Ptr event_buffer, Camera::Ptr camera,
                 ScreenMarker::Ptr marker, Visualizer::Ptr visualizer, Profiler::Ptr profiler)
    : sae_(sae),
      event_buffer_(event_buffer),
      camera_(camera),
      marker_(marker),
      T_world_cam_(Eigen::Isometry3d::Identity()),
      T_cam_world_(Eigen::Isometry3d::Identity()),
      active_(false),
      visualizer_(visualizer),
      profiler_(profiler) {
  std::string traj_path;
  nh_private.getParam("traj_path", traj_path);
  traj_output_.open(traj_path);

  optimizer_ = std::make_shared<Optimizer>(profiler);

  nh_private.getParam("sender/frequency", frequency_);
  period_ = 1.0 / frequency_;

  nh_private.getParam("receiver/tracker/distance_threshold", distance_threshold_);
  nh_private.getParam("receiver/tracker/alpha", alpha_);

  // Setup visualizer parameters
  double vis_frequency;
  nh_private.getParam("visualizer/frequency", vis_frequency);
  ros_timer_ = nh.createTimer(ros::Duration(1 / vis_frequency), &Tracker::timerCallback, this);
}

Tracker::~Tracker() { traj_output_.close(); }

void Tracker::activate(const std::array<cv::Point2d, 4>& locators, Time time) {
  cv::Mat r_vec, t_vec, R_mat;
  cv::solvePnP(marker_->locator_world_points, locators, camera_->getCameraMatrix(),
               camera_->getDistortion(), r_vec, t_vec, false, cv::SOLVEPNP_ITERATIVE);
  cv::Rodrigues(r_vec, R_mat);

  // Get camera pose in world frame (T_world_cam)
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  cv::cv2eigen(R_mat, R);
  cv::cv2eigen(t_vec, t);
  T_cam_world_.prerotate(R);
  T_cam_world_.pretranslate(t);
  T_world_cam_ = T_cam_world_.inverse();

  std::array<Eigen::Vector3d, 8> tracker_world_points;
  std::array<Eigen::Vector2d, 8> tracker_image_points;
  for (auto i = 0; i < 8; ++i) {
    tracker_world_points[i].x() = marker_->tracker_world_points[i].x;
    tracker_world_points[i].y() = marker_->tracker_world_points[i].y;
    tracker_world_points[i].z() = marker_->tracker_world_points[i].z;
    tracker_image_points[i] =
        camera_->project(transformPoint(T_cam_world_, tracker_world_points[i]));
  }

  // Initialize edge trackers
  for (int i = 0; i < 4; ++i) {
    edge_trackers_[i] =
        std::make_shared<EdgeTracker>(tracker_world_points[i], tracker_world_points[(i + 1) % 4],
                                      tracker_image_points[i], tracker_image_points[(i + 1) % 4]);
    edge_trackers_[i]->setThreshold(distance_threshold_, alpha_);
  }

  cv::Mat image = sae_->convertToRGB(period_, true);
  std::array<cv::Point2d, 4> points1, points2;
  for (int i = 0; i < 4; ++i) {
    points1[i].x = tracker_image_points[i].x();
    points1[i].y = tracker_image_points[i].y();
    points2[i].x = tracker_image_points[i + 4].x();
    points2[i].y = tracker_image_points[i + 4].y();
  }
  drawQuad(image, points1);
  drawQuad(image, points2);

  visualizer_->visualizeImage(image);

  for (int i = 4; i < 8; ++i) {
    edge_trackers_[i] = std::make_shared<EdgeTracker>(
        tracker_world_points[i], tracker_world_points[(i + 1) % 4 + 4], tracker_image_points[i],
        tracker_image_points[(i + 1) % 4 + 4]);
    edge_trackers_[i]->setThreshold(distance_threshold_, alpha_);
  }

  optimizer_->initialize(camera_->getK(), edge_trackers_);

  active_ = true;
}

void Tracker::update(const Event& e) {
  for (const auto& edge : edge_trackers_) {
    edge->updateEvent(e);
  }
}

Eigen::Isometry3d Tracker::track(Time t) {
  double half_thres = distance_threshold_ / 2;

  if (std::any_of(edge_trackers_.begin(), edge_trackers_.end(),
                  [](auto& edge) { return edge->needUpdate(); })) {
    // Get new pose from optimizer
#ifdef ENABLE_PROFILER
    profiler_->start("optimizer");
#endif
    T_cam_world_ = optimizer_->optimize(T_cam_world_);
#ifdef ENABLE_PROFILER
    profiler_->stop("optimizer");
#endif
    T_world_cam_ = T_cam_world_.inverse();

    // Update edge trackers
    for (auto& edge : edge_trackers_) {
      auto world_point_begin = transformPoint(T_cam_world_, edge->getWorldPointBegin());
      auto world_point_end = transformPoint(T_cam_world_, edge->getWorldPointEnd());
      edge->updateEdge(camera_->project(world_point_begin), camera_->project(world_point_end));
    }
  }

  return T_world_cam_;
}

std::array<cv::Point2d, 8> Tracker::getTrackerPoints() {
  std::array<cv::Point2d, 8> points;
  for (int i = 0; i < 8; ++i) {
    points[i].x = edge_trackers_[i]->getImagePointBegin().x();
    points[i].y = edge_trackers_[i]->getImagePointBegin().y();
  }
  return points;
}

void Tracker::timerCallback(const ros::TimerEvent& ros_timer) { runVisualizer(); }

void Tracker::runVisualizer() {
  // Visualize latest marker position
  if (active_) {
    cv::Mat image = sae_->convertToRGB(period_, true);
    for (const auto& edge : edge_trackers_) {
      auto begin_point = edge->getImagePointBegin();
      auto end_point = edge->getImagePointEnd();
      cv::Point2d cv_begin(begin_point.x(), begin_point.y());
      cv::Point2d cv_end(end_point.x(), end_point.y());
      cv::line(image, cv_begin, cv_end, cv::Scalar(0, 255, 0), 2);
    }
    visualizer_->visualizeImage(image);
  }
}

}  // namespace evlc::screen