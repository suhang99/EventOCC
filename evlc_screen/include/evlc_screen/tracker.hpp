#pragma once

#include "evlc_common/buffer.hpp"
#include "evlc_common/camera.hpp"
#include "evlc_common/event.hpp"
#include "evlc_common/timer.hpp"
#include "evlc_common/types.hpp"
#include "evlc_screen/edge_tracker.hpp"
#include "evlc_screen/marker.hpp"
#include "evlc_screen/optimizer.hpp"
#include "evlc_screen/visualizer.hpp"

namespace evlc::screen {

class Tracker {
 public:
  using Ptr = std::shared_ptr<Tracker>;

  Tracker(ros::NodeHandle nh, ros::NodeHandle nh_private, SAE::Ptr sae,
          TemporalBuffer<Event>::Ptr event_buffer, Camera::Ptr camera, ScreenMarker::Ptr marker,
          Visualizer::Ptr visualizer, Profiler::Ptr profiler);
  ~Tracker();

  /**
   * @brief Run tracking algorithm for the marker
   * @param t current track time
   * @return Relative pose
   */
  Eigen::Isometry3d track(Time t);

  /**
   * @brief Update one event
   */
  void update(const Event& e);

  /**
   * @brief Activate tracker
   * @param locators locator points
   * @param t track time
   */
  void activate(const std::array<cv::Point2d, 4>& locators, Time t);

  /**
   * @brief return the status of tracker
   * @return true is active, otherwise false
   */
  bool isActive() const { return active_; }

  /**
   * @brief Getter of tracker points
   * @return tracker points
   */
  std::array<cv::Point2d, 8> getTrackerPoints();

  /**
   * @brief periodic function
   * @param timer ros timer
   */
  void timerCallback(const ros::TimerEvent& timer);

  /**
   * @brief function to visualize tracker
   */
  void runVisualizer();

 private:
  SAE::Ptr sae_;
  TemporalBuffer<Event>::Ptr event_buffer_;
  Camera::Ptr camera_;
  std::array<EdgeTracker::Ptr, 8> edge_trackers_;
  std::array<cv::Point2d, 8> tracker_points_;
  Optimizer::Ptr optimizer_;
  ScreenMarker::Ptr marker_;
  Visualizer::Ptr visualizer_;
  Profiler::Ptr profiler_;

  std::ofstream traj_output_;  // File output to record trajectory
  int distance_threshold_;
  Eigen::Isometry3d T_world_cam_;  // Camera pose in world frame
  Eigen::Isometry3d T_cam_world_;  // World point in camera frame
  double alpha_;                   // parameter for edge tracker
  double frequency_, period_;
  bool active_;
  ros::Timer ros_timer_;
};

}  // namespace evlc::screen