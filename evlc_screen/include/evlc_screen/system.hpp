#pragma once

#include <ros/ros.h>

#include <thread>

#include "evlc_common/buffer.hpp"
#include "evlc_common/camera.hpp"
#include "evlc_common/event.hpp"
#include "evlc_common/filter.hpp"
#include "evlc_common/timer.hpp"
#include "evlc_common/types.hpp"
#include "evlc_screen/decoder.hpp"
#include "evlc_screen/detector.hpp"
#include "evlc_screen/marker.hpp"
#include "evlc_screen/preprocessor.hpp"
#include "evlc_screen/tracker.hpp"
#include "evlc_screen/visualizer.hpp"
#include "evlc_msgs/EventArray.h"

namespace evlc::screen {

/**
 * @brief screen-based VLC system which contains all operations
 */
class System {
 public:
  using Ptr = std::shared_ptr<System>;

  /**
   * @brief the status of system
   */
  enum class Status {
    Idle,       // System is inactive
    Scanning,   // Scanning for signal
    Receiving,  // Actively receiving data
  };

  System(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  /**
   * @brief function to process events
   */
  void run();

  /**
   * @brief callback function to process events
   *
   * @param msg message containing incoming events
   */
  void eventsCallback(const evlc_msgs::EventArray::ConstPtr& msg);

  /**
   * @brief timer callback function for perodic check
   * @param timer ros timer
   */
  void timerCallback(const ros::TimerEvent& timer);

  /**
   * @brief activation function for the system
   * @param events incoming events
   */
  void tryWakeup(const std::vector<Event>& events);

  /**
   * @brief run detection module of the system
   * @param events incoming events
   */
  void runDetector(const std::vector<Event>& events);

  /**
   * @brief run Tracker module of the system
   * @param events incoming events
   */
  void runTracker(const std::vector<Event>& events);

  /**
   * @brief run Decoder module
   */
  void runDecoder();

  /**
   * @brief run system cleaner
   * @param t Time
   */
  void runCleaner(double t = 0);

 private:
  void findScreenRefresh(double t);

  // ROS parameters
  ros::NodeHandle nh_, nh_private_;
  ros::Subscriber events_sub_;
  ros::Timer timer_;

  // Event containers
  SAE::Ptr sae_;
  TemporalBuffer<Event>::Ptr event_buffer_;

  Status status_;
  Camera::Ptr camera_;
  ScreenMarker::Ptr marker_;

  // System modules
  Preprocessor::Ptr preprocessor_;
  Visualizer::Ptr visualizer_;
  Detector::Ptr detector_;
  Decoder::Ptr decoder_;
  Tracker::Ptr tracker_;

  // Timers to schedule each module
  Time t_detect_, t_track_;              // Scheduled time
  double detect_period_, track_period_;  // Operating period

  // Parameters for decoder
  std::vector<Time> t_decode_;
  int decode_times_;
  int decode_count_;
  bool set_decoder_timer_;

  // Some other parameters
  long long int event_count_;
  int min_events_to_start_;
  double temporal_detector_period_;
  double frequency_, period_;

  Profiler::Ptr profiler_;

  // Intermediate results
  std::array<cv::Point2d, 4> locator_points_;
  std::array<cv::Point2d, 8> tracker_points_;
  std::vector<Pose> poses_;
  std::vector<std::string> data_;

  bool record_tracker_;
  std::vector<double> tracker_record_times_;
  std::vector<double> tracker_records_;

  // Input and Output
  std::string event_topic_;
  bool read_from_rosbag_;
  std::string rosbag_path_;
  size_t poses_saved_, data_saved_;
  std::ofstream traj_output_, data_output_, log_output_;
};

}  // namespace evlc::screen
