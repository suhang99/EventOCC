#include "evlc_screen/system.hpp"

#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <functional>

#include "evlc_common/timer.hpp"

namespace evlc::screen {

System::System(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      status_(Status::Idle),
      event_count_(0),
      decode_count_(-1),
      set_decoder_timer_(true) {
  std::string verbose_level;
  nh_private_.getParam("verbose_level", verbose_level);
  if (verbose_level == "debug") {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  } else if (verbose_level == "info") {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  } else if (verbose_level == "warn") {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn);
  } else if (verbose_level == "error") {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Error);
  } else {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Fatal);
  }

  std::string log_output_path;
  nh_private_.getParam("log_output_path", log_output_path);
  ROS_INFO_STREAM("Saving log to " << log_output_path);
  log_output_.open(log_output_path);

  nh_private_.getParam("rosbag_play", read_from_rosbag_);
  if (read_from_rosbag_) {
    nh_private_.getParam("rosbag_file", rosbag_path_);
    ROS_INFO_STREAM("Read rosbag from " << rosbag_path_);
  }

  // Set event topic and subscriber
  nh_private_.getParam("event_topic", event_topic_);
  events_sub_ =
      nh_.subscribe<evlc_msgs::EventArray>(event_topic_, 10000, &System::eventsCallback, this);
  timer_ = nh_.createTimer(ros::Duration(1.0), &System::timerCallback, this);

  // Set camera parameters
  camera_ = std::make_shared<Camera>(nh_private);
  int height = camera_->getHeight();
  int width = camera_->getWidth();

  // Initialize screen parameters
  nh_private_.getParam("sender/frequency", frequency_);
  period_ = 1.0 / frequency_;
  ROS_INFO_STREAM("Screen data frequency: " << frequency_ << " Hz");

  // Initialize marker parameters
  double marker_size;
  int payload_cells;
  nh_private_.getParam("sender/marker_size", marker_size);
  nh_private_.getParam("sender/payload_cells", payload_cells);
  ROS_INFO_STREAM("Marker size = " << marker_size << " m");
  ROS_INFO_STREAM("Payload = " << payload_cells << " cells");
  log_output_ << "marker_size = " << marker_size << endl;
  log_output_ << "payload_cells = " << payload_cells << endl;
  marker_ = std::make_shared<ScreenMarker>(marker_size, payload_cells);

  // Initialize event container
  sae_ = std::make_shared<SAE>(width, height);
  event_buffer_ = std::make_shared<TemporalBuffer<Event>>();
  event_buffer_->setTimespan(1);

  profiler_ = std::make_shared<Profiler>();

  // Initialize modules
  preprocessor_ = std::make_shared<Preprocessor>(nh_, nh_private_);
  visualizer_ = std::make_shared<Visualizer>(nh_, nh_private_, sae_, event_buffer_);
  tracker_ = std::make_shared<Tracker>(nh_, nh_private_, sae_, event_buffer_, camera_, marker_,
                                       visualizer_, profiler_);
  detector_ = std::make_shared<Detector>(nh_, nh_private_, sae_, event_buffer_, marker_,
                                         visualizer_, profiler_);
  decoder_ = std::make_shared<Decoder>(nh_, nh_private_, sae_, marker_, visualizer_);

  // Initialize module periods
  int detect_freq, track_freq;
  nh_private_.getParam("receiver/detector/frequency", detect_freq);
  nh_private_.getParam("receiver/detector/temporal_detector_period", temporal_detector_period_);
  nh_private_.getParam("receiver/tracker/frequency", track_freq);
  nh_private_.getParam("receiver/min_events_to_start", min_events_to_start_);
  detect_period_ = 1.0 / detect_freq;
  track_period_ = 1.0 / track_freq;

  // Initialize parameters for decoder
  nh_private_.getParam("receiver/decoder/times", decode_times_);
  t_decode_.resize(decode_times_);

  double detect_start_time = 0;
  nh_private_.getParam("receiver/detector/start_time", detect_start_time);
  // Initialize time scheduler
  t_detect_ = detect_start_time;
  t_track_ = 0;

  // add data and poses output
  poses_saved_ = 0;
  std::string traj_output_path;
  nh_private_.getParam("traj_output_path", traj_output_path);
  traj_output_.open(traj_output_path);
  ROS_INFO_STREAM("Saving tracker results to " << traj_output_path);

  data_saved_ = 0;
  std::string data_output_path;
  nh_private_.getParam("data_output_path", data_output_path);
  data_output_.open(data_output_path);
  ROS_INFO_STREAM("Saving decoder results to " << data_output_path);
}

void System::run() {
  // Setup event callback function
  std::function<void(const evlc_msgs::EventArray::ConstPtr&)> event_callback =
      std::bind(&System::eventsCallback, this, std::placeholders::_1);

  if (read_from_rosbag_) {
    // Setup rosbag input
    std::unique_ptr<rosbag::Bag> bag = std::make_unique<rosbag::Bag>();
    bag->open(rosbag_path_, rosbag::bagmode::Read);
    std::unique_ptr<rosbag::View> view;
    rosbag::View::iterator current;

    if (bag->isOpen()) {
      view = std::make_unique<rosbag::View>(*bag, static_cast<rosbag::TopicQuery>(event_topic_));
      current = view->begin();

      auto msg = rosbag::MessageInstance{*current};
      auto ins = msg.instantiate<evlc_msgs::EventArray>();

      double t_visualize = ins->events.front().ts.toSec();

      while (current != view->end()) {
        auto msg = rosbag::MessageInstance{*current};
        auto ins = msg.instantiate<evlc_msgs::EventArray>();
        if (ins != nullptr) {
          event_callback(ins);
        }

        runCleaner(ins->events.back().ts.toSec());

        if (ins->events.back().ts.toSec() > t_visualize) {
          visualizer_->visualizeEvents();
          tracker_->runVisualizer();
        }

        while (ins->events.back().ts.toSec() > t_visualize) {
          t_visualize += 0.1;
        }

        current++;
      }
      bag->close();
      ROS_INFO("Finish processing rosbag. Shutdown ROS");
      ros::shutdown();
    } else {
      ROS_ERROR_STREAM("Cannot open rosbag from " << rosbag_path_);
      return;
    }
  }
}

void System::eventsCallback(const evlc_msgs::EventArray::ConstPtr& msg) {
  std::vector<Event> events;
  preprocessor_->preprocess(msg, events);

  // Main entrance of system
  switch (status_) {
    case Status::Idle:
      tryWakeup(events);
      break;
    case Status::Scanning:
      runDetector(events);
      break;
    case Status::Receiving:
      runTracker(events);
      break;
    default:
      runDetector(events);
      break;
  }
}

void System::tryWakeup(const std::vector<Event>& events) {
  for (const auto& event : events) {
    sae_->update(event);
    event_buffer_->push(event);
  }
  // Wake up if has enough events
  if (event_buffer_->size() > min_events_to_start_) {
    // Set system to active
    status_ = Status::Scanning;
    ROS_INFO("Activate system");
  }
}

void System::runDetector(const std::vector<Event>& events) {
  for (const auto& e : events) {
    // Note that update detector first
    detector_->update(e);
    sae_->update(e);
    event_buffer_->push(e);

    // Not yet for detection
    if (e.t < t_detect_) {
      continue;
    }
    auto result = detector_->detect(t_detect_);

    // Update timer according to the detection result
    switch (result.status) {
      case Detector::Status::Temporal:
        // Find initial ROI, move to temporal detection
        t_detect_ += temporal_detector_period_;
        break;
      case Detector::Status::Spatial: {
        // Temporal detection finished, move to spatial detection
        auto num_periods = static_cast<long int>(std::ceil(e.t - result.phase) * frequency_);
        // Set detection time when one screen frame has fully refreshed
        t_detect_ = num_periods * period_ + result.phase + 0.5 * period_;
        break;
      }
      case Detector::Status::Success: {
        // Temporal detection finished, move to spatial detection
        auto num_periods = static_cast<long int>(std::ceil(e.t - result.phase) * frequency_);

        t_decode_[0] = num_periods * period_ + result.phase - 0.5 * period_;

        // Disable detector
        detector_->reset();

        auto locators = result.roi.getPoints();
        auto bits = decoder_->extractMarkerBits(locators, period_);
        int roll = decoder_->getMarkerRoll(bits);

        if (roll >= 0) {
          std::rotate(locators.begin(), locators.begin() + roll, locators.end());
          locator_points_ = locators;
          // update decoder and tracker
          ROS_INFO("Activate tracker and decoder");
          tracker_->activate(locators, e.t);
          tracker_points_ = tracker_->getTrackerPoints();

          // Set timer for tracker and decoder
          t_track_ = e.t + track_period_;

          while (t_decode_.front() < e.t) {
            for (auto& t : t_decode_) {
              t += period_;
            }
          }

          status_ = Status::Receiving;
        } else {
          ROS_DEBUG_STREAM("Failed to detect inner locator. roll = " << roll);
        }
        break;
      }
      default:
        // Set next invoke time
        while (t_detect_ <= e.t) {
          t_detect_ += detect_period_;
        }
        break;
    }
  }
}

void System::runTracker(const std::vector<Event>& events) {

  for (const auto& e : events) {
    sae_->update(e);
    event_buffer_->push(e);

#ifndef STATIC_SETUP
    tracker_->update(e);

    if (e.t > t_track_) {
      auto T_world_cam = tracker_->track(e.t);
      tracker_points_ = tracker_->getTrackerPoints();

      poses_.emplace_back(e.t, T_world_cam);

      // Set next invoke time
      while (t_track_ < e.t) {
        t_track_ += track_period_;
      }
    }
#endif

    if (decode_count_ == -1) {
      findScreenRefresh(e.t);
      continue;
    }

    // Decode several times for each screen frame
    if (e.t > t_decode_[decode_count_]) {
      runDecoder();
    }
  }

  // Reset timers when tracker is inactive
  if (not tracker_->isActive()) {
    t_detect_ = events.back().t + detect_period_;
    t_track_ = -1;
    t_decode_.clear();
  }
}

void System::runDecoder() {

  auto decoded_data = decoder_->decode(tracker_points_);

  if (not decoded_data.empty()) {
    data_.emplace_back(decoded_data);
  }

  ++decode_count_;
  if (decode_count_ == decode_times_) {
    decode_count_ = 0;
    for (auto& t : t_decode_) {
      t += period_;
    }
    t_detect_ += period_;
  }
}

void System::timerCallback(const ros::TimerEvent& timer) { runCleaner(); }

void System::runCleaner(double t) {
  // Clean events
  event_buffer_->clean();

  // Write poses to file
  while (poses_saved_ < poses_.size()) {
    auto tum_data = poses_[poses_saved_].convertToTUM();
    traj_output_ << tum_data[0] << " " << tum_data[1] << " " << tum_data[2] << " " << tum_data[3]
                 << " " << tum_data[4] << " " << tum_data[5] << " " << tum_data[6] << " "
                 << tum_data[7] << std::endl;
    ++poses_saved_;
  }

  // write decoded message to file
  while (data_saved_ < data_.size()) {
    data_output_ << data_[data_saved_] << std::flush;
    ++data_saved_;
  }
}

void System::findScreenRefresh(double t) {
  static bool to_initialize = true;
  static double t_check = t_decode_.front();
  static double t_start = -1, t_end = -1;
  static const int total_check = 50;
  static double check_step = period_ / 2 / (total_check - 1);
  static int check_count = 0;
  static const double thres = 0.25 * marker_->inner_cells;

  // Make sure the starting time to check is later than the first incoming event
  if (to_initialize) {
    while (t > t_check) {
      t_check += period_;
    }
    to_initialize = false;
  }

  if (t > t_check) {
    auto bits = decoder_->extractMarkerBits(marker_->trackersToLocators(tracker_points_),
                                            period_ / decode_times_);
    if (t_start < 0) {
      if (bits.row(1).cast<int>().sum() > thres) {
        t_start = t_check;
      }
    } else if (t_end < 0) {
      if (bits.row(bits.rows() - 2).cast<int>().sum() > thres) {
        t_end = t_check + 0.1 * period_;
        if (t_end - t_start > period_) {
          ROS_DEBUG("Failed to find screen refresh time");
        }
        double decode_step = t_end - t_start;
        if (decode_times_ > 1) {
          decode_step = (t_end - t_start) / (decode_times_ - 1);
        }
        for (int i = 0; i < decode_times_; ++i) {
          t_decode_[i] = t_start + i * decode_step + period_;
        }
        decode_count_ = 0;
        ROS_DEBUG("Decoder timers are set");
        log_output_ << "t_decode = [" << t_decode_[0];
        for (int i = 1; i < t_decode_.size(); ++i) {
          log_output_ << ", " << t_decode_[i];
        }
        log_output_ << "]" << endl;
      }
    }
    ++check_count;
    t_check += check_step;
    if (check_count == total_check) {
      t_start = -1;
      t_end = -1;
    }
  }
}

}  // namespace evlc::screen