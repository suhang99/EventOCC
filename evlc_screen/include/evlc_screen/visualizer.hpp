#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include "evlc_common/buffer.hpp"
#include "evlc_common/event.hpp"
#include "evlc_common/types.hpp"

namespace evlc::screen {

class Visualizer {
 public:
  using Ptr = std::shared_ptr<Visualizer>;

  Visualizer(ros::NodeHandle nh, ros::NodeHandle nh_private, SAE::Ptr sae,
             TemporalBuffer<Event>::Ptr event_buffer);
  ~Visualizer() = default;

  /**
   * Whether visualizer is active
   */
  bool isActive() const { return active_; }

  /**
   * @brief visualize events in SAE periodically for ROS
   * @param timer ROS timer
   */
  void visualizeEvents(const ros::TimerEvent& timer);

  /**
   * @brief visualzie events in SAE
   */
  void visualizeEvents();

  /**
   * @brief update image for visualization
   * @param image image to visualize
   * @param caption image caption
   */
  void visualizeImage(const cv::Mat& image, const std::string& caption = "");

  /**
   * @brief visualzie a ROI on SAE image
   * @param roi ROI to visualize
   */
  void visualizeROI(const ROI& roi);

 private:
  ros::NodeHandle nh_, nh_private_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher sae_pub_;
  ros::Timer timer_;
  SAE::Ptr sae_;
  double frequency_, period_;
  double vis_frequency_, vis_period_;  // for visualizer
  TemporalBuffer<Event>::Ptr event_buffer_;
  bool active_;
};

}  // namespace evlc::screen