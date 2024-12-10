#pragma once

#include <ros/ros.h>

#include "evlc_common/event.hpp"
#include "evlc_common/filter.hpp"
#include "evlc_common/types.hpp"
#include "evlc_msgs/EventArray.h"

namespace evlc::screen {

/**
 * @brief Preprocessor module for events
 */
class Preprocessor {
 public:
  using Ptr = std::shared_ptr<Preprocessor>;

  Preprocessor(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  /**
   * @brief event preprocess function for ROS topic
   * @param preprocessed events
   */
  void preprocess(const evlc_msgs::EventArray::ConstPtr& msg, std::vector<Event>& events);

 private:
  ros::NodeHandle nh_, nh_private_;
  std::vector<FilterBase::Ptr> filters_;
  SAE::Ptr sae_;
  int subsample_rate_;
};

}  // namespace evlc