#include "evlc_screen/preprocessor.hpp"

namespace evlc::screen {

Preprocessor::Preprocessor(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  int height, width;
  nh_private_.getParam("camera/image_width", width);
  nh_private_.getParam("camera/image_height", height);
  sae_ = std::make_shared<SAE>(width, height);

  nh_private_.getParam("receiver/preprocessor/subsample_rate", subsample_rate_);
  ROS_INFO_STREAM("Event subsample rate: " << subsample_rate_);

  // Initialize filters
  bool refractory_active, neighbor_active;
  nh_private_.getParam("receiver/preprocessor/filters/refractory/active", refractory_active);
  nh_private_.getParam("receiver/preprocessor/filters/neighbor/active", neighbor_active);
  if (refractory_active) {
    double same_pol_thres, oppo_pol_thres;
    nh_private_.getParam("receiver/preprocessor/filters/refractory/same_pol_thres", same_pol_thres);
    nh_private_.getParam("receiver/preprocessor/filters/refractory/oppo_pol_thres", oppo_pol_thres);
    filters_.emplace_back(std::make_shared<RefractoryFilter>(sae_, same_pol_thres, oppo_pol_thres));
  }
  if (neighbor_active) {
    int neighbor_thres, radius;
    double max_age;
    nh_private_.getParam("receiver/preprocessor/filters/neighbor/neighbor_thres", neighbor_thres);
    nh_private_.getParam("receiver/preprocessor/filters/neighbor/radius", radius);
    nh_private_.getParam("receiver/preprocessor/filters/neighbor/max_age", max_age);
    filters_.emplace_back(std::make_shared<NeighborFilter>(sae_, neighbor_thres, radius, max_age));
  }
}

void Preprocessor::preprocess(const evlc_msgs::EventArray::ConstPtr& msg,
                              std::vector<Event>& events) {
  events.reserve(static_cast<size_t>(msg->events.size() / subsample_rate_));
  for (auto i = 0; i < msg->events.size(); i += subsample_rate_) {
    Event e(msg->events[i].x, msg->events[i].y, msg->events[i].ts.toSec(), msg->events[i].polarity);
    if (std::all_of(filters_.begin(), filters_.end(), [&e](auto f) { return f->run(e); })) {
      events.emplace_back(e);
    }
    sae_->update(e);
  }
}

}  // namespace evlc::screen