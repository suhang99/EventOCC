#include <ros/ros.h>

#include "evlc_screen/system.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "evlc_screen_node");
  ros::NodeHandle nh, nh_private("~");
  evlc::screen::System system(nh, nh_private);
  system.run();
  ros::spin();
  ros::shutdown();

  return 0;
}