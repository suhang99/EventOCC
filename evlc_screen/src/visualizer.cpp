#include "evlc_screen/visualizer.hpp"

#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include "evlc_common/utils.hpp"

namespace evlc::screen {

Visualizer::Visualizer(ros::NodeHandle nh, ros::NodeHandle nh_private, SAE::Ptr sae,
                       TemporalBuffer<Event>::Ptr event_buffer)
    : nh_(nh), nh_private_(nh_private), sae_(sae), event_buffer_(event_buffer) {
  image_transport::ImageTransport it(nh);
  image_pub_ = it.advertise("/evlc/image", 10);
  sae_pub_ = it.advertise("/evlc/events", 10);
  nh_private_.getParam("sender/frequency", frequency_);

  period_ = 1.0 / frequency_;
  nh_private_.getParam("visualizer/active", active_);
  nh_private_.getParam("visualizer/frequency", vis_frequency_);

  vis_period_ = 1.0 / vis_frequency_;
  timer_ = nh_.createTimer(ros::Duration(vis_period_), &Visualizer::visualizeEvents, this);

  if (active_) {
    ROS_INFO("Visualizer is ON");
    ROS_INFO_STREAM("Visualizer update frequency: " << vis_frequency_ << " hz");
  } else {
    ROS_INFO("Visualizer is OFF");
  }
}

void Visualizer::visualizeEvents(const ros::TimerEvent& timer) { visualizeEvents(); }

void Visualizer::visualizeEvents() {
  // Publish SAE image
  sensor_msgs::ImagePtr msg;
  cv::Mat sae_image = sae_->convertToRGB(period_, true);
  // Add the number of events
  std::string caption;
  auto event_rate = event_buffer_->size() / event_buffer_->getTimespan();
  if (event_rate > 1e6) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(1) << event_rate / 1e6;
    caption = out.str() + "Mev/s";
  } else if (event_rate > 1e3) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(1) << event_rate / 1e3;
    caption = out.str() + "Kev/s";
  } else {
    caption = std::to_string(event_rate) + "ev/s";
  }
  cv::putText(sae_image, caption, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
              cv::Scalar(0, 255, 0), 2);
  std::ostringstream out;
  out << std::fixed << std::setprecision(1) << sae_->getLatestTime() << "s";
  cv::putText(sae_image, out.str(), cv::Point(20, sae_->getHeight() - 30), cv::FONT_HERSHEY_SIMPLEX,
              1, cv::Scalar(0, 255, 0), 2);

  std_msgs::Header header;
  header.stamp = ros::Time::now();
  msg = cv_bridge::CvImage(header, "bgr8", sae_image).toImageMsg();

  sae_pub_.publish(msg);
}

void Visualizer::visualizeImage(const cv::Mat& image, const std::string& caption) {
  // Publish incoming image
  cv::Mat frame;

  if (image.channels() == 1) {
    cv::cvtColor(image, frame, cv::COLOR_GRAY2BGR);
  } else {
    frame = image.clone();
  }
  if (frame.type() == CV_32FC3) {
    frame.convertTo(frame, CV_8UC3);
    frame = frame * 255;
  }

  // Add image caption at the top left corner
  if (not caption.empty()) {
    cv::putText(frame, caption, cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                cv::Scalar(0, 255, 0), 2);
  }

  sensor_msgs::ImagePtr msg;
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
  image_pub_.publish(msg);
}

void Visualizer::visualizeROI(const ROI& roi) {
  sensor_msgs::ImagePtr msg;
  cv::Mat sae_image = sae_->convertToRGB(period_);
  drawQuad(sae_image, roi.getPoints());
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  msg = cv_bridge::CvImage(header, "bgr8", sae_image).toImageMsg();
  image_pub_.publish(msg);
}

}  // namespace evlc::screen