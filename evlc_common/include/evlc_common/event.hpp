#pragma once

#include <deque>

#include "evlc_common/types.hpp"

namespace evlc {

/**
 * @brief Basic event class
 */
class Event {
 public:
  using Ptr = std::shared_ptr<Event>;

  Event() = default;
  Event(uint16_t x, uint16_t y, Time t, bool p) : x(x), y(y), t(t), p(p) {}

  // Declare comparison functions
  friend bool operator<(const Event& e1, const Event& e2);
  friend bool operator>(const Event& e1, const Event& e2);

  uint16_t x, y;
  Time t;
  bool p;
};

/**
 * @brief Blinking event class
 */
class BlinkEvent {
 public:
  BlinkEvent() = default;
  BlinkEvent(int x, int y, double t, double dt) : x(x), y(y), t(t), dt(dt) {}
  int x, y;
  double t, dt;
};

/**
 * @brief Surface of Active Events
 */
class SAE {
 public:
  using Ptr = std::shared_ptr<SAE>;

  SAE() = default;
  SAE(size_t width, size_t height) : width_(width), height_(height) {
    pos_map_ = Eigen::MatrixXd::Zero(height, width);
    neg_map_ = Eigen::MatrixXd::Zero(height, width);
  }
  virtual ~SAE() = default;

  void update(const Event& e) {
    if (e.p == EVENT_POS) {
      pos_map_(e.y, e.x) = e.t;
    } else {
      neg_map_(e.y, e.x) = e.t;
    }
  }

  void reset() {
    pos_map_.setZero();
    neg_map_.setZero();
  }

  cv::Mat convertToGray(double span, bool polarity = true);
  cv::Mat convertToRGB(double span, bool white_background = false);

  inline const size_t getWidth() const { return width_; }
  inline const size_t getHeight() const { return height_; }
  inline const Eigen::MatrixXd& pos() const { return pos_map_; }
  inline const Eigen::MatrixXd& neg() const { return neg_map_; }
  inline double getValue(int x, int y) const { return std::max(pos_map_(y, x), neg_map_(y, x)); };
  inline bool getPolarity(int x, int y) const {
    return (pos_map_(y, x) > neg_map_(y, x)) ? true : false;
  }
  inline double getPosValue(int x, int y) const { return pos_map_(y, x); };
  inline double getNegValue(int x, int y) const { return neg_map_(y, x); };
  inline double getLatestTime() const { return std::max(pos_map_.maxCoeff(), neg_map_.maxCoeff()); }

 protected:
  size_t width_, height_;
  Eigen::MatrixXd pos_map_, neg_map_;
};

}  // namespace evlc