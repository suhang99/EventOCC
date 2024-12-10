#pragma once

#include "evlc_common/event.hpp"
#include "evlc_common/types.hpp"

namespace evlc::screen {

class EdgeTracker {
 public:
  using Ptr = std::shared_ptr<EdgeTracker>;

  class Slot {
   public:
    Event event;
    double distance;
    bool valid;
  };

  /**
   * @brief
   */
  EdgeTracker(Eigen::Vector3d pw1, Eigen::Vector3d pw2, Eigen::Vector2d pc1, Eigen::Vector2d pc2);

  /**
   * @brief Set distance threshod
   * @param thres distance threshold
   * @param alpha parameter that control the number of events per edge
   */
  void setThreshold(double thres, double alpha) {
    thres_ = thres;
    half_thres_ = thres / 2;
    alpha_ = alpha;
    capacity_ = static_cast<size_t>(alpha * (image_point_begin_ - image_point_end_).norm());
    resetCache();
  }

  /**
   * @brief Update one event for edge tracker
   * @param event Incoming event
   */
  void updateEvent(const Event& event);

  /**
   * @brief Update edge position
   * @param begin Beginning image point
   * @param end Ending image point
   */
  void updateEdge(const Eigen::Vector2d& begin, const Eigen::Vector2d& end);

  /**
   * @brief Check whether edge position needs update
   * @return true if it needs update, false otherwise
   */
  inline bool needUpdate() { return meanDistance() > half_thres_ ? true : false; }

  /**
   * @brief Begining world point getter
   * @return beginning world point
   */
  const Eigen::Vector3d& getWorldPointBegin() const { return world_point_begin_; }

  /**
   * @brief Ending world point getter
   * @return ending world point
   */
  const Eigen::Vector3d& getWorldPointEnd() const { return world_point_end_; }

  /**
   * @brief Beginning image point getter
   * @return beginning image point
   */
  const Eigen::Vector2d& getImagePointBegin() const { return image_point_begin_; }

  /**
   * @brief Ending image point getter
   * @return ending image point
   */
  const Eigen::Vector2d& getImagePointEnd() const { return image_point_end_; }

  /**
   * @brief Get all events
   * @return events
   */
  std::vector<Event> getEvents();

  /**
   * @brief return the mean distance of events to edge
   * @return mean distance
   */
  double meanDistance();

 private:
  /**
   * @brief add event into cache
   * @param event event to add
   * @param distance event to edge distance
   * @param weight closeness for the begining point
   */
  inline void addEvent(const Event& event, double distance, double weight) {
    size_t idx = floor(weight * capacity_);
    idx = (idx == capacity_) ? capacity_ - 1 : idx;
    if (cache_[idx].valid and cache_[idx].event.t >= event.t) {
      return;
    } else {
      cache_[idx].event = event;
      cache_[idx].distance = distance;
      cache_[idx].valid = true;
    }
  }

  /**
   * @brief project event onto edge
   * @param event Event to project
   * @param distance event-edge distance
   * @param projection projected point
   * @param weight closeness to begining point
   */
  void project(const Event& event, double& distance, Eigen::Vector2d& projection, double& weight);

  /**
   * @brief Reset cache
   */
  void resetCache();

  Eigen::Vector3d world_point_begin_, world_point_end_;
  Eigen::Vector2d image_point_begin_, image_point_end_;
  double thres_, half_thres_, alpha_;
  size_t capacity_;
  std::vector<Slot> cache_;
};
}  // namespace evlc::screen