#include "evlc_screen/edge_tracker.hpp"

#include <algorithm>

namespace evlc::screen {

EdgeTracker::EdgeTracker(Eigen::Vector3d pw1, Eigen::Vector3d pw2, Eigen::Vector2d pc1,
                         Eigen::Vector2d pc2)
    : world_point_begin_(pw1),
      world_point_end_(pw2),
      image_point_begin_(pc1),
      image_point_end_(pc2) {}

void EdgeTracker::updateEvent(const Event& event) {
  double distance;
  double weight;
  Eigen::Vector2d projection;
  project(event, distance, projection, weight);

  if (distance <= thres_) {
    addEvent(event, distance, weight);
  }
}

void EdgeTracker::updateEdge(const Eigen::Vector2d& begin, const Eigen::Vector2d& end) {
  image_point_begin_ = begin;
  image_point_end_ = end;
  capacity_ = static_cast<size_t>(alpha_ * (begin - end).norm());
  auto old_cache = cache_;
  resetCache();
  size_t idx = 0;
  size_t n = old_cache.size();
  // Rearrange old cache into new cache
  for (const auto& element : old_cache) {
    if (element.valid) {
      double distance = 0, weight = 0;
      Eigen::Vector2d projection(0, 0);
      project(element.event, distance, projection, weight);

      idx = std::floor(weight * capacity_);
      idx = (idx == capacity_) ? capacity_ - 1 : idx;
      // Select latest event
      if (cache_[idx].valid and cache_[idx].event.t >= element.event.t) {
        continue;
      }
      cache_[idx].event = element.event;
      cache_[idx].distance = distance;
      cache_[idx].valid = true;
    }
  }
}

std::vector<Event> EdgeTracker::getEvents() {
  std::vector<Event> events;
  for (const auto& c : cache_) {
    if (c.valid) {
      events.emplace_back(c.event);
    }
  }
  return events;
}

double EdgeTracker::meanDistance() {
  size_t count = 0;
  double sum = 0.0;
  for (auto& c : cache_) {
    if (c.valid) {
      ++count;
      sum += c.distance;
    }
  }
  return sum / count;
}

void EdgeTracker::project(const Event& event, double& distance, Eigen::Vector2d& projection,
                          double& weight) {
  Eigen::Vector2d pe(event.x, event.y);
  Eigen::Vector2d diff = image_point_end_ - image_point_begin_;
  double l2 = diff.squaredNorm();
  // If line is small
  if (diff.squaredNorm() < EPSILON) {
    distance = (image_point_begin_ - pe).norm();
    projection = image_point_begin_;
    weight = 0;
  } else {
    weight = std::max(0.0, std::min(1.0, (pe - image_point_begin_).dot(diff) / l2));
    projection = image_point_begin_ + weight * diff;
    distance = (pe - projection).norm();
  }
}

void EdgeTracker::resetCache() {
  cache_.resize(capacity_);
  for (auto& c : cache_) {
    c.valid = false;
  }
}

}  // namespace evlc::screen