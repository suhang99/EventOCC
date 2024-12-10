#pragma once

#include <deque>

#include "evlc_common/types.hpp"

namespace evlc {

/**
 * @brief Template class for buffer to store data with certain time window
 */
template <typename T>
class TemporalBuffer {
 public:
  using Ptr = std::shared_ptr<TemporalBuffer>;

  TemporalBuffer() : span_(1) {}

  /**
   * @brief Setter of timespan
   * @param span timespan
   */
  void setTimespan(double span) { span_ = span; }

  /**
   * @brief Getter of timespan
   * @return timespan
   */
  double getTimespan() const { return span_; }

  /**
   * @brief Push one element into buffer
   * @param element element
   */
  inline void push(const T& element) { elements_.emplace_back(std::move(element)); }

  /**
   * @brief Push a vector of elements
   * @param elements a vector of elements
   */
  void push(const std::vector<T>& elements) {
    elements_.insert(elements_.end(), elements.begin(), elements.end());
  }

  /**
   * @brief Remove outdated elements
   */
  void clean() {
    if (elements_.empty()) {
      return;
    }
    double t = elements_.back().t - span_;
    while (!elements_.empty() and elements_.front().t < t) {
      elements_.pop_front();
    }
  }

  /**
   * @brief Clear all elements
   */
  void clear() { elements_.clear(); }

  /**
   * @brief Get elements
   * @return elements
   */
  const std::deque<T>& data() const { return elements_; }

  /**
   * @brief Get the number of elements
   * @return size
   */
  size_t size() const { return elements_.size(); }

 protected:
  std::deque<T> elements_;
  double span_;
};

}  // namespace evlc