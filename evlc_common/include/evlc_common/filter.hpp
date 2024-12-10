#pragma once

#include "evlc_common/event.hpp"
#include "evlc_common/types.hpp"

namespace evlc {

/**
 * @brief Abstract class for filter
 */
class FilterBase {
 public:
  using Ptr = std::shared_ptr<FilterBase>;

  virtual bool run(const Event& e) const = 0;
};

/**
 * @brief Refractory filter that suppress hot pixels
 */
class RefractoryFilter : public FilterBase {
 public:
  using Ptr = std::shared_ptr<RefractoryFilter>;
  RefractoryFilter(SAE::Ptr sae, double same_pol_thres = 1e-3, double oppo_pol_thres = 1e-3);

  bool run(const Event& e) const override;

 private:
  const SAE::Ptr sae_;
  double same_pol_thres_, oppo_pol_thres_;
};

/**
 * @brief Neighbor filter that suppress sparse events
 */
class NeighborFilter : public FilterBase {
 public:
  using Ptr = std::shared_ptr<NeighborFilter>;

  NeighborFilter(SAE::Ptr sae, size_t neighbor_thres = 3, size_t radius = 2, double max_age = 1e-2);

  bool run(const Event& e) const override;

 private:
  const SAE::Ptr sae_;
  int width_, height_;
  size_t min_neighbors_, radius_, diameter_;
  double max_age_;
};

}  // namespace evlc