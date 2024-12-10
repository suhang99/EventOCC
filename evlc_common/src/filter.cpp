#include "evlc_common/filter.hpp"

namespace evlc {

RefractoryFilter::RefractoryFilter(SAE::Ptr sae, double same_pol_thres, double oppo_pol_thres)
    : sae_(sae), same_pol_thres_(same_pol_thres), oppo_pol_thres_(oppo_pol_thres) {}

bool RefractoryFilter::run(const Event& e) const {
  if (e.p == EVENT_POS) {
    if (e.t - sae_->getPosValue(e.x, e.y) < same_pol_thres_ or
        e.t - sae_->getNegValue(e.x, e.y) < oppo_pol_thres_) {
      return false;
    }
  } else {
    if (e.t - sae_->getNegValue(e.x, e.y) < same_pol_thres_ or
        e.t - sae_->getPosValue(e.x, e.y) < oppo_pol_thres_) {
      return false;
    }
  }
  return true;
}

NeighborFilter::NeighborFilter(SAE::Ptr sae, size_t min_neighbors, size_t radius, double max_age)
    : sae_(sae),
      min_neighbors_(min_neighbors),
      radius_(radius),
      max_age_(max_age),
      height_(sae->getHeight()),
      width_(sae->getWidth()),
      diameter_(2 * radius + 1) {}

bool NeighborFilter::run(const Event& e) const {
  // Check whether the kernel is out of bound
  int start_x = std::max(0, static_cast<int>(e.x - radius_));
  int start_y = std::max(0, static_cast<int>(e.y - radius_));
  int end_x = std::min(width_ - 1, static_cast<int>(e.x + radius_));
  int end_y = std::min(height_ - 1, static_cast<int>(e.y + radius_));

  int count = 0;
  auto thres = e.t - max_age_;
  for (int y = start_y; y < end_y; ++y) {
    for (int x = start_x; x < end_x; ++x) {
      if (sae_->pos()(y, x) > thres or sae_->neg()(y, x)) {
        ++count;
      }
    }
  }

  if (count > min_neighbors_) {
    return true;
  } else {
    return false;
  }
}

}  // namespace evlc