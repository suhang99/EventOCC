#pragma once

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "evlc_common/timer.hpp"
#include "evlc_common/types.hpp"
#include "evlc_screen/edge_tracker.hpp"

namespace evlc::screen {

class Optimizer {
 public:
  using Ptr = std::shared_ptr<Optimizer>;

  Optimizer(Profiler::Ptr profiler) : profiler_(profiler){};

  /**
   * @brief Initialize optimize with camera matrix and edge trackers
   * @param K camera matrix
   * @param edge_tracker edge trackers
   */
  void initialize(const Eigen::Matrix3d& K, const std::array<EdgeTracker::Ptr, 8>& edge_trackers);

  /**
   * @brief Optimize pose by using the data in edge tracker
   * @param T_init initial pose
   * @return optimized pose
   */
  Eigen::Isometry3d optimize(const Eigen::Isometry3d& T_init);

 private:
  ceres::Solver::Options options_;
  ceres::Solver::Summary summary_;
  std::array<EdgeTracker::Ptr, 8> edge_trackers_;
  Eigen::Matrix3d K_;
  Profiler::Ptr profiler_;
};

struct AlignmentError {
  AlignmentError(Eigen::Matrix3d K, Eigen::Vector3d begin, Eigen::Vector3d end, Event e)
      : K_(K), world_point_begin_(begin), world_point_end_(end), e_(e) {}

  /**
   * @brief Implement point-to-line loss function
   * @param q quaternion
   * @param t translation
   * @param residual residual
   */
  template <typename T>
  bool operator()(const T* q, const T* t, T* residual) const;

  static ceres::CostFunction* create(Eigen::Matrix3d K, Eigen::Vector3d p_begin,
                                     Eigen::Vector3d p_end, Event e) {
    return new ceres::AutoDiffCostFunction<AlignmentError, 1, 4, 3>(
        new AlignmentError(K, p_begin, p_end, e));
  }

 private:
  const Eigen::Matrix3d K_;
  const Eigen::Vector3d world_point_begin_, world_point_end_;
  const Event e_;
};

}  // namespace evlc::screen