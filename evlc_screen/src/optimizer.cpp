#include "evlc_screen/optimizer.hpp"

#include "evlc_common/utils.hpp"

namespace evlc::screen {

void Optimizer::initialize(const Eigen::Matrix3d& K,
                           const std::array<EdgeTracker::Ptr, 8>& edge_trackers) {
  K_ = K;
  edge_trackers_ = edge_trackers;
  options_.max_num_iterations = 10;
  options_.linear_solver_type = ceres::DENSE_QR;
  options_.minimizer_progress_to_stdout = false;
}

Eigen::Isometry3d Optimizer::optimize(const Eigen::Isometry3d& T_init) {
  std::array<double, 4> q;
  std::array<double, 3> t;
  Eigen::Quaterniond quaternion(T_init.rotation());
  q[0] = quaternion.w();
  q[1] = quaternion.x();
  q[2] = quaternion.y();
  q[3] = quaternion.z();
  t[0] = T_init.translation()[0];
  t[1] = T_init.translation()[1];
  t[2] = T_init.translation()[2];

  // Build optimization problem
  ceres::Problem problem;
  problem.AddParameterBlock(q.data(), 4, new ceres::QuaternionManifold());
  problem.AddParameterBlock(t.data(), 3);

  // Add residual for each edge
  for (const auto& edge : edge_trackers_) {
    auto p_begin = edge->getWorldPointBegin();
    auto p_end = edge->getWorldPointEnd();
    for (const auto& e : edge->getEvents()) {
      problem.AddResidualBlock(AlignmentError::create(K_, p_begin, p_end, e),
                               new ceres::HuberLoss(0.1), q.data(), t.data());
    }
  }
  ceres::Solve(options_, &problem, &summary_);

  Eigen::Isometry3d T;
  quaternion.w() = q[0];
  quaternion.x() = q[1];
  quaternion.y() = q[2];
  quaternion.z() = q[3];
  Eigen::Vector3d translation(t[0], t[1], t[2]);
  T.setIdentity();
  T.prerotate(quaternion.toRotationMatrix());
  T.pretranslate(translation);
  return T;
}

template <typename T>
auto AlignmentError::operator()(const T* q, const T* t, T* residual) const -> bool {
  // Transform world frame into camera frame
  T P_world[3], Q_world[3], P_camera[3], Q_camera[3];
  P_world[0] = T(world_point_begin_.x());
  P_world[1] = T(world_point_begin_.y());
  P_world[2] = T(world_point_begin_.z());
  Q_world[0] = T(world_point_end_.x());
  Q_world[1] = T(world_point_end_.y());
  Q_world[2] = T(world_point_end_.z());
  ceres::QuaternionRotatePoint(q, P_world, P_camera);
  ceres::QuaternionRotatePoint(q, Q_world, Q_camera);
  P_camera[0] += t[0];
  P_camera[1] += t[1];
  P_camera[2] += t[2];
  Q_camera[0] += t[0];
  Q_camera[1] += t[1];
  Q_camera[2] += t[2];
  // Project two corner points onto 2D image frame
  T u1 = K_(0, 0) * P_camera[0] / P_camera[2] + K_(0, 2);
  T v1 = K_(1, 1) * P_camera[1] / P_camera[2] + K_(1, 2);
  T u2 = K_(0, 0) * Q_camera[0] / Q_camera[2] + K_(0, 2);
  T v2 = K_(1, 1) * Q_camera[1] / Q_camera[2] + K_(1, 2);

  T x = T(e_.x);
  T y = T(e_.y);

  // The same as what Balance::project does
  T du = u2 - u1;
  T dv = v2 - v1;
  T w = ((x - u1) * du + (y - v1) * dv) / (du * du + dv * dv);
  w = ceres::fmax(T(0.0), ceres::fmin(T(1.0), w));
  T u = u1 + w * du;
  T v = v1 + w * dv;
  residual[0] = ceres::sqrt((x - u) * (x - u) + (y - v) * (y - v));
  return true;
}

}  // namespace evlc::screen