#pragma once

#include "Eigen/Dense"

namespace dynamical {
namespace trajectory {

template <int state_dim, int input_dim, typename Scalar = double>
class Trajectory {
 public:
  using x_VectorType = Eigen::Matrix<Scalar, state_dim, 1>;
  using u_VectorType = Eigen::Matrix<Scalar, input_dim, 1>;

  virtual ~Trajectory() = default;

  virtual x_VectorType GetDesiredState(int step) = 0;

  virtual u_VectorType GetFeedforward(int step) { return u_VectorType::Zero(); }
};

}  // namespace trajectory
}  // namespace dynamical
