#pragma once

#include "Eigen/Dense"

namespace dynamical {
namespace trajectory {

template <int state_dim, int input_dim, typename Scalar>
class Trajectory {
 public:
  virtual ~Trajectory() = default;

  virtual Eigen::Matrix<Scalar, state_dim, 1> GetDesiredState(double time) = 0;

  virtual Eigen::Matrix<Scalar, input_dim, 1> GetFeedforward(double time) {
    return Eigen::Matrix<Scalar, input_dim, 1>::Zero();
  }
};

}  // namespace trajectory
}  // namespace dynamical
