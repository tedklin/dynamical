// Uses the minimum norm solution to create a zero-indexed stepwise trajectory.
// https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/12a.pdf
//
// Let C be the controllability matrix of the system (not the same C as the
// measurement matrix).
// The result of C^T * (C * C^T)^-1 * x_target is:
// [[ u[num_steps_ - 1]` ],
//  [ u[num_steps_ - 2]` ],
//  .
//  .
//  [ u[1]` ],
//  [ u[0]` ]]
// where each u` is itself a column vector {rows=input_dim, cols=1}.
// Each u` is the reverse of the actual input u we want.
//
//
// I'm not entirely sure about the reliability of this. The accuracy is
// not always satisfactory, see min_energy_test.cpp for more information.

#pragma once

#include "dynamical/trajectory/trajectory.hpp"

#include "dynamical/utils/exception.hpp"

#include "Eigen/Dense"

namespace dynamical {
namespace trajectory {

template <int state_dim, int input_dim, typename Scalar = double>
class MinEnergy : public Trajectory<state_dim, input_dim, Scalar> {
 public:
  using typename Trajectory<state_dim, input_dim, Scalar>::x_VectorType;
  using typename Trajectory<state_dim, input_dim, Scalar>::u_VectorType;

  using ControllabilityMatrixType =
      Eigen::Matrix<Scalar, state_dim, Eigen::Dynamic>;

  MinEnergy(const ControllabilityMatrixType& controllability_matrix,
            int num_steps, const x_VectorType& target_state)
      : controllability_matrix_(controllability_matrix),
        num_steps_(num_steps),
        target_state_(target_state) {
    Eigen::Matrix<Scalar, state_dim, state_dim> C_CT =
        controllability_matrix_ * controllability_matrix_.transpose();

    Eigen::FullPivLU<decltype(C_CT)> lu(C_CT);
    if (!lu.isInvertible()) {
      throw dynamical_error("min energy trajectory: (C * C^T) not invertible.");
    }

    input_sequence_ =
        controllability_matrix_.transpose() * C_CT.inverse() * target_state_;
    input_sequence_.reverseInPlace();
  }

  x_VectorType GetDesiredState(int step) {
    // TODO: after fixing pure open-loop, figure this out?
    return x_VectorType::Zero();
  }

  u_VectorType GetFeedforward(int step) {
    if (step >= num_steps_) {
      throw dynamical_error(
          "min energy trajectory: tried to access step out of range!");
    }
    return input_sequence_.block(step * input_dim, 0, input_dim, 1);
  }

  const ControllabilityMatrixType controllability_matrix_;
  const int num_steps_;
  const x_VectorType target_state_;

 private:
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> input_sequence_;
};

}  // namespace trajectory
}  // namespace dynamical
