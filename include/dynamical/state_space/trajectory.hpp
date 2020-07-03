#pragma once

#include "dynamical/utils/exception.hpp"

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

// CAUTION: THE CURRENT IMPLEMENTATION FOR MIN ENERGY CONTROL HAS LARGELY
// UNKNOWN BEHAVIOR! SEE trajectory_test.cpp FOR MORE INFORMATION!
//
// Also note that I'm not entirely sure if this is a valid method for
// multiple-input systems.
//
// TODO: revisit this
//
//
// Uses the minimum norm solution to create a zero-indexed stepwise trajectory.
// https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/12a.pdf
//
// The resultant input sequence encoded in "inverted_input_sequence_" is of the
// form:
// [[ u[num_steps_ - 1] ],
//  [ u[num_steps_ - 2] ],
//  .
//  .
//  [ u[1] ],
//  [ u[0] ]]
// where each u is a **column** vector of dimension (row=input_dim, col=1).
//
// Note that the entire "inverted_input_sequence_" is a column vector regardless
// of the input dimension.
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
      throw dynamical_error(
          "min energy trajectory: controllability matrix not invertible.");
    }

    inverted_input_sequence_ =
        controllability_matrix_.transpose() * C_CT.inverse() * target_state_;
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

    // input sequence is inverted: the first step (step 0) corresponds to the
    // last row (num_steps_ - 1) of the matrix.
    step = num_steps_ - 1 - step;
    return inverted_input_sequence_.block(step * input_dim, 0, input_dim, 1);
  }

  const ControllabilityMatrixType controllability_matrix_;
  const int num_steps_;
  const x_VectorType target_state_;

 private:
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> inverted_input_sequence_;
};

}  // namespace trajectory
}  // namespace dynamical
