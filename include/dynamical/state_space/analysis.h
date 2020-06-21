#pragma once

#include "dynamical/state_space/plant.h"

namespace dynamical {
namespace analysis {

template <int state_dim, int input_dim, int output_dim, typename Scalar>
auto get_controllability_matrix(
    const dynamical::Plant<state_dim, input_dim, output_dim, Scalar>& plant,
    int num_steps_allowed = state_dim) {
  Eigen::Matrix<Scalar, state_dim, Eigen::Dynamic> controllability_matrix(
      state_dim, (num_steps_allowed * input_dim));
  Eigen::Matrix<Scalar, state_dim, input_dim> step_block = plant.B_;

  for (int step = 0; step != num_steps_allowed; ++step) {
    for (int block_index = 0; block_index != input_dim; ++block_index) {
      int controllability_col = (step * input_dim) + block_index;
      controllability_matrix.col(controllability_col) =
          step_block.col(block_index);
    }
    step_block = plant.A_ * step_block;
  }

  return controllability_matrix;
}

// TODO: check the numerical robustness of this?
template <int state_dim, int input_dim, int output_dim, typename Scalar>
bool is_controllable(Plant<state_dim, input_dim, output_dim, Scalar>& plant) {
  auto controllability_matrix = get_controllability_matrix(plant);
  Eigen::ColPivHouseholderQR<decltype(controllability_matrix)> qr_decomp(
      controllability_matrix);
  auto rank = qr_decomp.rank();
  return rank >= state_dim;
}

// TODO: get_observability_matrix and check_observability?

// discretization by diagonalization
// https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf
template <int state_dim, int input_dim, int output_dim, typename Scalar>
dynamical::DiscretePlant<state_dim, input_dim, output_dim, Scalar> discretize(
    const dynamical::ContinuousPlant<state_dim, input_dim, output_dim, Scalar>&
        continuous_plant,
    double dt) {
  Eigen::EigenSolver<decltype(continuous_plant.A_)> eigensolver(
      continuous_plant.A_);
  auto A_eigenbasis = eigensolver.eigenvectors();

  decltype(A_eigenbasis) A_eigenbasis_inverse;
  bool A_eigenbasis_invertible;
  A_eigenbasis.computeInverseWithCheck(A_eigenbasis_inverse,
                                       A_eigenbasis_invertible);
  if (!A_eigenbasis_invertible) {
    throw std::runtime_error(
        "(dynamical) Discretization error: eigenbasis not invertible.");
  }

  auto A_eigenvalues = eigensolver.eigenvalues();
  decltype(A_eigenbasis) homogeneous_sol = decltype(A_eigenbasis)::Zero();
  decltype(A_eigenbasis) particular_sol = decltype(A_eigenbasis)::Zero();
  for (unsigned index = 0; index != A_eigenvalues.rows(); ++index) {
    auto eigenvalue = A_eigenvalues(index, 0);
    homogeneous_sol(index, index) = std::exp(eigenvalue * dt);
    if (eigenvalue == 0) {
      particular_sol(index, index) = dt;
    } else {
      particular_sol(index, index) =
          (std::exp(eigenvalue * dt) - 1) / eigenvalue;
    }
  }

  decltype(continuous_plant.A_) A_discretized =
      A_eigenbasis * homogeneous_sol * A_eigenbasis_inverse;
  decltype(continuous_plant.B_) B_discretized = A_eigenbasis * particular_sol *
                                                A_eigenbasis_inverse *
                                                continuous_plant.B_;

  dynamical::DiscretePlant<state_dim, input_dim, output_dim, Scalar>
      discretized_plant(continuous_plant.GetX(), A_discretized, B_discretized,
                        continuous_plant.C_, continuous_plant.D_);

  return discretized_plant;
}

}  // namespace analysis
}  // namespace dynamical
