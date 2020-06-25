#pragma once

#include "dynamical/state_space/controller.hpp"
#include "dynamical/state_space/plant.hpp"

#include <cmath>
#include <iostream>  // TODO: take iostream out after debugging
#include <stdexcept>

namespace dynamical {
namespace analysis {

template <int state_dim, int input_dim, int output_dim, typename Scalar>
Eigen::Matrix<Scalar, state_dim, Eigen::Dynamic> get_controllability_matrix(
    const Plant<state_dim, input_dim, output_dim, Scalar>& plant,
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
bool is_controllable(
    const Plant<state_dim, input_dim, output_dim, Scalar>& plant) {
  Eigen::Matrix<Scalar, state_dim, Eigen::Dynamic> controllability_matrix =
      get_controllability_matrix(plant);
  Eigen::ColPivHouseholderQR<decltype(controllability_matrix)> qr_decomp(
      controllability_matrix);
  auto rank = qr_decomp.rank();
  return rank >= state_dim;
}

// TODO: get_observability_matrix and check_observability?

template <typename Scalar, int state_dim>
bool stability_helper(
    const Eigen::Matrix<Scalar, state_dim, state_dim>& dynamics_matrix,
    bool is_discrete) {
  Eigen::EigenSolver<Eigen::Matrix<Scalar, state_dim, state_dim>> eigensolver(
      dynamics_matrix);
  Eigen::Matrix<std::complex<double>, state_dim, 1> eigenvalues =
      eigensolver.eigenvalues();

  for (unsigned row = 0; row < eigenvalues.rows(); ++row) {
    std::cout << eigenvalues(row, 0) << '\n';
    if (is_discrete && std::abs(eigenvalues(row, 0)) >= 1) {
      std::cout
          << "(dynamical) stability: triggered discrete unstable condition\n";
      return false;
    }
    if (!is_discrete && eigenvalues(row, 0).real() > 0) {
      std::cout
          << "(dynamical) stability: triggered continuous unstable condition\n";
      return false;
    }
  }
  return true;
}

// TODO: consider type identification (tricky with templates, use std::issame)
template <int state_dim, int input_dim, int output_dim, typename Scalar>
bool is_stable(
    const DiscretePlant<state_dim, input_dim, output_dim, Scalar>&
        discrete_plant,
    const Feedback<state_dim, input_dim, output_dim, Scalar>& feedback) {
  std::cout << "running discrete stability check...\n\n";

  Eigen::Matrix<Scalar, state_dim, state_dim> dynamics_matrix =
      discrete_plant.A_ + discrete_plant.B_ * feedback.GetK();

  return stability_helper(dynamics_matrix, true);
}

template <int state_dim, int input_dim, int output_dim, typename Scalar>
bool is_stable(
    const ContinuousPlant<state_dim, input_dim, output_dim, Scalar>&
        continuous_plant,
    const Feedback<state_dim, input_dim, output_dim, Scalar>& feedback) {
  std::cout << "running continuous stability check...\n\n";

  Eigen::Matrix<Scalar, state_dim, state_dim> dynamics_matrix =
      continuous_plant.A_ + continuous_plant.B_ * feedback.GetK();

  return stability_helper(dynamics_matrix, false);
}

template <int state_dim, int input_dim, int output_dim, typename Scalar>
bool is_stable(const DiscretePlant<state_dim, input_dim, output_dim, Scalar>&
                   discrete_plant) {
  std::cout << "running discrete stability check...\n\n";

  return stability_helper(discrete_plant.A_, true);
}

template <int state_dim, int input_dim, int output_dim, typename Scalar>
bool is_stable(const ContinuousPlant<state_dim, input_dim, output_dim, Scalar>&
                   continuous_plant) {
  std::cout << "running continuous stability check...\n\n";

  return stability_helper(continuous_plant.A_, false);
}

// discretization by diagonalization
// the returned discrete model has complex Scalar type std::complex<double>
// https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf
template <int state_dim, int input_dim, int output_dim, typename Scalar>
DiscretePlant<state_dim, input_dim, output_dim, std::complex<double>>
discretize(const ContinuousPlant<state_dim, input_dim, output_dim, Scalar>&
               continuous_plant,
           double dt) {
  Eigen::EigenSolver<Eigen::Matrix<Scalar, state_dim, state_dim>> eigensolver(
      continuous_plant.A_);
  Eigen::Matrix<std::complex<double>, state_dim, state_dim> A_eigenbasis =
      eigensolver.eigenvectors();

  decltype(A_eigenbasis) A_eigenbasis_inverse;
  // // eigen-3.3.7 (more efficient?)
  // bool A_eigenbasis_invertible;
  // A_eigenbasis.computeInverseWithCheck(A_eigenbasis_inverse,
  //                                      A_eigenbasis_invertible);
  // if (!A_eigenbasis_invertible) {
  //   throw std::runtime_error(
  //       "(dynamical) Discretization error: eigenbasis not invertible.");
  // }

  // eigen-3.3.4 (default installation with apt)
  Eigen::FullPivLU<decltype(A_eigenbasis)> lu(A_eigenbasis);
  if (!lu.isInvertible()) {
    throw std::runtime_error(
        "(dynamical) Discretization error: eigenbasis not invertible.");
  }
  A_eigenbasis_inverse = A_eigenbasis.inverse();

  Eigen::Matrix<std::complex<double>, state_dim, 1> A_eigenvalues =
      eigensolver.eigenvalues();
  decltype(A_eigenbasis) homogeneous_sol = decltype(A_eigenbasis)::Zero();
  decltype(A_eigenbasis) particular_sol = decltype(A_eigenbasis)::Zero();
  for (unsigned index = 0; index != A_eigenvalues.rows(); ++index) {
    std::complex<double> eigenvalue = A_eigenvalues(index, 0);
    homogeneous_sol(index, index) = std::exp(eigenvalue * dt);
    if (std::abs(eigenvalue) == 0) {
      particular_sol(index, index) = dt;
    } else {
      particular_sol(index, index) =
          (std::exp(eigenvalue * dt) - 1.0) / eigenvalue;
    }
  }

  Eigen::Matrix<std::complex<double>, state_dim, state_dim> A_discretized =
      A_eigenbasis * homogeneous_sol * A_eigenbasis_inverse;
  Eigen::Matrix<std::complex<double>, state_dim, input_dim> B_discretized =
      A_eigenbasis * particular_sol * A_eigenbasis_inverse *
      continuous_plant.B_;

  DiscretePlant<state_dim, input_dim, output_dim, std::complex<double>>
      discretized_plant(continuous_plant.GetX(), A_discretized, B_discretized,
                        continuous_plant.C_, continuous_plant.D_);

  // std::cout << A_eigenbasis << "\n\n";
  // std::cout << A_eigenbasis_inverse << "\n\n";
  // std::cout << A_eigenvalues << "\n\n";

  return discretized_plant;
}

}  // namespace analysis
}  // namespace dynamical
