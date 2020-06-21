#pragma once

#include <cmath>
#include <stdexcept>

#include "Eigen/Dense"
#include "dynamical/math/integration.h"

namespace dynamical {

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class Plant {
 public:
  using A_matrix_type = Eigen::Matrix<Scalar, state_dim, state_dim>;
  using B_matrix_type = Eigen::Matrix<Scalar, state_dim, input_dim>;
  using C_matrix_type = Eigen::Matrix<Scalar, output_dim, state_dim>;
  using D_matrix_type = Eigen::Matrix<Scalar, output_dim, input_dim>;
  using x_vector_type = Eigen::Matrix<Scalar, state_dim, 1>;
  using y_vector_type = Eigen::Matrix<Scalar, output_dim, 1>;
  using u_vector_type = Eigen::Matrix<Scalar, input_dim, 1>;

  Plant() = delete;

  Plant(const x_vector_type& x_initial, const A_matrix_type& A,
        const B_matrix_type& B,
        const C_matrix_type& C = C_matrix_type::Identity(),
        const D_matrix_type& D = D_matrix_type::Zero())
      : A_(A), B_(B), C_(C), D_(D), x_initial_(x_initial), x_(x_initial) {}

  virtual ~Plant() = default;

  virtual void Update(const u_vector_type& u) = 0;

  void SetXInitial(const x_vector_type& initial_x) { x_initial_ = initial_x; }

  void ResetToXInitial() { x_ = x_initial_; }

  const x_vector_type& GetX() const { return x_; }

  const y_vector_type& GetY() const { return y_; }

  const A_matrix_type A_;
  const B_matrix_type B_;
  const C_matrix_type C_;
  const D_matrix_type D_;

 protected:
  x_vector_type x_initial_ = x_vector_type::Zero();
  x_vector_type x_ = x_vector_type::Zero();
  y_vector_type y_ = y_vector_type::Zero();
};

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class DiscretePlant : public Plant<state_dim, input_dim, output_dim, Scalar> {
 public:
  using Plant<state_dim, input_dim, output_dim, Scalar>::Plant;

  using typename Plant<state_dim, input_dim, output_dim, Scalar>::u_vector_type;

  void Update(const u_vector_type& u) override {
    this->x_ = this->A_ * this->x_ + this->B_ * u;
    this->y_ = this->C_ * this->x_ + this->D_ * u;
  }
};

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class ContinuousPlant : public Plant<state_dim, input_dim, output_dim, Scalar> {
 public:
  using Plant<state_dim, input_dim, output_dim, Scalar>::Plant;

  using typename Plant<state_dim, input_dim, output_dim, Scalar>::u_vector_type;
  using typename Plant<state_dim, input_dim, output_dim, Scalar>::x_vector_type;

  void Update(const u_vector_type& u) override {}

  void UpdateSim(const u_vector_type& u, double dt) {
    // TODO: test this against discretized model?
    // https://en.cppreference.com/w/cpp/language/lambda#Lambda_capture
    this->x_ = numerical::integration::rk4(
        [&](const x_vector_type& x) -> x_vector_type {
          return this->A_ * x + this->B_ * u;
        },
        this->x_, dt);

    this->y_ = this->C_ * this->x_ + this->D_ * u;
  }
};

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
  Eigen::EigenSolver<A_matrix_type> eigensolver(continuous_plant.A_);
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
  decltype(A_eigenbasis) homogeneous_sol = A_matrix_type::Zero();
  decltype(A_eigenbasis) particular_sol = A_matrix_type::Zero();
  for (unsigned row = 0; row != A_eigenvalues_.rows(); ++row) {
    auto eigenvalue = A_eigenvalues(row, 0);
    homogeneous_sol(row, row) = std::exp(eigenvalue * dt);
    if (eigenvalue == 0) {
      particular_sol(row, row) = dt;
    } else {
      particular_sol(row, row) = (std::exp(eigenvalue * dt) - 1) / eigenvalue;
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
