#pragma once

#include "dynamical/math/integration.h"

#include <cmath>
#include <stdexcept>

#include "Eigen/Dense"

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

}  // namespace dynamical
