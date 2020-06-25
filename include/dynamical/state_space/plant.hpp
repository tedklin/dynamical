#pragma once

#include "dynamical/math/integration.hpp"

#include <cmath>

#include "Eigen/Dense"

namespace dynamical {

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class Plant {
 public:
  using A_MatrixType = Eigen::Matrix<Scalar, state_dim, state_dim>;
  using B_MatrixType = Eigen::Matrix<Scalar, state_dim, input_dim>;
  using C_MatrixType = Eigen::Matrix<Scalar, output_dim, state_dim>;
  using D_MatrixType = Eigen::Matrix<Scalar, output_dim, input_dim>;
  using x_VectorType = Eigen::Matrix<Scalar, state_dim, 1>;
  using y_VectorType = Eigen::Matrix<Scalar, output_dim, 1>;
  using u_VectorType = Eigen::Matrix<Scalar, input_dim, 1>;

  Plant() = delete;

  Plant(const x_VectorType& x_initial, const A_MatrixType& A,
        const B_MatrixType& B, const C_MatrixType& C = C_MatrixType::Identity(),
        const D_MatrixType& D = D_MatrixType::Zero())
      : A_(A), B_(B), C_(C), D_(D), x_initial_(x_initial), x_(x_initial) {}

  virtual ~Plant() = default;

  virtual void Update(const u_VectorType& u) = 0;

  void ResetToXInitial() { x_ = x_initial_; }

  void SetXInitial(const x_VectorType& initial_x) { x_initial_ = initial_x; }

  const x_VectorType& GetXInitial() const { return x_initial_; }

  const x_VectorType& GetX() const { return x_; }

  const y_VectorType& GetY() const { return y_; }

  const A_MatrixType A_;
  const B_MatrixType B_;
  const C_MatrixType C_;
  const D_MatrixType D_;

 protected:
  x_VectorType x_initial_;
  x_VectorType x_;
  y_VectorType y_ = y_VectorType::Zero();
};

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class DiscretePlant : public Plant<state_dim, input_dim, output_dim, Scalar> {
 public:
  using Plant<state_dim, input_dim, output_dim, Scalar>::Plant;

  using typename Plant<state_dim, input_dim, output_dim, Scalar>::u_VectorType;

  void Update(const u_VectorType& u) override {
    this->x_ = this->A_ * this->x_ + this->B_ * u;
    this->y_ = this->C_ * this->x_ + this->D_ * u;
  }
};

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class ContinuousPlant : public Plant<state_dim, input_dim, output_dim, Scalar> {
 public:
  using Plant<state_dim, input_dim, output_dim, Scalar>::Plant;

  using typename Plant<state_dim, input_dim, output_dim, Scalar>::u_VectorType;
  using typename Plant<state_dim, input_dim, output_dim, Scalar>::x_VectorType;

  // TODO: implement this real-time?
  void Update(const u_VectorType& u) override {}

  void UpdateSim(const u_VectorType& u, double dt) {
    // https://en.cppreference.com/w/cpp/language/lambda#Lambda_capture
    this->x_ = numerical::integration::rk4(
        [&](const x_VectorType& x) -> x_VectorType {
          return this->A_ * x + this->B_ * u;
        },
        this->x_, dt);

    this->y_ = this->C_ * this->x_ + this->D_ * u;
  }
};

}  // namespace dynamical
