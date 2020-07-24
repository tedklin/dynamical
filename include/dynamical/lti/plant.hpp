// Simulation for plants, with support for artificial Gaussian noise.

#pragma once

#include "dynamical/math/integral.hpp"

#include <cmath>
#include <memory>
#include <random>

#include "Eigen/Dense"

namespace dynamical {
namespace lti {
namespace sim {

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

  virtual ~Plant() = default;

  Plant() = delete;

  Plant(const x_VectorType& x_initial, const A_MatrixType& A,
        const B_MatrixType& B, const C_MatrixType& C = C_MatrixType::Identity(),
        const D_MatrixType& D = D_MatrixType::Zero())
      : A_(A), B_(B), C_(C), D_(D), x_initial_(x_initial), x_(x_initial) {
    std::random_device rd;
    random_generator_ = std::make_shared<std::mt19937>(rd());
  }

  virtual void Update(const u_VectorType& u) = 0;

  const x_VectorType& GetX() const { return x_; }

  const y_VectorType& GetY() const { return y_; }

  void ResetToXInitial() { x_ = x_initial_; }

  void SetXInitial(const x_VectorType& initial_x) { x_initial_ = initial_x; }

  const x_VectorType& GetXInitial() const { return x_initial_; }

  void EnableOutputNoise(double noise_stddev) {
    output_noise_present_ = true;
    output_noise_distribution_ =
        std::make_shared<std::normal_distribution<>>(0, noise_stddev);
  }

  void DisableNoise() { output_noise_present_ = false; }

  const A_MatrixType A_;
  const B_MatrixType B_;
  const C_MatrixType C_;
  const D_MatrixType D_;

 protected:
  x_VectorType x_initial_;
  x_VectorType x_;
  y_VectorType y_ = y_VectorType::Zero();

  bool output_noise_present_ = false;
  double noise_stddev_ = 0;
  std::shared_ptr<std::mt19937> random_generator_;
  std::shared_ptr<std::normal_distribution<>> output_noise_distribution_;

  // TODO: process noise
  // TODO: different noise distributions for each scalar variable (pass in a
  // vector for stddev)

  y_VectorType GenerateOutputNoise() {
    return y_VectorType::NullaryExpr(
        [&]() { return (*output_noise_distribution_)(*random_generator_); });
  }
};

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class DiscretePlant : public Plant<state_dim, input_dim, output_dim, Scalar> {
 public:
  using Plant<state_dim, input_dim, output_dim, Scalar>::Plant;

  using typename Plant<state_dim, input_dim, output_dim, Scalar>::y_VectorType;
  using typename Plant<state_dim, input_dim, output_dim, Scalar>::u_VectorType;

  void Update(const u_VectorType& u) override {
    this->y_ = this->C_ * this->x_ + this->D_ * u;
    this->x_ = this->A_ * this->x_ + this->B_ * u;

    if (this->output_noise_present_) {
      this->y_ += this->GenerateOutputNoise();
    }
  }
};

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class ContinuousPlant : public Plant<state_dim, input_dim, output_dim, Scalar> {
 public:
  using Plant<state_dim, input_dim, output_dim, Scalar>::Plant;

  using typename Plant<state_dim, input_dim, output_dim, Scalar>::u_VectorType;
  using typename Plant<state_dim, input_dim, output_dim, Scalar>::x_VectorType;

  void SetTimestep(double timestep) { timestep_ = timestep; }

  // TODO: consider using discretization method in analysis.hpp?
  void Update(const u_VectorType& u) override {
    Update(u, timestep_);

    if (this->output_noise_present_) {
      this->y_ += this->GenerateOutputNoise();
    }
  }

  // runge kutta with zero-order hold
  // https://math.stackexchange.com/questions/2946737/
  void Update(const u_VectorType& u, double dt) {
    this->y_ = this->C_ * this->x_ + this->D_ * u;
    this->x_ = numerical::integral::rk4(
        [&](const x_VectorType& x) -> x_VectorType {
          return this->A_ * x + this->B_ * u;
        },
        this->x_, dt);
  }

 private:
  double timestep_ = 0.01;
};

}  // namespace sim
}  // namespace lti
}  // namespace dynamical
