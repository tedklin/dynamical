// Simulation for plants, with support for generated Gaussian noise.

#pragma once

#include <cmath>
#include <memory>
#include <random>

#include "Eigen/Dense"

#include "dynamical/math/integral.hpp"
#include "dynamical/utils/noise_gen.hpp"

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
      : A_(A), B_(B), C_(C), D_(D), x_initial_(x_initial), x_(x_initial) {}

  virtual void Update(const u_VectorType& u) = 0;

  const x_VectorType& GetX() const { return x_; }

  const y_VectorType& GetY() const { return y_; }

  void ResetToXInitial() { x_ = x_initial_; }

  void SetXInitial(const x_VectorType& initial_x) { x_initial_ = initial_x; }

  const x_VectorType& GetXInitial() const { return x_initial_; }

  void EnableOutputNoise(const y_VectorType& output_stddev) {
    output_noise_gen_ = std::make_shared<NoiseGenerator>(output_stddev);
  }

  void DisableOutputNoise() { output_noise_gen_.reset(); }

  void EnableProcessNoise(const x_VectorType& process_stddev) {
    process_noise_gen_ = std::make_shared<NoiseGenerator>(process_stddev);
  }

  void DisableProcessNoise() { process_noise_gen_.reset(); }

  const A_MatrixType A_;
  const B_MatrixType B_;
  const C_MatrixType C_;
  const D_MatrixType D_;

 protected:
  x_VectorType x_initial_;
  x_VectorType x_;
  y_VectorType y_ = y_VectorType::Zero();

  std::shared_ptr<NoiseGenerator> output_noise_gen_;
  std::shared_ptr<NoiseGenerator> process_noise_gen_;

  y_VectorType OutputNoise() {
    if (output_noise_gen_) {
      return output_noise_gen_->GetNoise();
    }
    return y_VectorType::Zero();
  }

  x_VectorType ProcessNoise() {
    if (process_noise_gen_) {
      return process_noise_gen_->GetNoise();
    }
    return x_VectorType::Zero();
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
    this->y_ = this->C_ * this->x_ + this->D_ * u + this->OutputNoise();
    this->x_ = this->A_ * this->x_ + this->B_ * u + this->ProcessNoise();
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
    this->y_ += this->OutputNoise();
    this->x_ += this->ProcessNoise();
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
