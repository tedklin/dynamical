#pragma once

#include "dynamical/math/integral.hpp"

#include <cmath>
#include <memory>
#include <random>

#include "Eigen/Dense"

// Simulation for plants.

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

  // TODO: revisit copy-control members' treatment of the unique_ptrs
  Plant(const Plant& other)
      : A_(other.A_),
        B_(other.B_),
        C_(other.C_),
        D_(other.D_),
        x_initial_(other.x_initial_),
        x_(other.x_),
        y_(other.y_) {}

  Plant& operator=(const Plant& rho) {
    A_ = rho.A_;
    B_ = rho.B_;
    C_ = rho.C_;
    D_ = (rho.D_);
    x_initial_ = (rho.x_initial);
    x_ = (rho.x_);
    y_ = (rho.y_);
    noise_present_ = false;
    noise_stddev_ = 0;
    return *this;
  }

  virtual void Update(const u_VectorType& u) = 0;

  const x_VectorType& GetX() const { return x_; }

  const y_VectorType& GetY() const { return y_; }

  void ResetToXInitial() { x_ = x_initial_; }

  void SetXInitial(const x_VectorType& initial_x) { x_initial_ = initial_x; }

  const x_VectorType& GetXInitial() const { return x_initial_; }

  void EnableNoise(double new_noise_stddev) {
    noise_present_ = true;

    // TODO: profile this?
    std::random_device rd;
    random_generator_.reset(new std::mt19937(rd()));
    noise_distribution_.reset(
        new std::normal_distribution<>{0, new_noise_stddev});
  }

  void DisableNoise() { noise_present_ = false; }

  const A_MatrixType A_;
  const B_MatrixType B_;
  const C_MatrixType C_;
  const D_MatrixType D_;

 protected:
  x_VectorType x_initial_;
  x_VectorType x_;
  y_VectorType y_ = y_VectorType::Zero();

  bool noise_present_ = false;
  double noise_stddev_ = 0;
  std::unique_ptr<std::mt19937> random_generator_;
  std::unique_ptr<std::normal_distribution<>> noise_distribution_;
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

    if (this->noise_present_) {
      y_VectorType y_noise = y_VectorType::NullaryExpr([&]() {
        return (*(this->noise_distribution_))(*(this->random_generator_));
      });
      this->y_ += y_noise;
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

  void SetTimestep(double timestep) { timestep_ = timestep }

  // TODO: consider using discretization method in analysis.hpp?
  void Update(const u_VectorType& u) override { UpdateSim(u, timestep_); }

  // runge kutta with zero-order hold
  // https://math.stackexchange.com/questions/2946737/
  void UpdateSim(const u_VectorType& u, double dt) {
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
