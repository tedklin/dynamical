#pragma once

#include "Eigen/Dense"

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

  Plant() = delete;  // delete default constructor

  Plant(const A_matrix_type& A, const B_matrix_type& B,
        const C_matrix_type& C = C_matrix_type::Identity(),
        const D_matrix_type& D = D_matrix_type::Zero())
      : A_(A), B_(B), C_(C), D_(D) {}

  Plant(const Plant& other)
      : A_(other.A_), B_(other.B_), C_(other.C_), D_(other.D_) {}

  virtual ~Plant() = default;

  const A_matrix_type A_;
  const B_matrix_type B_;
  const C_matrix_type C_;
  const D_matrix_type D_;

  const x_vector_type& GetX() { return x_; }
  const y_vector_type& GetY() { return y_; }

  virtual void Update(const u_vector_type& u) = 0;

 private:
  x_vector_type x_;
  y_vector_type y_;
};

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class DiscretePlant : public Plant {
 public:
  using Plant::Plant;  // inherit constructors

  void Update(const u_vector_type& u) override {
    x_ = A_ * x_ + B_ * u;
    y_ = C_ * x_ + D_ * u;
  }
};
