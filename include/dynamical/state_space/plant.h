#pragma once

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

  const A_matrix_type A_;
  const B_matrix_type B_;
  const C_matrix_type C_;
  const D_matrix_type D_;

  const x_vector_type& GetX() const { return x_; }
  const y_vector_type& GetY() const { return y_; }

  void SetXInitial(const x_vector_type& initial_x) { x_initial_ = initial_x; }

  void ResetToXInitial() { x_ = x_initial_; }

  virtual void Update(const u_vector_type& u) = 0;

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

namespace analysis {

template <int state_dim, int input_dim, int output_dim, typename Scalar>
auto get_reachability_matrix(
    const dynamical::Plant<state_dim, input_dim, output_dim, Scalar>& plant,
    int num_steps_allowed = state_dim) {
  Eigen::Matrix<Scalar, state_dim, Eigen::Dynamic> reachability_matrix(
      state_dim, (num_steps_allowed * input_dim));
  Eigen::Matrix<Scalar, state_dim, input_dim> step_block = plant.B_;

  for (int step = 0; step != num_steps_allowed; ++step) {
    for (int block_index = 0; block_index != input_dim; ++block_index) {
      int reachability_col = (step * input_dim) + block_index;
      reachability_matrix.col(reachability_col) = step_block.col(block_index);
    }
    step_block = plant.A_ * step_block;
  }

  // TODO: this returns an entire copy, but the underlying reachability matrix
  // will be destroyed (local lifetime) if we pass by pointer or reference. is
  // there a more elegant way to do this without needing the user to declare the
  // reachability matrix themselves?
  return reachability_matrix;
}

}  // namespace analysis
}  // namespace dynamical
