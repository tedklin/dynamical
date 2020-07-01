#pragma once

#include "dynamical/state_space/plant.hpp"

#include <memory>

#include "Eigen/Dense"

namespace dynamical {
namespace lti {

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class Feedback {
 public:
  // TODO: figure out how to forward types from a completely unrelated template
  // (in this case, Plant<..>)
  using x_VectorType = Eigen::Matrix<Scalar, state_dim, 1>;
  using u_VectorType = Eigen::Matrix<Scalar, input_dim, 1>;

  // K_ref matrix dimensions might be different than K if our reference signal
  // is not full state, but we should be able to avoid differentiating between
  // the two by ensuring our trajectories output reference signals with the
  // appropriate dimensions.
  using K_MatrixType = Eigen::Matrix<Scalar, input_dim, state_dim>;

  explicit Feedback(const K_MatrixType& K,
                    const K_MatrixType& K_ref = K_MatrixType::Zero())
      : K_(K), K_ref_(K_ref) {}

  u_VectorType GetU(const x_VectorType& x_hat,
                    const x_VectorType& ref = x_VectorType::Zero()) const {
    return K_ * x_hat + K_ref_ * ref;
  }

  const K_MatrixType& GetK() const { return K_; }

  const K_MatrixType& GetKref() const { return K_ref_; }

  void SetK(const K_MatrixType& K) { K_ = K; }

  void SetKref(const K_MatrixType& K_ref) { K_ref_ = K_ref; }

 private:
  K_MatrixType K_;
  K_MatrixType K_ref_;
};

// Currently only supports discrete-time systems.
template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class Observer {
 public:
  // TODO: figure out how to forward types from a completely unrelated template
  // (in this case, Plant<..>)
  using A_MatrixType = Eigen::Matrix<Scalar, state_dim, state_dim>;
  using B_MatrixType = Eigen::Matrix<Scalar, state_dim, input_dim>;
  using C_MatrixType = Eigen::Matrix<Scalar, output_dim, state_dim>;
  using D_MatrixType = Eigen::Matrix<Scalar, output_dim, input_dim>;
  using x_VectorType = Eigen::Matrix<Scalar, state_dim, 1>;
  using y_VectorType = Eigen::Matrix<Scalar, output_dim, 1>;
  using u_VectorType = Eigen::Matrix<Scalar, input_dim, 1>;

  using L_MatrixType = Eigen::Matrix<Scalar, state_dim, output_dim>;

  Observer() = delete;

  Observer(const DiscretePlant<state_dim, input_dim, output_dim, Scalar>& plant,
           const L_MatrixType& L,
           const x_VectorType& x_hat_initial = x_VectorType::Zero())
      : A_(plant.A_),
        B_(plant.B_),
        C_(plant.C_),
        D_(plant.D_),
        L_(plant.L_),
        x_hat_(x_hat_initial) {}

  void UpdateEstimate(const y_VectorType& y, const u_VectorType& u) {
    y_hat_ = C_ * x_hat_ + D_ * u;
    x_hat_ = A_ * x_hat_ + B_ * u + L_ * (y - y_hat_);
  }

  const x_VectorType& GetXhat() const { return x_hat_; }

  const A_MatrixType A_;
  const B_MatrixType B_;
  const C_MatrixType C_;
  const D_MatrixType D_;
  const L_MatrixType L_;

 protected:
  x_VectorType x_hat_;
  y_VectorType y_hat_ = y_VectorType::Zero();
};

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class Controller {
 public:
  using FeedbackType = Feedback<state_dim, input_dim, output_dim, Scalar>;
  using ObserverType = Observer<state_dim, input_dim, output_dim, Scalar>;

  Controller(std::shared_ptr<FeedbackType> feedback_ptr,
             std::shared_ptr<ObserverType> observer_ptr)
      : feedback_ptr_(feedback_ptr), observer_ptr_(observer_ptr) {}

 private:
  std::shared_ptr<FeedbackType> feedback_ptr_;
  std::shared_ptr<ObserverType> observer_ptr_;
};

}  // namespace lti
}  // namespace dynamical
