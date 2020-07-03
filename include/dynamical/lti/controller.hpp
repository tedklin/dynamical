#pragma once

#include "dynamical/lti/plant.hpp"
#include "dynamical/trajectory/trajectory.hpp"

#include <memory>

#include "Eigen/Dense"

// State feedback controller.

namespace dynamical {
namespace lti {

template <int state_dim, int input_dim, typename Scalar = double>
class Feedback {
 public:
  using x_VectorType = Eigen::Matrix<Scalar, state_dim, 1>;
  using u_VectorType = Eigen::Matrix<Scalar, input_dim, 1>;

  // K_ref matrix dimensions might be different than K if our reference signal
  // is not full state, but we should be able to avoid differentiating between
  // the two by ensuring our trajectories output reference signals with the
  // appropriate dimensions.
  using K_MatrixType = Eigen::Matrix<Scalar, input_dim, state_dim>;

  Feedback() = delete;

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
template <int state_dim, int input_dim, int output_dim,
          typename Scalar = double>
class Observer {
 public:
  using A_MatrixType = Eigen::Matrix<Scalar, state_dim, state_dim>;
  using B_MatrixType = Eigen::Matrix<Scalar, state_dim, input_dim>;
  using C_MatrixType = Eigen::Matrix<Scalar, output_dim, state_dim>;
  using D_MatrixType = Eigen::Matrix<Scalar, output_dim, input_dim>;
  using x_VectorType = Eigen::Matrix<Scalar, state_dim, 1>;
  using y_VectorType = Eigen::Matrix<Scalar, output_dim, 1>;
  using u_VectorType = Eigen::Matrix<Scalar, input_dim, 1>;

  using L_MatrixType = Eigen::Matrix<Scalar, state_dim, output_dim>;

  Observer() = delete;

  Observer(const x_VectorType& x_hat_initial, const L_MatrixType& L,
           const A_MatrixType& A, const B_MatrixType& B, const C_MatrixType& C,
           const D_MatrixType& D)
      : L_(L), A_(A), B_(B), C_(C), D_(D), x_hat_(x_hat_initial) {}

  Observer(
      const x_VectorType& x_hat_initial, const L_MatrixType& L,
      const sim::DiscretePlant<state_dim, input_dim, output_dim, Scalar>& plant)
      : Observer(x_hat_initial, L, plant.A_, plant.B_, plant.C_, plant.D_) {}

  void UpdateEstimate(const y_VectorType& y, const u_VectorType& u) {
    y_hat_ = C_ * x_hat_ + D_ * u;
    x_hat_ = A_ * x_hat_ + B_ * u + L_ * (y - y_hat_);
  }

  const x_VectorType& GetXhat() const { return x_hat_; }

  const L_MatrixType L_;
  const A_MatrixType A_;
  const B_MatrixType B_;
  const C_MatrixType C_;
  const D_MatrixType D_;

 protected:
  x_VectorType x_hat_;
  y_VectorType y_hat_ = y_VectorType::Zero();
};

template <int state_dim, int input_dim, int output_dim,
          typename Scalar = double>
class Controller {
 public:
  using x_VectorType = Eigen::Matrix<Scalar, state_dim, 1>;
  using y_VectorType = Eigen::Matrix<Scalar, output_dim, 1>;
  using u_VectorType = Eigen::Matrix<Scalar, input_dim, 1>;

  using FeedbackType = Feedback<state_dim, input_dim, Scalar>;
  using ObserverType = Observer<state_dim, input_dim, output_dim, Scalar>;
  using TrajectoryType = trajectory::Trajectory<state_dim, input_dim, Scalar>;

  Controller(std::shared_ptr<FeedbackType> feedback_ptr,
             std::shared_ptr<ObserverType> observer_ptr,
             std::shared_ptr<TrajectoryType> trajectory_ptr)
      : feedback_(feedback_ptr),
        observer_(observer_ptr),
        trajectory_(trajectory_ptr) {}

  // The intuition behind this might be misleading at first to people accustomed
  // to direct feedback (like I was). The y passed in does not actually play a
  // direct role in the generation of the u returned, but rather influences the
  // next u. The sole purpose of the y passed in is to update the state
  // estimator.
  // TODO: come up with a more straightforward name for this function
  const u_VectorType& GetU(const y_VectorType& y, int trajectory_step) {
    u_ = feedback_->GetU(observer_->GetXhat(),
                         trajectory_->GetDesiredState(trajectory_step)) +
         trajectory_->GetFeedforward(trajectory_step);

    observer_->UpdateEstimate(y, u_);

    return u_;
  }

 private:
  std::shared_ptr<FeedbackType> feedback_;
  std::shared_ptr<ObserverType> observer_;
  std::shared_ptr<TrajectoryType> trajectory_;

  u_VectorType u_ = u_VectorType::Zero();
};

}  // namespace lti
}  // namespace dynamical
