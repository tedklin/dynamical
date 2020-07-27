// State feedback controller as seen in A&M "Feedback Systems".

// The Controller class provides an interface for integrating
// Trajectory (dynamical/trajectory/trajectory.hpp), Feedback, Observer
// components.

// The Observer (and thus the complete Controller) only support discrete-time
// systems, but the Feedback component is the same for both discrete-time and
// continuous-time.

#pragma once

#include <memory>

#include "Eigen/Dense"

#include "dynamical/lti/plant.hpp"
#include "dynamical/trajectory/trajectory.hpp"
#include "dynamical/utils/exception.hpp"

namespace dynamical {
namespace lti {

template <int state_dim, int input_dim, typename Scalar = double>
class Feedback {
 public:
  using x_VectorType = Eigen::Matrix<Scalar, state_dim, 1>;
  using u_VectorType = Eigen::Matrix<Scalar, input_dim, 1>;

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
      : A_(A), B_(B), C_(C), D_(D), x_hat_(x_hat_initial), L_(L) {}

  Observer(
      const x_VectorType& x_hat_initial, const L_MatrixType& L,
      const sim::DiscretePlant<state_dim, input_dim, output_dim, Scalar>& plant)
      : Observer(x_hat_initial, L, plant.A_, plant.B_, plant.C_, plant.D_) {}

  virtual void UpdateEstimate(const y_VectorType& y, const u_VectorType& u) {
    y_hat_ = C_ * x_hat_ + D_ * u;
    x_hat_ = A_ * x_hat_ + B_ * u + L_ * (y - y_hat_);
  }

  const x_VectorType& GetXhat() const { return x_hat_; }

  const L_MatrixType& GetL() const { return L_; }

  void SetL(const L_MatrixType& L) { L_ = L; }

  const A_MatrixType A_;
  const B_MatrixType B_;
  const C_MatrixType C_;
  const D_MatrixType D_;

 protected:
  x_VectorType x_hat_;
  y_VectorType y_hat_ = y_VectorType::Zero();

  L_MatrixType L_;
};

template <int state_dim, int input_dim, int output_dim,
          typename Scalar = double>
class KalmanFilter : public Observer<state_dim, input_dim, output_dim, Scalar> {
 public:
  using A_MatrixType = Eigen::Matrix<Scalar, state_dim, state_dim>;
  using B_MatrixType = Eigen::Matrix<Scalar, state_dim, input_dim>;
  using C_MatrixType = Eigen::Matrix<Scalar, output_dim, state_dim>;
  using D_MatrixType = Eigen::Matrix<Scalar, output_dim, input_dim>;
  using x_VectorType = Eigen::Matrix<Scalar, state_dim, 1>;
  using y_VectorType = Eigen::Matrix<Scalar, output_dim, 1>;
  using u_VectorType = Eigen::Matrix<Scalar, input_dim, 1>;

  using L_MatrixType = Eigen::Matrix<Scalar, state_dim, output_dim>;
  using P_MatrixType = Eigen::Matrix<Scalar, state_dim, state_dim>;
  using R_MatrixType = Eigen::Matrix<Scalar, output_dim, output_dim>;

  KalmanFilter() = delete;

  KalmanFilter(const x_VectorType& x_hat_initial, const A_MatrixType& A,
               const B_MatrixType& B, const C_MatrixType& C,
               const D_MatrixType& D, P_MatrixType P, P_MatrixType Q,
               R_MatrixType R)
      : Observer<state_dim, input_dim, output_dim, Scalar>(
            x_hat_initial, L_MatrixType::Zero(), A, B, C, D),
        P_(P),
        Q_(Q),
        R_(R) {}

  KalmanFilter(
      const x_VectorType& x_hat_initial,
      const sim::DiscretePlant<state_dim, input_dim, output_dim, Scalar>& plant,
      P_MatrixType P, P_MatrixType Q, R_MatrixType R)
      : Observer<state_dim, input_dim, output_dim, Scalar>(
            x_hat_initial, L_MatrixType::Zero(), plant),
        P_(P),
        Q_(Q),
        R_(R) {}

  void UpdateEstimate(const y_VectorType& y, const u_VectorType& u) {
    // Predict state covariance matrix.
    P_ = this->A_ * P_ * this->A_.transpose() + Q_;

    // Calculate system uncertainty.
    S_ = this->C_ * P_ * this->C_.transpose() + R_;
    Eigen::FullPivLU<decltype(S_)> lu(S_);
    if (!lu.isInvertible()) {
      throw dynamical_error("kalman filter: S matrix not invertible.");
    }

    // Update Kalman gain.
    this->L_ = P_ * this->C_.transpose() * S_.inverse();

    // Predict and update state estimate.
    this->y_hat_ = this->C_ * this->x_hat_ + this->D_ * u;
    this->x_hat_ =
        this->A_ * this->x_hat_ + this->B_ * u + this->L_ * (y - this->y_hat_);

    // TODO: find more numerically stable version of this calculation?
    // Update state covariance matrix.
    P_ = P_ - (this->L_ * this->C_ * P_);
  }

 private:
  // Quick summary of what these matrices represent, based on Labbe's set of
  // Jupyter notebooks for practical Kalman filter design:
  //
  // Recall covariance matrices encode variances on the diagonal and covariances
  // on non-diagonal elements. Variance can roughly be thought of here as how
  // much we trust something. For now, we will ignore initial settings of
  // covariances (they will update themselves as the filter evolves).
  //
  // If we *do not* have a lot of trust in our model, then we should design our
  // process variances (along the diagonal of Q) to have *larger* magnitude.
  // Similarly, if we *do* have a lot of trust in our estimate of the initial
  // state, then our initial P should reflect that with a diagonal of elements
  // of *smaller* magnitude. The key is to set variances that truly reflect your
  // confidence in the data.

  P_MatrixType P_;  // state covariance matrix
  P_MatrixType Q_;  // process noise covariance matrix
  R_MatrixType R_;  // measurement noise covariance matrix

  R_MatrixType S_ = R_MatrixType::Zero();  // system uncertainty
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

  // The intuition behind this might be misleading at first. The y passed in
  // does not actually play a direct role in the generation of the u returned,
  // but rather influences the next u. The sole purpose of the y passed in is to
  // update the state estimate.
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
