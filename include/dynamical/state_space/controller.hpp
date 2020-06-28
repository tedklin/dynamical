#pragma once

#include "dynamical/state_space/plant.hpp"

#include <memory>

#include "Eigen/Dense"

namespace dynamical {

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class Feedback {
 public:
  // TODO: figure out how to forward types from a completely unrelated template
  // (in this case, Plant<..>)
  using x_VectorType = Eigen::Matrix<Scalar, state_dim, 1>;
  using u_VectorType = Eigen::Matrix<Scalar, input_dim, 1>;

  // K_ref matrix dimensions are different than K if our reference signal is not
  // full state, but we should be able to avoid differentiating between the two
  // by ensuring our trajectories output reference signals with the appropriate
  // dimensions.
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

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class Observer {
  // TODO: implement
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

  // TODO: does this even make sense to have?
  Controller(const FeedbackType& feedback, const ObserverType& observer) {
    feedback_ptr_ = std::make_shared<FeedbackType>(feedback);
    observer_ptr_ = std::make_shared<ObserverType>(observer);
  }

 private:
  std::shared_ptr<FeedbackType> feedback_ptr_;
  std::shared_ptr<ObserverType> observer_ptr_;
};

}  // namespace dynamical
