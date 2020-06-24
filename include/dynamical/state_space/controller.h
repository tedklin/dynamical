#pragma once

#include "dynamical/math/integration.h"

#include <memory>

#include "Eigen/Dense"

namespace dynamical {

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class Feedback {
 public:
  using K_MatrixType = Eigen::Matrix<Scalar, input_dim, state_dim>;
  using Kref_MatrixType = Eigen::Matrix<Scalar, input_dim, state_dim>;

  explicit Feedback(const K_MatrixType& K,
                    const Kref_MatrixType& Kref = Kref_MatrixType::Zero())
      : K_(K), Kref_(Kref) {}

  const K_MatrixType& GetK() const { return K_; }

  const Kref_MatrixType& GetKref() const { return Kref_; }

  void SetK(const K_MatrixType& new_K) { K_ = new_K; }

  void SetKref(const Kref_MatrixType& new_Kref) { Kref_ = new_Kref; }

 private:
  K_MatrixType K_;
  Kref_MatrixType Kref_;
};

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class Observer {};

template <int state_dim, int input_dim, int output_dim = state_dim,
          typename Scalar = double>
class Controller {
 public:
  using FeedbackType = Feedback<state_dim, input_dim, output_dim, Scalar>;
  using ObserverType = Observer<state_dim, input_dim, output_dim, Scalar>;

 private:
  std::shared_ptr<FeedbackType>;
  std::shared_ptr<ObserverType>;
};

}  // namespace dynamical
