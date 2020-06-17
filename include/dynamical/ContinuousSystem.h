#pragma once

#include "Eigen/Eigen"

// simple continuous-time state feedback system analyzer
// 3-states, 1-input

class ContinuousSystem {
 public:
  explicit ContinuousSystem(const Eigen::Matrix3d &A)
      : ContinuousSystem(A, Eigen::Vector3d::Zero(),
                         Eigen::RowVector3d::Zero()) {}

  ContinuousSystem(const Eigen::Matrix3d &A, const Eigen::Vector3d &B,
                   const Eigen::RowVector3d &K)
      : A_(A), B_(B), K_(K) {
    system_ = A + (B * K);
    Eigen::EigenSolver<Eigen::Matrix3d> solver(system_);
    eigenvalues_ = solver.eigenvalues();
  }

  const Eigen::Matrix3d &GetSystem() const;

  const Eigen::EigenSolver<Eigen::Matrix3d>::EigenvalueType &GetEigenvalues()
      const;

  bool IsStable() const;

 private:
  // eventually template this
  int STATE_DIM = 3;
  int INPUT_DIM = 1;
  using SCALAR = double;

  // eventually type alias all of this using above template stuff
  Eigen::Matrix3d A_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d B_ = Eigen::Vector3d::Zero();
  Eigen::RowVector3d K_ = Eigen::RowVector3d::Zero();
  Eigen::Matrix3d system_;
  Eigen::EigenSolver<Eigen::Matrix3d>::EigenvalueType eigenvalues_;
};
