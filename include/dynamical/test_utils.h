#pragma once

#include <cmath>
#include <iostream>

#include "Eigen/Dense"

namespace testing {
namespace test_utils {

bool check_matrix_equality(Eigen::MatrixXd expected, Eigen::MatrixXd actual,
                           double tolerance = 1e-9) {
  if (expected.rows() != actual.rows() || expected.cols() != actual.cols()) {
    // TODO: make into exception?
    std::cout
        << "Error: tried to compare matrices of different dimensions!\n\n";
    return false;
  }
  for (int col = 0; col != expected.cols(); ++col) {
    for (int row = 0; row != expected.rows(); ++row) {
      if (std::abs(expected(row, col) - actual(row, col)) > tolerance) {
        return false;
      }
    }
  }
  return true;
}

// TODO: why can't we overload this?
bool check_complex_matrix_equality(Eigen::MatrixXcd expected,
                                   Eigen::MatrixXcd actual,
                                   double tolerance = 1e-5) {
  if (expected.rows() != actual.rows() || expected.cols() != actual.cols()) {
    std::cout
        << "Error: tried to compare matrices of different dimensions!\n\n";
    return false;
  }
  for (int col = 0; col != expected.cols(); ++col) {
    for (int row = 0; row != expected.rows(); ++row) {
      if (std::abs(expected(row, col) - actual(row, col)) > tolerance) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace test_utils
}  // namespace testing
