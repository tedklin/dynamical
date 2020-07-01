#pragma once

#include "dynamical/utils/exception.hpp"

#include <cmath>
#include <iostream>

#include "Eigen/Dense"

namespace dynamical_testing {
namespace dynamical_test_utils {

bool check_matrix_equality(Eigen::MatrixXcd expected, Eigen::MatrixXcd actual,
                           double tolerance = 1e-9) {
  if (expected.rows() != actual.rows() || expected.cols() != actual.cols()) {
    throw dynamical::dynamical_error(
        "matrix equality check: tried to compare matrices of different "
        "dimensions!\n\n");
    return false;
  }
  for (int col = 0; col != expected.cols(); ++col) {
    for (int row = 0; row != expected.rows(); ++row) {
      if (std::abs(expected(row, col) - actual(row, col)) > tolerance) {
        std::cerr << "matrix equality check: expected\n"
                  << expected(row, col) << "\n\nbut got\n"
                  << actual(row, col) << "\n\n";
        return false;
      }
    }
  }
  return true;
}

}  // namespace dynamical_test_utils
}  // namespace dynamical_testing
