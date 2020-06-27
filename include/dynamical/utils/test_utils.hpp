#pragma once

#include <cmath>
#include <iostream>

#include "Eigen/Dense"

namespace testing {
namespace dynamical_test_utils {

// TODO: fix this, right now complex matrix types can't match to the template
// parameters correctly
// template <typename S, int num_rows, int num_cols,
// typename T> bool check_matrix_equality(Eigen::Matrix<S, num_rows, num_cols>
// expected,
//                            Eigen::Matrix<T, num_rows, num_cols> actual,
//                            double tolerance = 1e-9) {
//   if (expected.rows() != actual.rows() || expected.cols() != actual.cols()) {
//     // TODO: make into exception?
//     std::cout << "matrix equality check error: tried to compare matrices of "
//                  "different dimensions!\n\n";
//     return false;
//   }
//   for (int col = 0; col != expected.cols(); ++col) {
//     for (int row = 0; row != expected.rows(); ++row) {
//       if (std::abs(expected(row, col) - actual(row, col)) > tolerance) {
//         std::cout << "matrix equality check error: expected\n"
//                   << expected(row, col) << "\n\nbut got\n"
//                   << actual(row, col) << "\n\n";
//         return false;
//       }
//     }
//   }
//   return true;
// }

bool check_matrix_equality(Eigen::MatrixXd expected, Eigen::MatrixXd actual,
                           double tolerance = 1e-9) {
  if (expected.rows() != actual.rows() || expected.cols() != actual.cols()) {
    // TODO: make into exception?
    std::cout << "matrix equality check error: tried to compare matrices of "
                 "different dimensions!\n\n";
    return false;
  }
  for (int col = 0; col != expected.cols(); ++col) {
    for (int row = 0; row != expected.rows(); ++row) {
      if (std::abs(expected(row, col) - actual(row, col)) > tolerance) {
        std::cout << "matrix equality check error: expected\n"
                  << expected(row, col) << "\n\nbut got\n"
                  << actual(row, col) << "\n\n";
        return false;
      }
    }
  }
  return true;
}

// can't overload because Eigen types are too similar (impl of Xcd might be
// derived from Xd?)
bool check_complex_matrix_equality(Eigen::MatrixXcd expected,
                                   Eigen::MatrixXcd actual,
                                   double tolerance = 1e-5) {
  if (expected.rows() != actual.rows() || expected.cols() != actual.cols()) {
    // TODO: make into exception?
    std::cout << "matrix equality check error: tried to compare matrices of "
                 "different dimensions!\n\n";
    return false;
  }
  for (int col = 0; col != expected.cols(); ++col) {
    for (int row = 0; row != expected.rows(); ++row) {
      if (std::abs(expected(row, col) - actual(row, col)) > tolerance) {
        std::cout << "matrix equality check error: expected\n"
                  << expected(row, col) << "\n\nbut got\n"
                  << actual(row, col) << "\n\n";
        return false;
      }
    }
  }
  return true;
}

// ugh this file makes my eyes hurt.
bool check_mixed_matrix_equality(Eigen::MatrixXd expected,
                                 Eigen::MatrixXcd actual,
                                 double tolerance = 1e-5) {
  if (expected.rows() != actual.rows() || expected.cols() != actual.cols()) {
    // TODO: make into exception?
    std::cout << "matrix equality check error: tried to compare matrices of "
                 "different dimensions!\n\n";
    return false;
  }
  for (int col = 0; col != expected.cols(); ++col) {
    for (int row = 0; row != expected.rows(); ++row) {
      if (std::abs(expected(row, col) - actual(row, col)) > tolerance) {
        std::cout << "matrix equality check error: expected\n"
                  << expected(row, col) << "\n\nbut got\n"
                  << actual(row, col) << "\n\n";
        return false;
      }
    }
  }
  return true;
}

}  // namespace dynamical_test_utils
}  // namespace testing
