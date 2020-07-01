#include "dynamical/math/integral.hpp"

#include <cmath>

#include "Eigen/Dense"
#include "gtest/gtest.h"

// NOTE: the tests in this file that are currently implemented are ONLY for
// checking the function matching capabilities of the runge-kutta
// implementation. A test that actually checks the effectiveness of runge-kutta
// in the context of estimating continuous-time dynamics can be found in
// state_space/plant_test.cpp.

namespace dynamical_testing {

TEST(Integration, RK4Scalar) {
  double result = dynamical::numerical::integral::rk4(
      [](const double& x) -> double { return std::exp(x); }, 2.0, 0.01);
}

TEST(Integration, RK4Vector) {
  // mock continuous-time plant
  const Eigen::Matrix2d A = Eigen::Matrix2d::Random();
  const Eigen::Matrix2d B = Eigen::Matrix2d::Random();
  const Eigen::Vector2d curr_x = Eigen::Vector2d::Random();
  const Eigen::Vector2d u = Eigen::Vector2d::Random();

  Eigen::Vector2d result = dynamical::numerical::integral::rk4(
      [&](const Eigen::Vector2d& x) -> Eigen::Vector2d {
        return A * x + B * u;
      },
      curr_x, 0.01);
}

}  // namespace dynamical_testing
