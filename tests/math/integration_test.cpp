#include "dynamical/math/integration.h"

#include <cmath>

#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace testing {

TEST(IntegrationTest, RK4Scalar) {
  // we know the function dy_dt(y) (this is what's inside the lambda)
  // we also know y0 at some time t0.
  // runge kutta gives y1, which is y(t0 + dt)

  // remember we deal with linear time-invariant systems, so dy/dt should only
  // depend on y, not t.
  // but then what parameter does y take (if not time) and how can we test
  // this???

  // double result = dynamical::numerical::integration::rk4(
  //     [](const double& x) -> double { return std::exp(x); }, 2.0, 0.01);
}

TEST(IntegrationTest, RK4Vector) {}

}  // namespace testing
