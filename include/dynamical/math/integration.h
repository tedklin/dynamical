#pragma once

#include <functional>

namespace dynamical {
namespace numerical {
namespace integration {
// TODO: add other numerical integration techniques?
// keep in mind they should all have the same parameter list and return type.

// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
// note that the y here is just a name and is completely arbitrary.
template <typename T>
T rk4(std::function<T(const T&)> dy_dt, T y0, double dt) {
  T k1 = dy_dt(y0);
  T k2 = dy_dt(y0 + (dt * k1 / 2.0));
  T k3 = dy_dt(y0 + (dt * k2 / 2.0));
  T k4 = dy_dt(y0 + dt * k3);
  return y0 + (dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4));
}
// these didn't work as the function parameter, don't know why
// T (*dy_dt)(const T& y)
// std::function<T(const T&)> dy_dt

// // THIS VERSION WORKS, but we want to ensure the type of the function (having
// one parameter of the same type as the return type) if we can!
// template <typename F, typename T>
// T rk4(F dy_dt, T y0, double dt) {
//   T k1 = dy_dt(y0);
//   T k2 = dy_dt(y0 + (dt * k1 / 2.0));
//   T k3 = dy_dt(y0 + (dt * k2 / 2.0));
//   T k4 = dy_dt(y0 + dt * k3);
//   return y0 + (dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4));
// }

}  // namespace integration
}  // namespace numerical
}  // namespace dynamical
