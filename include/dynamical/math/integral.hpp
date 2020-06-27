#pragma once

#include <functional>

namespace dynamical {
namespace numerical {

// This wasn't tested rigorously yet. Runge-kutta was only tested in the context
// of discretization with very small steps (see state_space/plant_test.cpp).
namespace integral {
// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods

// THIS VERSION WORKS FOR NOW, but can we ensure the type of the function
// (having one parameter of the same type as the return type) somehow?
template <typename F, typename T>
T rk4(const F& dy_dt, T y0, double dt) {
  T k1 = dy_dt(y0);
  T k2 = dy_dt(y0 + (dt * k1 / 2.0));
  T k3 = dy_dt(y0 + (dt * k2 / 2.0));
  T k4 = dy_dt(y0 + dt * k3);
  return y0 + (dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4));
}

// template <typename T>
// T rk4(T (*dy_dt)(const T& y) dy_dt, T y0, double dt) {
//   T k1 = dy_dt(y0);
//   T k2 = dy_dt(y0 + (dt * k1 / 2.0));
//   T k3 = dy_dt(y0 + (dt * k2 / 2.0));
//   T k4 = dy_dt(y0 + dt * k3);
//   return y0 + (dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4));
// }
// these didn't work as the function parameter
// T (*dy_dt)(const T& y)
// std::function<T(const T&)> dy_dt

// TODO: add other numerical integration methods?

}  // namespace integral

}  // namespace numerical
}  // namespace dynamical
