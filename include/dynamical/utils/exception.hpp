#pragma once

#include <stdexcept>
#include <string>

namespace dynamical {

std::runtime_error dynamical_error(const std::string& msg) {
  return std::runtime_error("(dynamical exception) " + msg);
}

}  // namespace dynamical
