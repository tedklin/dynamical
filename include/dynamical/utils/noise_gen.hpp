// Gaussian noise generation for Eigen column vectors.

#pragma once

#include <cmath>
#include <memory>
#include <random>
#include <vector>

#include "Eigen/Dense"

#include "dynamical/utils/exception.hpp"

namespace dynamical {

class NoiseGenerator {
 public:
  NoiseGenerator(Eigen::Matrix<double, Eigen::Dynamic, 1> stddev)
      : dimension_(stddev.rows()) {
    std::random_device rd;
    generator_ = std::make_shared<std::mt19937>(rd());
    for (int i = 0; i != stddev.rows(); ++i) {
      if (stddev(i, 0) <= 0) {
        throw dynamical_error(
            "Noise generation stddev input not greater than 0!");
      }
      distributions_.push_back(
          std::make_shared<std::normal_distribution<>>(0, stddev(i, 0)));
    }
  }

  const int GetDimension() const { return dimension_; }

  Eigen::Matrix<double, Eigen::Dynamic, 1> GetNoise() {
    Eigen::Matrix<double, Eigen::Dynamic, 1> w(dimension_);
    for (int i = 0; i != w.rows(); ++i) {
      w(i, 0) = (*distributions_[i])(*generator_);
    }
    return w;
  }

 private:
  const int dimension_;
  std::shared_ptr<std::mt19937> generator_;
  std::vector<std::shared_ptr<std::normal_distribution<>>> distributions_;
};

}  // namespace dynamical
