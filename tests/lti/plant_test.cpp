#include "dynamical/lti/plant.hpp"

#include "dynamical/lti/analysis.hpp"
#include "dynamical/utils/test_utils.hpp"

#include <iostream>

#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace dynamical_testing {

TEST(Plant, DimensionCheck) {
  constexpr int num_states = 3, num_inputs = 2, num_outputs = num_states;
  using SimplePlant = dynamical::lti::sim::DiscretePlant<num_states, num_inputs,
                                                         num_outputs, double>;
  SimplePlant plant(
      SimplePlant::x_VectorType::Random(), SimplePlant::A_MatrixType::Random(),
      SimplePlant::B_MatrixType::Random(), SimplePlant::C_MatrixType::Random(),
      SimplePlant::D_MatrixType::Random());

  ASSERT_EQ(num_states, plant.GetX().rows());
  ASSERT_EQ(1, plant.GetX().cols());
  ASSERT_EQ(num_outputs, plant.GetY().rows());
  ASSERT_EQ(1, plant.GetY().cols());
  ASSERT_EQ(num_states, plant.A_.rows());
  ASSERT_EQ(num_states, plant.A_.cols());
  ASSERT_EQ(num_states, plant.B_.rows());
  ASSERT_EQ(num_inputs, plant.B_.cols());
  ASSERT_EQ(num_outputs, plant.C_.rows());
  ASSERT_EQ(num_states, plant.C_.cols());
  ASSERT_EQ(num_outputs, plant.D_.rows());
  ASSERT_EQ(num_inputs, plant.D_.cols());
}

TEST(Plant, DefaultArgumentCheck) {
  constexpr int num_states = 3, num_inputs = 2;  // num_outputs = num_states
  using SimplePlant =
      dynamical::lti::sim::DiscretePlant<num_states, num_inputs>;
  SimplePlant plant(SimplePlant::x_VectorType::Random(),
                    SimplePlant::A_MatrixType::Random(),
                    SimplePlant::B_MatrixType::Random());

  ASSERT_EQ(SimplePlant::C_MatrixType::Identity(), plant.C_);
  ASSERT_EQ(SimplePlant::D_MatrixType::Zero(), plant.D_);
}

TEST(Plant, PropagateDiscreteDynamics) {
  // Example 2: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using SISOPlant =
      dynamical::lti::sim::DiscretePlant<num_states, num_inputs, num_outputs>;

  // Explicitly define A and B matrices for system that we know is controllable
  SISOPlant::A_MatrixType test_A;
  test_A << /*[[*/ 1, 1 /*]*/,
      /*[*/ 0, 2 /*]]*/;
  SISOPlant::B_MatrixType test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ 1 /*]]*/;

  // Test a couple of random initial state / target state pairs.
  for (int i = 0; i != 20; ++i) {
    SISOPlant::x_VectorType x_initial = SISOPlant::x_VectorType::Random();
    SISOPlant plant(x_initial, test_A, test_B);

    SISOPlant::x_VectorType x_target = SISOPlant::x_VectorType::Random();
    Eigen::MatrixXd controllability_matrix =
        dynamical::lti::analysis::get_controllability_matrix(plant);

    // calculate a two-step input sequence that should reach the target state
    Eigen::Vector2d inverted_input_sequence =
        controllability_matrix.inverse() *
        (x_target - (plant.A_ * plant.A_ * x_initial));

    ASSERT_TRUE(
        dynamical_test_utils::check_matrix_equality(x_initial, plant.GetX()));

    plant.Update(inverted_input_sequence.row(1));
    plant.Update(inverted_input_sequence.row(0));

    ASSERT_TRUE(
        dynamical_test_utils::check_matrix_equality(x_target, plant.GetX()));

    // std::cout << "===============\n";
    // std::cout << "Test #" << i << '\n';
    // std::cout << "\nX Initial: \n" << x_initial << '\n';
    // std::cout << "\nX Target: \n" << x_target << '\n';
    // std::cout << "\nControllability: \n" << controllability_matrix << '\n';
    // std::cout << "\nInput Sequence: \n" << inverted_input_sequence << '\n';
    // std::cout << "\nX Actual: \n" << plant.GetX() << "\n\n";
  }
}

// Note: this is doubling as a proxy test for
// dynamical::numerical::integral:rk4().
TEST(Plant, PropagateContinuousDynamics) {
  // Compare the accuracy of runge-kutta against a model that was discretized by
  // diagonalization.

  constexpr int num_states = 3, num_inputs = 1, num_outputs = 3;
  using ContinuousPlant =
      dynamical::lti::sim::ContinuousPlant<num_states, num_inputs, num_outputs>;
  using DiscretePlant =
      dynamical::lti::sim::DiscretePlant<num_states, num_inputs, num_outputs,
                                         std::complex<double>>;

  for (int i = 0; i != 50; ++i) {
    ContinuousPlant continuous_plant(ContinuousPlant::x_VectorType::Random(),
                                     ContinuousPlant::A_MatrixType::Random(),
                                     ContinuousPlant::B_MatrixType::Random());

    constexpr double dt = 0.01;

    DiscretePlant discrete_plant =
        dynamical::lti::analysis::discretize(continuous_plant, dt);

    for (int step = 0; step != 1000; ++step) {
      ContinuousPlant::u_VectorType u_cont =
          ContinuousPlant::u_VectorType::Random();
      continuous_plant.Update(u_cont, dt);
      discrete_plant.Update(u_cont);
    }

    // big tolerance because we've taken 1000 steps lol
    ASSERT_TRUE(dynamical_test_utils::check_matrix_equality(
        discrete_plant.GetX(), continuous_plant.GetX(), 1e-2));
  }
}

// TODO: figure out scheme to actually evaluate this instead of just reading
// output
TEST(Plant, GaussianNoise) {
  // Observe the effects of the gaussian noise generator on a static plant.

  constexpr int num_states = 3, num_inputs = 1;
  using SimplePlant =
      dynamical::lti::sim::DiscretePlant<num_states, num_inputs>;

  SimplePlant::x_VectorType x;
  x << 1, 1, 0;  // we should see the output have this column vector as the mean
  SimplePlant plant(x, SimplePlant::A_MatrixType::Identity(),
                    SimplePlant::B_MatrixType::Zero());

  std::cout << "=============\nrealistic noise\n\n";
  plant.EnableOutputNoise(0.1);
  for (int i = 0; i != 20; ++i) {
    plant.Update(SimplePlant::u_VectorType::Zero());
    std::cout << plant.GetY() << "\n\n";
  }

  std::cout << "=============\na lot of noise\n\n";
  plant.EnableOutputNoise(1);
  for (int i = 0; i != 10; ++i) {
    plant.Update(SimplePlant::u_VectorType::Zero());
    std::cout << plant.GetY() << "\n\n";
  }

  std::cout << "=============\nno noise\n\n";
  plant.DisableNoise();
  for (int i = 0; i != 10; ++i) {
    plant.Update(SimplePlant::u_VectorType::Zero());
    std::cout << plant.GetY() << "\n\n";
  }
}

}  // namespace dynamical_testing
