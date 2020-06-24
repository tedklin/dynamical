#include "dynamical/state_space/plant.h"

#include "dynamical/state_space/analysis.h"
#include "dynamical/test_utils.h"

#include <iostream>

#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace testing {

TEST(PlantTest, DimensionCheck) {
  constexpr int num_states = 3, num_inputs = 2, num_outputs = num_states;
  using SimplePlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs, double>;
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

TEST(PlantTest, DefaultArgumentCheck) {
  constexpr int num_states = 3, num_inputs = 2;  // num_outputs = num_states
  using SimplePlant = dynamical::DiscretePlant<num_states, num_inputs>;
  SimplePlant plant(SimplePlant::x_VectorType::Random(),
                    SimplePlant::A_MatrixType::Random(),
                    SimplePlant::B_MatrixType::Random());

  ASSERT_EQ(SimplePlant::C_MatrixType::Identity(), plant.C_);
  ASSERT_EQ(SimplePlant::D_MatrixType::Zero(), plant.D_);
}

TEST(PlantTest, PropagateDiscreteDynamics) {
  // example 2: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using SISOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  // explicitly define A and B matrices for system that we know is controllable
  SISOPlant::A_MatrixType test_A;
  test_A << /*[[*/ 1, 1 /*]*/,
      /*[*/ 0, 2 /*]]*/;
  SISOPlant::B_MatrixType test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ 1 /*]]*/;

  // test a couple of random initial state / target state pairs
  for (int fuzz_index = 0; fuzz_index != 5; ++fuzz_index) {
    SISOPlant::x_VectorType x_initial = SISOPlant::x_VectorType::Random();
    SISOPlant plant(x_initial, test_A, test_B);

    SISOPlant::x_VectorType x_target = SISOPlant::x_VectorType::Random();
    Eigen::MatrixXd controllability_matrix =
        dynamical::analysis::get_controllability_matrix(plant);

    // calculate a two-step input sequence that should reach the target state
    Eigen::Vector2d inverted_input_sequence =
        controllability_matrix.inverse() *
        (x_target - (plant.A_ * plant.A_ * x_initial));

    ASSERT_TRUE(test_utils::check_matrix_equality(x_initial, plant.GetX()));
    plant.Update(inverted_input_sequence.row(1));
    plant.Update(inverted_input_sequence.row(0));
    ASSERT_TRUE(test_utils::check_matrix_equality(x_target, plant.GetX()));

    // std::cout << "Test #" << fuzz_index << " ===========\n";
    // std::cout << "\nX Initial: \n" << x_initial << '\n';
    // std::cout << "\nX Target: \n" << x_target << '\n';
    // std::cout << "\nControllability: \n" << controllability_matrix << '\n';
    // std::cout << "\nInput Sequence: \n" << inverted_input_sequence << '\n';
    // std::cout << "\nX Actual: \n" << plant.GetX() << "\n\n";
  }
}

}  // namespace testing
