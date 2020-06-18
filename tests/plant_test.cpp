#include "dynamical/state_space/plant.h"

#include "Eigen/Dense"
#include "gtest/gtest.h"

TEST(PlantTest, DimensionCheck) {
  constexpr int num_states = 3, num_inputs = 1, num_outputs = num_states;
  using SimplePlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs, double>;
  SimplePlant plant(SimplePlant::x_vector_type::Random(),
                    SimplePlant::A_matrix_type::Random(),
                    SimplePlant::B_matrix_type::Random(),
                    SimplePlant::C_matrix_type::Random(),
                    SimplePlant::D_matrix_type::Random());

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
  constexpr int num_states = 3, num_inputs = 1;  // num_outputs = num_states
  using SimplePlant = dynamical::DiscretePlant<num_states, num_inputs>;
  SimplePlant plant(SimplePlant::x_vector_type::Random(),
                    SimplePlant::A_matrix_type::Random(),
                    SimplePlant::B_matrix_type::Random());

  ASSERT_EQ(SimplePlant::C_matrix_type::Identity(), plant.C_);
  ASSERT_EQ(SimplePlant::D_matrix_type::Zero(), plant.D_);
}

// TEST(PlantTest, PropagateDiscreteDynamics) {
//   constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
//   using SISOPlant =
//       dynamical::DiscretePlant<num_states, num_inputs, num_states>;

//   // need to explicitly define A and B to ensure reachability
//   SISOPlant::A_matrix_type test_A;
//   test_A << /*[[*/ 1, 1 /*]*/,
//       /*[*/ 0, 2 /*]]*/;
//   SISOPlant::B_matrix_type test_B;
//   test_B << /*[[*/ 0 /*]*/,
//       /*[*/ 1 /*]]*/;
//   SISOPlant::x_vector_type test_initial_x =
//   SISOPlant::x_vector_type::Random(); SISOPlant plant(test_initial_x, test_A,
//   test_B);

//   SISOPlant::x_vector_type target_x = SISOPlant::x_vector_type::Random() * 3;
//   Eigen::Matrix2d reachability_matrix;
//   reachability_matrix.col(0) = test_B;
//   reachability_matrix.col(1) = test_A * test_B;

//   ASSERT_EQ(test_initial_x, plant.GetX());
// }
