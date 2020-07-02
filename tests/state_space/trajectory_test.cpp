#include "dynamical/state_space/trajectory.hpp"
#include "dynamical/state_space/analysis.hpp"
#include "dynamical/state_space/plant.hpp"
#include "dynamical/utils/test_utils.hpp"

#include <iostream>

#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace dynamical_testing {

TEST(Trajectory, MinEnergy_Simple) {
  // Problem 2: https://inst.eecs.berkeley.edu/~ee16b/sp20/homework/prob10.pdf
  constexpr int num_states = 2, num_inputs = 1;
  using SimplePlant =
      dynamical::lti::sim::DiscretePlant<num_states, num_inputs>;
  using MinEnergy = dynamical::trajectory::MinEnergy<num_states, num_inputs>;

  SimplePlant::A_MatrixType A;
  A << /*[[*/ 1, 1 /*]*/,
      /*[*/ 0, 1 /*]]*/;
  SimplePlant::B_MatrixType B;
  B << /*[[*/ 0 /*]*/,
      /*[*/ 1 /*]]*/;
  SimplePlant::x_VectorType x_initial = SimplePlant::x_VectorType::Zero();

  SimplePlant plant(x_initial, A, B);

  constexpr int num_steps = 5;
  MinEnergy::ControllabilityMatrixType controllability_matrix =
      dynamical::lti::analysis::get_controllability_matrix(plant, num_steps);
  MinEnergy::x_VectorType target_state;
  target_state << /*[[*/ 1 /*]*/,
      /*[*/ 0 /*]]*/;

  std::cout << controllability_matrix << "\n\n";

  MinEnergy trajectory(controllability_matrix, num_steps, target_state);

  for (int step = 0; step != 5; ++step) {
    MinEnergy::u_VectorType u = trajectory.GetFeedforward(step);
    plant.Update(u);

    std::cout << "step #" << step << ":\n";
    std::cout << "input:\n" << u << "\n\n";
    std::cout << "state:\n" << plant.GetX() << "\n\n";
  }

  ASSERT_TRUE(
      dynamical_test_utils::check_matrix_equality(target_state, plant.GetX()));
}

}  // namespace dynamical_testing
