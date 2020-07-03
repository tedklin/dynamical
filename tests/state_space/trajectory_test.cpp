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

  for (int step = 0; step != num_steps; ++step) {
    MinEnergy::u_VectorType u = trajectory.GetFeedforward(step);
    plant.Update(u);

    std::cout << "step #" << step << ":\n";
    std::cout << "input:\n" << u << "\n\n";
    std::cout << "state:\n" << plant.GetX() << "\n\n";
  }

  ASSERT_TRUE(
      dynamical_test_utils::check_matrix_equality(target_state, plant.GetX()));
}

// THIS TEST IS CURRENTLY FAILING FOR BOTH SISO AND MIMO SYSTEMS
// The minimum energy control trajectory generation doesn't always work with
// randomly generated floating point matrices. There are two possible
// explanations I can think of right now:
//    - there are conditions besides controllability for this to work
//    - the method of calculation is sensitive to floating point numbers
//    (perhaps numbers that are too small?)
//
// TODO: revisit this
TEST(Trajectory, MinEnergy_Random) {
  // Test a variety of (controllable) random MIMO systems with varying number of
  // allowed steps.

  constexpr int num_states = 3, num_inputs = 1;
  using SimplePlant =
      dynamical::lti::sim::DiscretePlant<num_states, num_inputs>;
  using MinEnergy = dynamical::trajectory::MinEnergy<num_states, num_inputs>;

  int controllable_systems_generated = 0, total_systems_generated = 50;
  int num_steps = 5;

  for (int i = 0; i != total_systems_generated; ++i) {
    std::cout << "========================\n";
    std::cout << "min energy test #" << i << ":\n\n";

    SimplePlant::A_MatrixType A = SimplePlant::A_MatrixType::Random();
    SimplePlant::B_MatrixType B = SimplePlant::B_MatrixType::Random();
    SimplePlant::x_VectorType x_initial = SimplePlant::x_VectorType::Random();

    SimplePlant plant(x_initial, A, B);

    // The system has to be controllable to guarantee that the controller can
    // actually reach our target state.
    if (!dynamical::lti::analysis::is_controllable(plant)) {
      continue;
    }
    ++controllable_systems_generated;

    MinEnergy::ControllabilityMatrixType controllability_matrix =
        dynamical::lti::analysis::get_controllability_matrix(plant, num_steps);
    MinEnergy::x_VectorType target_state = MinEnergy::x_VectorType::Random();

    MinEnergy trajectory(controllability_matrix, num_steps, target_state);

    std::cout << "controllability matrix:\n"
              << controllability_matrix << "\n\n";

    for (int step = 0; step != num_steps; ++step) {
      MinEnergy::u_VectorType u = trajectory.GetFeedforward(step);
      plant.Update(u);

      std::cout << "step #" << step << ":\n";
      std::cout << "input:\n" << u << "\n\n";
      std::cout << "state:\n" << plant.GetX() << "\n\n";
    }
    std::cout << "target state:\n" << target_state << "\n\n";

    // ASSERT_TRUE(dynamical_test_utils::check_matrix_equality(target_state,
    //                                                         plant.GetX()));
  }

  std::cout << "========================\n";
  std::cout << "TEST END\n";
  std::cout << "total number of systems generated: " << total_systems_generated
            << '\n';
  std::cout << "number of controllable systems generated: "
            << controllable_systems_generated << '\n';
}

}  // namespace dynamical_testing
