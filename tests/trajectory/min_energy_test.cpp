#include "dynamical/trajectory/min_energy.hpp"

#include <iostream>

#include "Eigen/Dense"
#include "gtest/gtest.h"

#include "dynamical/lti/analysis.hpp"
#include "dynamical/lti/plant.hpp"
#include "dynamical/utils/test_utils.hpp"

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

// This test gives more insight on the effectiveness of the current minimum
// energy trajectory generation method.
//
// From inspecting the output, it seems that when the system matrices are small
// in magnitude, 9 times out of 10, the min energy path effectively gets the
// simulated plant to the target. There are several cases where the plant gets
// close but doesn't quite reach the target (could be corrected easily by
// feedback). There are also a few cases where the plant goes way off (largest
// error I've seen was 500%).
//
// However, when the matrices are large in magnitude (more variance?), the min
// energy path almost always leads the plant WAY off (on several orders of
// magnitude!).
//
// Increasing the number of steps also seems to increase the chances of
// generating a singular (C * C^T).
//
// TODO: figure out what causes these errors in min energy path creation.
TEST(Trajectory, MinEnergy_Random) {
  // Test a variety of (controllable) random MIMO systems with varying number of
  // allowed steps.

  constexpr int num_states = 3, num_inputs = 1;
  using SimplePlant =
      dynamical::lti::sim::DiscretePlant<num_states, num_inputs>;
  using MinEnergy = dynamical::trajectory::MinEnergy<num_states, num_inputs>;

  int controllable_systems_generated = 0, total_systems_generated = 50;

  int num_steps = 10;

  for (int i = 0; i != total_systems_generated; ++i) {
    std::cout << "========================\n";
    std::cout << "min energy test #" << i << ":\n\n";

    SimplePlant::A_MatrixType A = SimplePlant::A_MatrixType::Random();
    SimplePlant::B_MatrixType B = SimplePlant::B_MatrixType::Random();
    SimplePlant::x_VectorType x_initial = SimplePlant::x_VectorType::Random();

    std::cout << "A:\n" << A << "\n\n";
    std::cout << "B:\n" << B << "\n\n";
    std::cout << "x_initial:\n" << x_initial << "\n\n";

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

      // std::cout << "step #" << step << ":\n";
      // std::cout << "input:\n" << u << "\n\n";
      // std::cout << "state:\n" << plant.GetX() << "\n\n";
    }
    std::cout << "end state:\n" << plant.GetX() << "\n\n";
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
