#include "dynamical/state_space/analysis.hpp"

#include "dynamical/state_space/plant.hpp"
#include "dynamical/utils/test_utils.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace testing {

/* ============================= */
/* === CONTROLLABILITY TESTS === */
/* ============================= */

TEST(Controllability, SISO_ControllabilityMatrix) {
  // Example 2: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using SISOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  SISOPlant::A_MatrixType test_A;
  test_A << /*[[*/ 1, 1 /*]*/,
      /*[*/ 0, 2 /*]]*/;

  SISOPlant::B_MatrixType test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ 1 /*]]*/;

  SISOPlant::x_VectorType x_initial = SISOPlant::x_VectorType::Random();

  SISOPlant plant(x_initial, test_A, test_B);

  Eigen::MatrixXd calculated_controllability_matrix =
      dynamical::analysis::get_controllability_matrix(plant);

  Eigen::MatrixXd manual_controllability_matrix(2, 2);
  manual_controllability_matrix << /*[[*/ 0, 1 /*]*/,
      /*[*/ 1, 2 /*]]*/;

  ASSERT_TRUE(dynamical_test_utils::check_matrix_equality(
      manual_controllability_matrix, calculated_controllability_matrix));
}

TEST(Controllability, MIMO_ControllabilityMatrix) {
  // MIMO extension of SISO example above, checked by hand

  constexpr int num_states = 2, num_inputs = 2, num_outputs = 2;
  using MIMOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  MIMOPlant::A_MatrixType test_A;
  test_A << /*[[*/ 1, 1 /*]*/,
      /*[*/ 0, 2 /*]]*/;

  MIMOPlant::B_MatrixType test_B;
  test_B << /*[[*/ 0, 1 /*]*/,
      /*[*/ 1, 1 /*]]*/;

  MIMOPlant::x_VectorType x_initial = MIMOPlant::x_VectorType::Random();

  MIMOPlant plant(x_initial, test_A, test_B);

  Eigen::MatrixXd calculated_controllability_matrix =
      dynamical::analysis::get_controllability_matrix(plant);

  Eigen::MatrixXd manual_controllability_matrix(2, 4);
  manual_controllability_matrix << /*[[*/ 0, 1, 1, 2 /*]*/,
      /*[*/ 1, 1, 2, 2 /*]]*/;

  ASSERT_TRUE(dynamical_test_utils::check_matrix_equality(
      manual_controllability_matrix, calculated_controllability_matrix));
}

TEST(Controllability, SISO_Controllable) {
  // Example 2: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using SISOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  SISOPlant::A_MatrixType test_A;
  test_A << /*[[*/ 1, 1 /*]*/,
      /*[*/ 0, 2 /*]]*/;

  SISOPlant::B_MatrixType test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ 1 /*]]*/;

  SISOPlant::x_VectorType x_initial = SISOPlant::x_VectorType::Random();

  SISOPlant plant(x_initial, test_A, test_B);

  ASSERT_TRUE(dynamical::analysis::is_controllable(plant));
}

TEST(Controllability, SISO_Uncontrollable) {
  // Example 3: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using SISOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  SISOPlant::A_MatrixType test_A;
  test_A << /*[[*/ 1, 1 /*]*/,
      /*[*/ 0, 2 /*]]*/;

  SISOPlant::B_MatrixType test_B;
  test_B << /*[[*/ 1 /*]*/,
      /*[*/ 0 /*]]*/;

  SISOPlant::x_VectorType x_initial = SISOPlant::x_VectorType::Random();

  SISOPlant plant(x_initial, test_A, test_B);

  ASSERT_FALSE(dynamical::analysis::is_controllable(plant));
}

TEST(Controllability, MIMO_Controllable) {
  // The probability of getting a random uncontrollable plant with the two
  // states and two inputs should be very small. One matrix would have to be
  // something like the identity, and the other would have to be single-rank.

  constexpr int num_states = 2, num_inputs = 2, num_outputs = 2;
  using MIMOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  MIMOPlant::A_MatrixType random_A = MIMOPlant::A_MatrixType::Random();
  MIMOPlant::B_MatrixType random_B = MIMOPlant::B_MatrixType::Random();
  MIMOPlant::x_VectorType x_initial = MIMOPlant::x_VectorType::Random();

  MIMOPlant plant(x_initial, random_A, random_B);

  ASSERT_TRUE(dynamical::analysis::is_controllable(plant));
}

TEST(Controllability, MIMO_Uncontrollable) {
  // Random numbers, checked by hand.

  constexpr int num_states = 2, num_inputs = 2, num_outputs = 2;
  using MIMOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  MIMOPlant::A_MatrixType test_A;
  test_A << /*[[*/ 1, 0 /*]*/,
      /*[*/ 0, 1 /*]]*/;

  MIMOPlant::B_MatrixType test_B;
  test_B << /*[[*/ 1, 2 /*]*/,
      /*[*/ 2, 4 /*]]*/;

  MIMOPlant::x_VectorType x_initial = MIMOPlant::x_VectorType::Random();

  MIMOPlant plant(x_initial, test_A, test_B);

  ASSERT_FALSE(dynamical::analysis::is_controllable(plant));
}

/* =========================== */
/* === OBSERVABILITY TESTS === */
/* =========================== */

// TODO: test get_observability_matrix and is_observable

/* ======================= */
/* === STABILITY TESTS === */
/* ======================= */

TEST(Stability, Discrete_Unstable) {
  // Random numbers, checked by hand.

  constexpr int num_states = 3, num_inputs = 2;  // num_outputs = 3
  using DiscretePlant = dynamical::DiscretePlant<num_states, num_inputs>;

  DiscretePlant::A_MatrixType test_A;
  test_A << /*[[*/ 1, 0.8, 0.7 /*]*/,
      /*[*/ 0.7, 0.9, 1 /*]*/,
      /*[*/ 0.9, 0.8, 0.8 /*]]*/;

  DiscretePlant::B_MatrixType test_B;
  test_B << /*[[*/ 1, 1 /*]*/,
      /*[*/ 1, 1 /*]*/,
      /*[*/ 1, 0 /*]]*/;

  DiscretePlant::x_VectorType x_initial = DiscretePlant::x_VectorType::Random();

  DiscretePlant plant(x_initial, test_A, test_B);

  ASSERT_FALSE(dynamical::analysis::is_stable(plant));
}

TEST(Stability, Discrete_Stable) {
  // Random numbers, checked by hand.

  constexpr int num_states = 3, num_inputs = 2;  // num_outputs = 3
  using DiscretePlant = dynamical::DiscretePlant<num_states, num_inputs>;
  using Feedback = dynamical::Feedback<num_states, num_inputs>;

  DiscretePlant::A_MatrixType test_A;
  test_A << /*[[*/ 1, 0.8, 0.7 /*]*/,
      /*[*/ 0.7, 0.9, 1 /*]*/,
      /*[*/ 0.9, 0.8, 0.8 /*]]*/;

  DiscretePlant::B_MatrixType test_B;
  test_B << /*[[*/ 1, 1 /*]*/,
      /*[*/ 1, 1 /*]*/,
      /*[*/ 1, 0 /*]]*/;

  Feedback::K_MatrixType test_K;
  test_K << /*[[*/ -0.5, -0.2, -0.3 /*]*/,
      /*[*/ -0.3, -0.6, -0.5 /*]]*/;

  DiscretePlant::x_VectorType x_initial = DiscretePlant::x_VectorType::Random();

  DiscretePlant plant(x_initial, test_A, test_B);
  Feedback feedback(test_K);

  ASSERT_TRUE(dynamical::analysis::is_stable(plant, feedback));
}

TEST(Stability, Continuous_Unstable) {
  // Problem 4: https://inst.eecs.berkeley.edu/~ee16b/sp20/homework/prob11.pdf

  constexpr int num_states = 3, num_inputs = 1;  // num_outputs = 3
  using ContinuousPlant = dynamical::ContinuousPlant<num_states, num_inputs>;

  ContinuousPlant::A_MatrixType test_A;
  test_A << /*[[*/ 0, 1, 0 /*]*/,
      /*[*/ 100, 0, 0 /*]*/,
      /*[*/ 0, 0, 0 /*]]*/;

  ContinuousPlant::B_MatrixType test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ -25 /*]*/,
      /*[*/ 500 /*]]*/;

  ContinuousPlant::x_VectorType x_initial =
      ContinuousPlant::x_VectorType::Random();

  ContinuousPlant plant(x_initial, test_A, test_B);

  ASSERT_FALSE(dynamical::analysis::is_stable(plant));
}

TEST(Stability, Continuous_Stable) {
  // Problem 4: https://inst.eecs.berkeley.edu/~ee16b/sp20/homework/prob11.pdf

  constexpr int num_states = 3, num_inputs = 1;  // num_outputs = 3
  using ContinuousPlant = dynamical::ContinuousPlant<num_states, num_inputs>;
  using Feedback = dynamical::Feedback<num_states, num_inputs>;

  ContinuousPlant::A_MatrixType test_A;
  test_A << /*[[*/ 0, 1, 0 /*]*/,
      /*[*/ 100, 0, 0 /*]*/,
      /*[*/ 0, 0, 0 /*]]*/;

  ContinuousPlant::B_MatrixType test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ -25 /*]*/,
      /*[*/ 500 /*]]*/;

  Feedback::K_MatrixType test_K;
  test_K << 20, 5, 0.01;

  ContinuousPlant::x_VectorType x_initial =
      ContinuousPlant::x_VectorType::Random();

  ContinuousPlant plant(x_initial, test_A, test_B);
  Feedback feedback(test_K);

  ASSERT_TRUE(dynamical::analysis::is_stable(plant, feedback));
}

TEST(Stability, Discrete_Fuzzed_Sim) {
  // Multiple tests with discrete systems generated from random matrices.
  // The result of the is_stable function is compared against simulations with
  // extended number of steps.

  // This was only done with discrete systems because I haven't found a good way
  // to *guarantee* randomly generated A, B, and K matrices always giving us
  // nice continuous systems (eigenvalues with high norms).

  constexpr int num_states = 3, num_inputs = 2;
  using DiscretePlant = dynamical::DiscretePlant<num_states, num_inputs>;
  using Feedback = dynamical::Feedback<num_states, num_inputs>;

  int stable_systems_generated = 0, controllable_systems_generated = 0;

  for (int i = 0; i != 50; ++i) {
    std::cout << "========================\n";
    std::cout << "discrete stability test #" << i << ":\n\n";

    // Eigen's random generates floating point numbers in the range (-1.0, 1.0).
    // To balance the chances of generating a stable discrete system, we make
    // the plant matrices strictly positive and the feedback matrix strictly
    // negative.
    DiscretePlant::A_MatrixType random_A =
        DiscretePlant::A_MatrixType::Random().cwiseAbs() * 1.2;
    DiscretePlant::B_MatrixType random_B =
        DiscretePlant::B_MatrixType::Random().cwiseAbs() * 1.2;

    std::cout << "A:\n" << random_A << "\n\n";
    std::cout << "B:\n" << random_B << "\n\n";

    // The difference between the initial condition and our target state (in
    // this case, zero) needs to be large enough to show clear divergence but
    // small enough to not hit infinite values.
    DiscretePlant::x_VectorType x_initial, ref;
    x_initial << 5, 5, 5;
    DiscretePlant plant(x_initial, random_A, random_B);

    // The system has to be controllable to guarantee that the controller can
    // actually reach our target state.
    if (!dynamical::analysis::is_controllable(plant)) {
      std::cout << "uncontrollable system, skipping to next case...\n\n";
      continue;
    }
    std::cout << "controllable system, continuing...\n";
    ++controllable_systems_generated;

    Feedback::K_MatrixType random_K =
        Feedback::K_MatrixType::Random().cwiseAbs() * -1;
    Feedback feedback(random_K);

    std::cout << "K:\n" << random_K << "\n\n";

    bool is_system_stable = dynamical::analysis::is_stable(plant, feedback);
    if (is_system_stable) {
      std::cout << "stable system determined, running sim...\n";
      ++stable_systems_generated;  // avoiding adding bools (unsigned) to ints
    } else {
      std::cout << "unstable system determined, running sim...\n";
    }

    for (int step = 0; step != 1000; ++step) {
      plant.Update(feedback.GetU(plant.GetX()));
      //   std::cout << "plant state at step " << step << ":\n"
      //             << plant.GetX() << '\n';
    }

    std::cout << "plant state after 1000 steps:\n";
    std::cout << plant.GetX() << "\n\n";

    ASSERT_EQ(is_system_stable,
              dynamical_test_utils::check_matrix_equality(
                  DiscretePlant::x_VectorType::Zero(), plant.GetX(), 0.01));
  }

  std::cout << "========================\n";
  std::cout << "TEST END\n";
  std::cout << "number of controllable systems generated: "
            << controllable_systems_generated << '\n';
  std::cout << "number of stable systems generated: "
            << stable_systems_generated << "\n\n";
}

/* ============================ */
/* === DISCRETIZATION TESTS === */
/* ============================ */

TEST(Discretization, Dynamics_NoInput) {
  // Example 1: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using ContinuousPlant =
      dynamical::ContinuousPlant<num_states, num_inputs, num_outputs>;
  using DiscretePlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs,
                               std::complex<double>>;

  ContinuousPlant::A_MatrixType test_A;
  test_A << /*[[*/ 0, -1 /*]*/,
      /*[*/ 1, 0 /*]]*/;

  ContinuousPlant::B_MatrixType test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ 0 /*]]*/;

  ContinuousPlant::x_VectorType x_initial =
      ContinuousPlant::x_VectorType::Random();

  ContinuousPlant continuous_plant(x_initial, test_A, test_B);

  // making sure it works for all kinds of values of dt
  for (double dt = 2; dt > 0.001; dt /= 2) {
    DiscretePlant discrete_plant =
        dynamical::analysis::discretize(continuous_plant, dt);

    DiscretePlant::A_MatrixType expected_A;
    expected_A << /*[[*/ std::cos(dt), -std::sin(dt) /*]*/,
        /*[*/ std::sin(dt), std::cos(dt) /*]]*/;

    ASSERT_TRUE(dynamical_test_utils::check_matrix_equality(expected_A,
                                                            discrete_plant.A_));
  }
}

TEST(Discretization, Dynamics_NoInput_Fuzzed) {
  // This test takes advantage of the property that eigenvalues of a continuous
  // system always end up in the exponent of its discretized version.

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using ContinuousPlant =
      dynamical::ContinuousPlant<num_states, num_inputs, num_outputs>;
  using DiscretePlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs,
                               std::complex<double>>;

  for (int i = 0; i != 20; ++i) {
    ContinuousPlant::A_MatrixType random_A =
        ContinuousPlant::A_MatrixType::Random();
    ContinuousPlant::B_MatrixType zero_B =
        ContinuousPlant::B_MatrixType::Zero();
    ContinuousPlant::x_VectorType x_initial =
        ContinuousPlant::x_VectorType::Random();

    ContinuousPlant continuous_plant(x_initial, random_A, zero_B);
    ContinuousPlant continuous_plant_doubled(x_initial, random_A * 2, zero_B);

    // making sure it works for all kinds of values of dt
    for (double dt = 2; dt > 0.001; dt /= 2) {
      DiscretePlant discrete_plant =
          dynamical::analysis::discretize(continuous_plant, dt);
      DiscretePlant discrete_plant_doubled =
          dynamical::analysis::discretize(continuous_plant_doubled, dt);

      ASSERT_TRUE(dynamical_test_utils::check_matrix_equality(
          discrete_plant_doubled.A_, discrete_plant.A_ * discrete_plant.A_,
          1e-5));
    }
  }
}

TEST(Discretization, SimpleSecondOrder) {
  // An example of a system that can't be solved by diagonlization.
  // https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/12a.pdf

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 2;
  using ContinuousPlant =
      dynamical::ContinuousPlant<num_states, num_inputs, num_outputs>;
  using DiscretePlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs,
                               std::complex<double>>;

  ContinuousPlant::A_MatrixType test_A;
  test_A << /*[[*/ 0, 1 /*]*/,
      /*[*/ 0, 0 /*]]*/;

  constexpr double R = 9.71, M = 1.678;
  ContinuousPlant::B_MatrixType test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ 1.0 / R * M /*]]*/;

  ContinuousPlant::x_VectorType x_initial =
      ContinuousPlant::x_VectorType::Random();

  ContinuousPlant continuous_plant(x_initial, test_A, test_B);

  ASSERT_THROW(dynamical::analysis::discretize(continuous_plant, 0.01),
               std::runtime_error);
}

TEST(Discretization, SISO) {
  // Problem 2: https://inst.eecs.berkeley.edu/~ee16b/sp20/homework/prob12.pdf
  // Checked by hand.

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using ContinuousPlant =
      dynamical::ContinuousPlant<num_states, num_inputs, num_outputs>;
  using DiscretePlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs,
                               std::complex<double>>;

  ContinuousPlant::A_MatrixType test_A;
  test_A << /*[[*/ 0, 1 /*]*/,
      /*[*/ -2, -3 /*]]*/;

  ContinuousPlant::B_MatrixType test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ 2 /*]]*/;

  ContinuousPlant::x_VectorType x_initial =
      ContinuousPlant::x_VectorType::Random();

  ContinuousPlant continuous_plant(x_initial, test_A, test_B);

  DiscretePlant::A_MatrixType expected_A;
  expected_A << /*[[*/ 0.99094, 0.08610 /*]*/,
      /*[*/ -0.17221, 0.73262 /*]]*/;

  DiscretePlant::B_MatrixType expected_B;
  test_B << /*[[*/ 0.00905 /*]*/,
      /*[*/ 0.17221 /*]]*/;

  DiscretePlant discrete_plant =
      dynamical::analysis::discretize(continuous_plant, 0.1);

  ASSERT_TRUE(dynamical_test_utils::check_matrix_equality(
      expected_A, discrete_plant.A_, 1e-5));
}

// TODO: implement this
TEST(Discretization, MIMO) {}

}  // namespace testing
