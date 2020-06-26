#include "dynamical/state_space/analysis.hpp"

#include "dynamical/state_space/plant.hpp"
#include "dynamical/test_utils.hpp"

#include <cmath>
#include <iostream>

#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace testing {

/* ============================= */
/* === CONTROLLABILITY TESTS === */
/* ============================= */

// TODO: fuzz controllability matrix tests?
TEST(Controllability, SISO_ControllabilityMatrix) {
  // example 2: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

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

  ASSERT_TRUE(test_utils::check_matrix_equality(
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

  ASSERT_TRUE(test_utils::check_matrix_equality(
      manual_controllability_matrix, calculated_controllability_matrix));
}

TEST(Controllability, SISO_Controllable) {
  // example 2: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

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
  // example 3: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

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
  // the probability of getting a random uncontrollable plant with the two
  // states and two inputs should be very small. one matrix would have to be
  // something like the identity, and the other would have to be single-rank.

  constexpr int num_states = 2, num_inputs = 2, num_outputs = 2;
  using MIMOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  MIMOPlant::A_MatrixType test_A = MIMOPlant::A_MatrixType::Random();
  MIMOPlant::B_MatrixType test_B = MIMOPlant::B_MatrixType::Random();
  MIMOPlant::x_VectorType x_initial = MIMOPlant::x_VectorType::Random();

  MIMOPlant plant(x_initial, test_A, test_B);

  ASSERT_TRUE(dynamical::analysis::is_controllable(plant));
}

TEST(Controllability, MIMO_Uncontrollable) {
  // random numbers, checked by hand

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

/* ======================= */
/* === STABILITY TESTS === */
/* ======================= */

TEST(Stability, Discrete_Unstable) {
  // random numbers, checked by hand

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
  // random numbers, checked by hand

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
  // problem 4: https://inst.eecs.berkeley.edu/~ee16b/sp20/homework/prob11.pdf

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
  // problem 4: https://inst.eecs.berkeley.edu/~ee16b/sp20/homework/prob11.pdf

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

// TODO: implement this
TEST(Stability, Discrete_Fuzzed_Sim) {
  // multiple tests with random matrices, comparing the result of is_stable
  // function against simulations with extended number of steps.
}

// TODO: before implementing this, test
// plant_test.cpp/PropagateContinuousDynamics
TEST(Stability, Continuous_Fuzzed_Sim) {
  // multiple tests with random matrices, comparing the result of is_stable
  // function against simulations with extended number of steps.
}

/* ============================ */
/* === DISCRETIZATION TESTS === */
/* ============================ */

TEST(Discretization, Dynamics_NoInput) {
  // example 1: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

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

    ASSERT_TRUE(test_utils::check_complex_matrix_equality(expected_A,
                                                          discrete_plant.A_));
  }
}

TEST(Discretization, Dynamics_NoInput_Fuzzed) {
  // this test takes advantage of the property that eigenvalues of a continuous
  // system always end up in the exponent of its discretized version.

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using ContinuousPlant =
      dynamical::ContinuousPlant<num_states, num_inputs, num_outputs>;
  using DiscretePlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs,
                               std::complex<double>>;

  for (int i = 0; i != 20; ++i) {
    ContinuousPlant::A_MatrixType test_A =
        ContinuousPlant::A_MatrixType::Random();
    ContinuousPlant::B_MatrixType test_B =
        ContinuousPlant::B_MatrixType::Zero();
    ContinuousPlant::x_VectorType x_initial =
        ContinuousPlant::x_VectorType::Random();

    ContinuousPlant continuous_plant(x_initial, test_A, test_B);
    ContinuousPlant continuous_plant_doubled(x_initial, test_A * 2, test_B);

    for (double dt = 2; dt > 0.001; dt /= 2) {
      DiscretePlant discrete_plant =
          dynamical::analysis::discretize(continuous_plant, dt);
      DiscretePlant discrete_plant_doubled =
          dynamical::analysis::discretize(continuous_plant_doubled, dt);

      ASSERT_TRUE(test_utils::check_complex_matrix_equality(
          discrete_plant_doubled.A_, discrete_plant.A_ * discrete_plant.A_));
    }
  }
}

// TODO: finish this
TEST(Discretization, SISO) {
  // problem 2: https://inst.eecs.berkeley.edu/~ee16b/sp20/homework/prob12.pdf

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

  DiscretePlant discrete_plant =
      dynamical::analysis::discretize(continuous_plant, 0.01);
}

// TODO: implement this
TEST(Discretization, MIMO) {}

// TODO: compare discretization against the parent continuous system's Update
// function?
TEST(Discretization, Sim_Fuzzed) {}

}  // namespace testing
