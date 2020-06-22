#include "dynamical/state_space/analysis.h"

#include "dynamical/state_space/plant.h"
#include "dynamical/test_utils.h"

#include <cmath>
#include <iostream>

#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace testing {

// TODO: parameterize and fuzz controllability matrix tests?
TEST(AnalysisTest, ControllabilityMatrixSISO) {
  // example 2: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf
  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using SISOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  SISOPlant::A_matrix_type test_A;
  test_A << /*[[*/ 1, 1 /*]*/,
      /*[*/ 0, 2 /*]]*/;
  SISOPlant::B_matrix_type test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ 1 /*]]*/;

  SISOPlant::x_vector_type x_initial = SISOPlant::x_vector_type::Random();
  SISOPlant plant(x_initial, test_A, test_B);

  Eigen::MatrixXd calculated_controllability_matrix =
      dynamical::analysis::get_controllability_matrix(plant);

  Eigen::MatrixXd manual_controllability_matrix(2, 2);
  manual_controllability_matrix << /*[[*/ 0, 1 /*]*/,
      /*[*/ 1, 2 /*]]*/;

  ASSERT_TRUE(test_utils::check_matrix_equality(
      manual_controllability_matrix, calculated_controllability_matrix));
}

TEST(AnalysisTest, ControllabilityMatrixMIMO) {
  constexpr int num_states = 2, num_inputs = 2, num_outputs = 2;
  using MIMOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  MIMOPlant::A_matrix_type test_A;
  test_A << /*[[*/ 1, 1 /*]*/,
      /*[*/ 0, 2 /*]]*/;
  MIMOPlant::B_matrix_type test_B;
  test_B << /*[[*/ 0, 1 /*]*/,
      /*[*/ 1, 1 /*]]*/;

  MIMOPlant::x_vector_type x_initial = MIMOPlant::x_vector_type::Random();
  MIMOPlant plant(x_initial, test_A, test_B);

  Eigen::MatrixXd calculated_controllability_matrix =
      dynamical::analysis::get_controllability_matrix(plant);

  Eigen::MatrixXd manual_controllability_matrix(2, 4);
  manual_controllability_matrix << /*[[*/ 0, 1, 1, 2 /*]*/,
      /*[*/ 1, 1, 2, 2 /*]]*/;

  ASSERT_TRUE(test_utils::check_matrix_equality(
      manual_controllability_matrix, calculated_controllability_matrix));
}

TEST(AnalysisTest, ControllableSISO) {
  // example 2: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using SISOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  SISOPlant::A_matrix_type test_A;
  test_A << /*[[*/ 1, 1 /*]*/,
      /*[*/ 0, 2 /*]]*/;
  SISOPlant::B_matrix_type test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ 1 /*]]*/;

  SISOPlant::x_vector_type x_initial = SISOPlant::x_vector_type::Random();
  SISOPlant plant(x_initial, test_A, test_B);

  ASSERT_TRUE(dynamical::analysis::is_controllable(plant));
}

TEST(AnalysisTest, UncontrollableSISO) {
  // example 3: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using SISOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  SISOPlant::A_matrix_type test_A;
  test_A << /*[[*/ 1, 1 /*]*/,
      /*[*/ 0, 2 /*]]*/;
  SISOPlant::B_matrix_type test_B;
  test_B << /*[[*/ 1 /*]*/,
      /*[*/ 0 /*]]*/;

  SISOPlant::x_vector_type x_initial = SISOPlant::x_vector_type::Random();
  SISOPlant plant(x_initial, test_A, test_B);

  ASSERT_FALSE(dynamical::analysis::is_controllable(plant));
}

TEST(AnalysisTest, ControllableMIMO) {
  constexpr int num_states = 2, num_inputs = 2, num_outputs = 2;
  using MIMOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  // the probability of getting a random uncontrollable plant with the two
  // states and two inputs should be very small. one matrix would have to be
  // something like the identity, and the other would have to be single-rank.
  MIMOPlant::A_matrix_type test_A = MIMOPlant::A_matrix_type::Random();
  MIMOPlant::B_matrix_type test_B = MIMOPlant::B_matrix_type::Random();

  MIMOPlant::x_vector_type x_initial = MIMOPlant::x_vector_type::Random();
  MIMOPlant plant(x_initial, test_A, test_B);

  ASSERT_TRUE(dynamical::analysis::is_controllable(plant));
}

TEST(AnalysisTest, UncontrollableMIMO) {
  constexpr int num_states = 2, num_inputs = 2, num_outputs = 2;
  using MIMOPlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs>;

  MIMOPlant::A_matrix_type test_A;
  test_A << /*[[*/ 1, 0 /*]*/,
      /*[*/ 0, 1 /*]]*/;
  MIMOPlant::B_matrix_type test_B;
  test_B << /*[[*/ 1, 2 /*]*/,
      /*[*/ 2, 4 /*]]*/;

  MIMOPlant::x_vector_type x_initial = MIMOPlant::x_vector_type::Random();
  MIMOPlant plant(x_initial, test_A, test_B);

  ASSERT_FALSE(dynamical::analysis::is_controllable(plant));
}

TEST(AnalysisTest, DiscretizationSimple) {
  // example 1: https://inst.eecs.berkeley.edu/~ee16b/sp20/lecture/8a.pdf

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using ContinuousPlant =
      dynamical::ContinuousPlant<num_states, num_inputs, num_outputs>;
  using DiscretePlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs,
                               std::complex<double>>;

  ContinuousPlant::A_matrix_type test_A;
  test_A << /*[[*/ 0, -1 /*]*/,
      /*[*/ 1, 0 /*]]*/;
  ContinuousPlant::B_matrix_type test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ 0 /*]]*/;

  ContinuousPlant::x_vector_type x_initial =
      ContinuousPlant::x_vector_type::Random();
  ContinuousPlant continuous_plant(x_initial, test_A, test_B);

  // make sure it works for different values of dt
  for (double dt = 2; dt > 0.001; dt /= 2) {
    DiscretePlant discrete_plant =
        dynamical::analysis::discretize(continuous_plant, dt);

    DiscretePlant::A_matrix_type expected_A;
    expected_A << /*[[*/ std::cos(dt), -std::sin(dt) /*]*/,
        /*[*/ std::sin(dt), std::cos(dt) /*]]*/;

    ASSERT_TRUE(test_utils::check_complex_matrix_equality(expected_A,
                                                          discrete_plant.A_));
  }
}

TEST(AnalysisTest, DiscretizationSimpleFuzz) {
  // this test takes advantage of the fact that eigenvalues of a continuous
  // system end up in the exponent of its discretized version.

  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using ContinuousPlant =
      dynamical::ContinuousPlant<num_states, num_inputs, num_outputs>;
  using DiscretePlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs,
                               std::complex<double>>;

  for (int fuzz_index = 0; fuzz_index != 20; ++fuzz_index) {
    ContinuousPlant::A_matrix_type test_A =
        ContinuousPlant::A_matrix_type::Random();
    ContinuousPlant::B_matrix_type test_B =
        ContinuousPlant::B_matrix_type::Zero();
    ContinuousPlant::x_vector_type x_initial =
        ContinuousPlant::x_vector_type::Random();

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

// TODO: finish implementing this and discretization MIMO test
TEST(AnalysisTest, DiscretizationSISO) {
  constexpr int num_states = 2, num_inputs = 1, num_outputs = 1;
  using ContinuousPlant =
      dynamical::ContinuousPlant<num_states, num_inputs, num_outputs>;
  using DiscretePlant =
      dynamical::DiscretePlant<num_states, num_inputs, num_outputs,
                               std::complex<double>>;

  ContinuousPlant::A_matrix_type test_A;
  test_A << /*[[*/ 0, 1 /*]*/,
      /*[*/ -2, -3 /*]]*/;
  ContinuousPlant::B_matrix_type test_B;
  test_B << /*[[*/ 0 /*]*/,
      /*[*/ 2 /*]]*/;

  ContinuousPlant::x_vector_type x_initial =
      ContinuousPlant::x_vector_type::Random();
  ContinuousPlant continuous_plant(x_initial, test_A, test_B);

  DiscretePlant discrete_plant =
      dynamical::analysis::discretize(continuous_plant, 0.01);
}

}  // namespace testing
