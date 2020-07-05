#include "dynamical/lti/controller.hpp"

#include "dynamical/lti/plant.hpp"
#include "dynamical/utils/test_utils.hpp"

#include <iostream>

#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace dynamical_testing {

TEST(Feedback, SanityCheck) {
  constexpr int num_states = 3, num_inputs = 2;  // num_outputs = num_states
  using SimplePlant =
      dynamical::lti::sim::DiscretePlant<num_states, num_inputs>;
  using Feedback = dynamical::lti::Feedback<num_states, num_inputs>;

  for (int i = 0; i != 20; ++i) {
    Feedback::K_MatrixType random_K = Feedback::K_MatrixType::Random();
    Feedback::K_MatrixType random_Kref = Feedback::K_MatrixType::Random();

    Feedback feedback(random_K, random_Kref);

    SimplePlant::x_VectorType curr_state = SimplePlant::x_VectorType::Random();
    SimplePlant::x_VectorType desired_state =
        SimplePlant::x_VectorType::Random();

    SimplePlant::u_VectorType expected_u =
        random_K * curr_state + random_Kref * desired_state;
    ASSERT_TRUE(dynamical_test_utils::check_matrix_equality(
        expected_u, feedback.GetU(curr_state, desired_state)));
  }
}

TEST(Observer, MatchToNoiselessPlant) {
  constexpr int num_states = 3, num_inputs = 2, num_outputs = 2;
  using SimplePlant =
      dynamical::lti::sim::DiscretePlant<num_states, num_inputs, num_outputs>;
  using Observer =
      dynamical::lti::Observer<num_states, num_inputs, num_outputs>;

  for (int i = 0; i != 20; ++i) {
    Observer::A_MatrixType A = Observer::A_MatrixType::Random();
    Observer::B_MatrixType B = Observer::B_MatrixType::Random();
    Observer::C_MatrixType C = Observer::C_MatrixType::Random();
    Observer::D_MatrixType D = Observer::D_MatrixType::Random();
    Observer::L_MatrixType L = Observer::L_MatrixType::Random();

    Observer::x_VectorType x_initial = Observer::x_VectorType::Random();

    SimplePlant plant(x_initial, A, B, C, D);
    Observer observer(x_initial, L, plant);

    for (int step = 0; step != 10; ++step) {
      Observer::u_VectorType u = Observer::u_VectorType::Zero();

      plant.Update(u);
      observer.UpdateEstimate(plant.GetY(), u);
    }

    ASSERT_TRUE(dynamical_test_utils::check_matrix_equality(
        plant.GetX(), observer.GetXhat()));
  }
}

// TODO: implement this
TEST(Observer, KalmanFilter) {}

}  // namespace dynamical_testing
