#include "dynamical/state_space/controller.hpp"

#include "dynamical/state_space/plant.hpp"
#include "dynamical/test_utils.hpp"

#include <iostream>

#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace testing {

TEST(ControllerTest, FeedbackSanityCheck) {
  constexpr int num_states = 3, num_inputs = 2;  // num_outputs = num_states
  using SimplePlant = dynamical::DiscretePlant<num_states, num_inputs>;
  using SimpleFeedback = dynamical::Feedback<num_states, num_inputs>;

  SimpleFeedback::K_MatrixType test_K = SimpleFeedback::K_MatrixType::Random();
  SimpleFeedback::K_MatrixType test_Kref =
      SimpleFeedback::K_MatrixType::Random();

  SimpleFeedback feedback(test_K, test_Kref);

  SimplePlant::x_VectorType curr_state = SimplePlant::x_VectorType::Random();
  SimplePlant::x_VectorType desired_state = SimplePlant::x_VectorType::Random();

  SimplePlant::u_VectorType expected_u =
      -test_K * curr_state + test_Kref * desired_state;
  ASSERT_TRUE(test_utils::check_matrix_equality(
      expected_u, feedback.GetU(curr_state, desired_state)));
}

}  // namespace testing
