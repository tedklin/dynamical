#include "dynamical/state_space/controller.hpp"

#include "dynamical/state_space/plant.hpp"
#include "dynamical/utils/test_utils.hpp"

#include <iostream>

#include "Eigen/Dense"
#include "gtest/gtest.h"

namespace testing {

TEST(Controller, FeedbackSanityCheck) {
  constexpr int num_states = 3, num_inputs = 2;  // num_outputs = num_states
  using SimplePlant = dynamical::DiscretePlant<num_states, num_inputs>;
  using Feedback = dynamical::Feedback<num_states, num_inputs>;

  Feedback::K_MatrixType random_K = Feedback::K_MatrixType::Random();
  Feedback::K_MatrixType random_Kref = Feedback::K_MatrixType::Random();

  Feedback feedback(random_K, random_Kref);

  SimplePlant::x_VectorType curr_state = SimplePlant::x_VectorType::Random();
  SimplePlant::x_VectorType desired_state = SimplePlant::x_VectorType::Random();

  SimplePlant::u_VectorType expected_u =
      random_K * curr_state + random_Kref * desired_state;
  ASSERT_TRUE(dynamical_test_utils::check_matrix_equality(
      expected_u, feedback.GetU(curr_state, desired_state)));
}

}  // namespace testing