#include "cmake_sandbox/ContinuousSystem.h"

#include "Eigen/Eigen"
#include "gtest/gtest.h"

TEST(ContinuousSystemTest, SanityCheck) { ASSERT_EQ(20, 20); }

TEST(ContinuousSystemTest, SaltoStable) {
  Eigen::Matrix3d A;
  A << /*[[*/ 0, 1, 0, /*]*/
      /*[*/ 100, 0, 0, /*]*/
      /*[*/ 0, 0, 0;   /*]]*/

  Eigen::Vector3d B;
  B << /*[[*/ 0, /*]*/
      /*[*/ -25, /*]*/
      /*[*/ 500; /*]]*/

  Eigen::RowVector3d K;
  K << 20, 5, 0.01;

  ContinuousSystem salto_jumping_robot(A, B, K);

  ASSERT_TRUE(salto_jumping_robot.IsStable());
}
