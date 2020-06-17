#include "dynamical/ContinuousSystem.h"

#include <iostream>

#include "Eigen/Eigen"

// a continuous-time system taken from a homework problem for EECS16B @Berkeley

int main() {
  // clang-format automatically turns Eigen's comma initializer into a single
  // line, so the commented brackets are here to suppress that

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

  std::cout << "System Matrix\n";
  std::cout << salto_jumping_robot.GetSystem() << '\n';
  std::cout << "\nEigenvalues\n";
  std::cout << salto_jumping_robot.GetEigenvalues() << '\n';
  if (salto_jumping_robot.IsStable()) {
    std::cout << "\nSystem stable\n";
  } else {
    std::cout << "\nSystem unstable\n";
  }
}
