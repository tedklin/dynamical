include(GoogleTest)

FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        release-1.10.0
)
FetchContent_MakeAvailable(googletest)

# macro for adding gtests
macro(package_add_test TESTNAME)
  add_executable(${TESTNAME} ${ARGN})

  target_compile_features(${TESTNAME} PRIVATE cxx_std_17)
  set_target_properties(${TESTNAME} PROPERTIES CXX_EXTENSIONS OFF)
  target_compile_options(${TESTNAME} PRIVATE "-Wall")

  # Link the Google test infrastructure, mocking library, and a default main
  # function to the test executable.
  # Remove g_test_main if writing your own main function.
  target_link_libraries(${TESTNAME} gtest gmock gtest_main)

  target_link_libraries(${TESTNAME} dynamical_lib)

  # gtest_discover_tests replaces gtest_add_tests.
  # See https://cmake.org/cmake/help/v3.10/module/GoogleTest.html
  # for more options to pass to it.
  gtest_discover_tests(${TESTNAME}
    # Set a working directory to your project root so that you can find test
    # data via paths relative to project root.
    WORKING_DIRECTORY ${PROJECT_DIR}
    PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${PROJECT_DIR}"
  )
set_target_properties(${TESTNAME} PROPERTIES FOLDER tests)
endmacro()

package_add_test(integration_test math/integral_test.cpp)

package_add_test(plant_test lti/plant_test.cpp)
package_add_test(analysis_test lti/analysis_test.cpp)
package_add_test(controller_test lti/controller_test.cpp)

package_add_test(min_energy_test trajectory/min_energy_test.cpp)
# ^ ADD NEW TESTS HERE
