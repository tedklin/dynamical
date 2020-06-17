# macro for adding executable examples
macro(package_add_example EXAMPLENAME)
  # create an exectuable in which the tests will be stored
  add_executable(${EXAMPLENAME} ${ARGN})
  target_compile_features(${EXAMPLENAME} PRIVATE cxx_std_17)

  # link dynamical_lib (along with eigen)
  target_link_libraries(${EXAMPLENAME} PRIVATE dynamical_lib)
endmacro()

# package_add_example(salto SaltoJumpingRobot.cpp)
# ^ ADD MORE EXAMPLE EXECUTABLES HERE
