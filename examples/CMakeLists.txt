# macro for adding executable examples
macro(package_add_example EXAMPLENAME)
  add_executable(${EXAMPLENAME} ${ARGN})
  target_compile_features(${EXAMPLENAME} PRIVATE cxx_std_17)
  target_link_libraries(${EXAMPLENAME} PRIVATE dynamical_lib)
  set_target_properties(${EXAMPLENAME} PROPERTIES CXX_EXTENSIONS OFF)
  target_compile_options(${EXAMPLENAME} PRIVATE "-Wall")
endmacro()

# package_add_example(salto SaltoJumpingRobot.cpp)
# ^ ADD MORE EXAMPLE EXECUTABLES HERE
