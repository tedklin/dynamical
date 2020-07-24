# INTERFACE for header-only library
# https://cmake.org/cmake/help/latest/manual/cmake-buildsystem.7.html#interface-libraries
add_library(dynamical_lib INTERFACE)

# We need this directory, and users of our library will need it too
target_include_directories(dynamical_lib INTERFACE ..)

# Public dependency on Eigen (everything that uses this library is going to need Eigen)
target_link_libraries(dynamical_lib INTERFACE Eigen3::Eigen)

# All users of this library will need at least C++11
target_compile_features(dynamical_lib INTERFACE cxx_std_11)
