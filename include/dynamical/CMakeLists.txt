# INTERFACE (header-only library)
# https://cmake.org/cmake/help/latest/manual/cmake-buildsystem.7.html#interface-libraries
add_library(dynamical_lib INTERFACE)

target_include_directories(dynamical_lib INTERFACE ..)
target_link_libraries(dynamical_lib INTERFACE Eigen3::Eigen)
target_compile_features(dynamical_lib INTERFACE cxx_std_11)
