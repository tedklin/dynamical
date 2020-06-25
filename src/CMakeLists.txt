list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/placeholder.cpp")
# ^ APPEND NEW SOURCE FILES TO THE SOURCE_LIST

# Make an automatic library - will be static or dynamic based on user setting
# add_library(dynamical_lib ${SOURCE_LIST} ${HEADER_LIST})
add_library(dynamical_lib ${SOURCE_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(dynamical_lib PUBLIC ../include)

# dependency on Eigen
# TODO: do my examples' and tests' dependencies on Eigen justify this being PUBLIC
# or should i link the examples and tests to Eigen separately, all PRIVATE?
target_link_libraries(dynamical_lib PUBLIC Eigen3::Eigen)

# All users of this library will need at least C++11
target_compile_features(dynamical_lib PUBLIC cxx_std_11)
