list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/placeholder.cpp")
# ^ APPEND NEW SOURCE FILES TO THE SOURCE_LIST

# Make an automatic library - will be static or dynamic based on user setting
# add_library(dynamical_lib ${SOURCE_LIST} ${HEADER_LIST})
add_library(dynamical_lib ${SOURCE_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(dynamical_lib PUBLIC ../include)

# Public dependency on Eigen (everything that uses this library is going to need Eigen)
target_link_libraries(dynamical_lib PUBLIC Eigen3::Eigen)

# All users of this library will need at least C++11
target_compile_features(dynamical_lib PUBLIC cxx_std_11)
