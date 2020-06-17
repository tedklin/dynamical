# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.

# Optionally glob, but only for CMake 3.12 or later:
# file(GLOB HEADER_LIST CONFIGURE_DEPENDS "${ModernCMakeExample_SOURCE_DIR}/include/modern/*.hpp")
# set(HEADER_LIST "${ModernCMakeExample_SOURCE_DIR}/include/modern/lib.hpp")
list(APPEND HEADER_LIST "${PROJECT_SOURCE_DIR}/include/cmake_sandbox/ContinuousSystem.h")
# ^ APPEND NEW HEADERS TO THE HEADER_LIST

list(APPEND SOURCE_LIST "${PROJECT_SOURCE_DIR}/src/ContinuousSystem.cpp")
# ^ APPEND NEW SOURCE FILES TO THE SOURCE_LIST

# Make an automatic library - will be static or dynamic based on user setting
add_library(cmake_sandbox_lib ${SOURCE_LIST} ${HEADER_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(cmake_sandbox_lib PUBLIC ../include)

# dependency on Eigen
# TODO: do my examples' and tests' dependencies on Eigen justify this being PUBLIC
# or should i link the examples and tests to Eigen separately, all PRIVATE?
target_link_libraries(cmake_sandbox_lib PUBLIC Eigen3::Eigen)

# All users of this library will need at least C++11
target_compile_features(cmake_sandbox_lib PUBLIC cxx_std_11)

# IDEs should put the headers in a nice place
source_group(TREE "${PROJECT_SOURCE_DIR}/include" PREFIX "Header Files" FILES ${HEADER_LIST})
