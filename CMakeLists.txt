cmake_minimum_required(VERSION 3.11)
# 3.11 minimum (FetchContent), 3.14 preferred

project(dynamical LANGUAGES CXX)

# Only do these if this is the main project, 
# and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
  # If no default build type is specified, use the presence of a git
  # directory to determine whether to build Debug or Release.
  set(default_build_type "Release")
  if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/.git")
    set(default_build_type "Debug")
  endif()
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
    set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
        STRING "Choose the type of build." FORCE)
    # Retain possible values of build type for cmake-gui.
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
  endif()

  # c++17 enabled compiler
  set(CMAKE_CXX_STANDARD 17)

  # ensure -std=c++xx instead of -std=g++xx
  set(CMAKE_CXX_EXTENSIONS OFF)

  # nicely support folders in IDEs
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)

  # Testing only available if this is the main app.
  # Note this needs to be done in the main CMakeLists since it calls
  # enable_testing, which must be in the main CMakeLists.
  include(CTest)

  # ccache
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
      set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
      set(CMAKE_CUDA_COMPILER_LAUNCHER "${CCACHE_PROGRAM}") # CMake 3.9+
  endif()
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Using FindEigen3.cmake in cmake/ folder
find_package(Eigen3 3.3 REQUIRED)

# FetchContent
include(FetchContent)
# FetchContent_MakeAvailable macro (included in CMake 3.14)
if(${CMAKE_VERSION} VERSION_LESS 3.14)
  macro(FetchContent_MakeAvailable NAME)
    FetchContent_GetProperties(${NAME})
    if(NOT ${NAME}_POPULATED)
      FetchContent_Populate(${NAME})
      add_subdirectory(${${NAME}_SOURCE_DIR} ${${NAME}_BINARY_DIR})
    endif()
  endmacro()
endif()

add_subdirectory(include/dynamical)

add_subdirectory(examples)

# Tests are only added if this is the main project
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME AND BUILD_TESTING)
  add_subdirectory(tests)
endif()
