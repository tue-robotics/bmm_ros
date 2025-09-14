cmake_minimum_required(VERSION 3.0.2)

project(bmm_ros)

# Warnings
add_compile_options(-Wall -Wextra)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Catkin and ROS deps
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geolib2
  ed
)

# Eigen3 (header-only)
find_package(Eigen3 REQUIRED NO_MODULE)

# PCL (for PCD IO)
find_package(PCL 1.8 REQUIRED COMPONENTS io)

# ------------------------------------------------------------------------------
# Export config (make Eigen variables visible to catkin to silence warning)
# Note: catkin_package(DEPENDS <Token>) expects <Token>_INCLUDE_DIRS / _LIBRARIES.
# We use 'EIGEN3' (matches EIGEN3_INCLUDE_DIRS) to avoid case mismatch.
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
# Header-only; library target is optional:
# set(EIGEN3_LIBRARIES Eigen3::Eigen)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bmm_ros_lib
  CATKIN_DEPENDS roscpp geolib2 ed
  DEPENDS EIGEN3 PCL
)

# ------------------------------------------------------------------------------
# Targets

# Core library
add_library(bmm_ros_lib
  src/bayesian_mixture_model.cpp
  src/variational_gmm.cpp
)

target_include_directories(bmm_ros_lib
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

target_link_libraries(bmm_ros_lib
  PUBLIC
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

# Example executable
add_executable(bmm_ros
  src/main.cpp
)

target_link_libraries(bmm_ros
  PRIVATE
    bmm_ros_lib
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

# ------------------------------------------------------------------------------
# Tests
if (CATKIN_ENABLE_TESTING)
  find_package(GTest REQUIRED)
  catkin_add_gtest(bmm_test test/bmm_test.cpp)
  if (TARGET bmm_test)
    target_include_directories(bmm_test PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${PCL_INCLUDE_DIRS}
    )
    target_link_libraries(bmm_test
      PRIVATE
        bmm_ros_lib
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        GTest::GTest
        GTest::Main
    )
  endif()
endif()
