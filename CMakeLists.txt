cmake_minimum_required(VERSION 3.0.2)

project(bmm_ros)

# -------------- CMake Policies ------------------#
add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

# -------------- Support C++17 for using filesystem  ------------------#
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Catkin and ROS deps (rosconsole for ROS_* logging)
find_package(catkin REQUIRED COMPONENTS
  geolib2
  rosconsole
)

# Find Eigen (header-only, but export target is modern and preferred)
find_package(Eigen3 REQUIRED NO_MODULE)

# PCL (for PCD IO)
find_package(PCL 1.8 REQUIRED COMPONENTS io)
add_definitions(${PCL_DEFINITIONS})

# ------------------------------------------------------------------------------------------------
#                                        CATKIN EXPORT
# ------------------------------------------------------------------------------------------------
# Declare the catkin package and what we export to dependents
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bmm_ros_lib
  CATKIN_DEPENDS geolib2 rosconsole
  DEPENDS EIGEN3 PCL
)


# Build core library (no main.cpp here)
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

# Executable (example/entry point)
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

# Tests
if (CATKIN_ENABLE_TESTING)
  # catkin_add_gtest creates target bmm_test and links gtest
  catkin_add_gtest(bmm_test test/bmm_test.cpp)
  if (TARGET bmm_test)
    # Use the plain signature here because catkin_add_gtest used it internally
    target_link_libraries(bmm_test
      bmm_ros_lib
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      GTest::GTest
      GTest::Main
    )
    # No need to add include dirs: bmm_ros_lib exports them PUBLIC
  endif()
endif()