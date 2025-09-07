cmake_minimum_required(VERSION 3.0.2)

project(bmm_ros)

# -------------- CMake Policies ------------------#
#add_compile_options(-Wall -Werror=all)
#add_compile_options(-Wextra -Werror=extra)

# -------------- Support C++17 for using filesystem  ------------------#
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  geolib2
  ed
  ed_sensor_integration
)

# Find Eigen (header-only, but export target is modern and preferred)
find_package(Eigen3 REQUIRED NO_MODULE)

# ------------------------------------------------------------------------------------------------
#                                        CATKIN EXPORT
# ------------------------------------------------------------------------------------------------
# Declare the catkin package and what we export to dependents
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bmm_ros_lib
  CATKIN_DEPENDS roscpp geolib2 ed ed_sensor_integration
  DEPENDS Eigen3
)

include_directories(
  include
)

# Build core library (no main.cpp here)
add_library(bmm_ros_lib
        src/bayesian_gmm.cpp
        src/variational_gmm.cpp
)

target_include_directories(bmm_ros_lib
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(bmm_ros_lib
  PUBLIC
    ${catkin_LIBRARIES}
    Eigen3::Eigen
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
)
# # # Enable testing
# if (CATKIN_ENABLE_TESTING)
# #   find_package(catkin_lint_cmake REQUIRED)
# #   catkin_add_catkin_lint_test("-W2 --ignore HEADER_OUTSIDE_PACKAGE_INCLUDE_PATH")

#  # Utils unit tests (no models needed)
#   catkin_add_gtest(utils_tests test/test_utils.cpp)
#   if(TARGET utils_tests)
#     target_link_libraries(utils_tests sam_onnx_ros_core GTest::gtest_main ${catkin_LIBRARIES})
#     target_include_directories(utils_tests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
#   endif()

#   # SAM integration-ish tests (may need models)
#   catkin_add_gtest(sam_tests test/sam_test.cpp)
#   if(TARGET sam_tests)
#     target_link_libraries(sam_tests sam_onnx_ros_core GTest::gtest_main ${catkin_LIBRARIES})
#     target_include_directories(sam_tests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
#   endif()
# endif()



#If you want to debug
# set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_DEBUG "-g")