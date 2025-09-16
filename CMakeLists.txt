cmake_minimum_required(VERSION 3.0.2)

project(bmm_ros)

# -------------- CMake Policies ------------------#
add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

# -------------- Support C++17 for using filesystem  ------------------#
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  geolib2
)

find_package(Boost REQUIRED)
find_package(console_bridge REQUIRED)

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
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS geolib2
  DEPENDS EIGEN3 PCL console_bridge
)


# Build core library (no main.cpp here)
add_library(${PROJECT_NAME}_lib
        src/bayesian_mixture_model.cpp
        src/variational_gmm.cpp
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_lib
  PUBLIC
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

# Executable (example/entry point)
add_executable(${PROJECT_NAME}
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
  PRIVATE
    ${PROJECT_NAME}_lib
    ${catkin_LIBRARIES}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test("-W2 --ignore HEADER_OUTSIDE_PACKAGE_INCLUDE_PATH")

  # catkin_add_gtest creates target bmm_test and links gtest
  catkin_add_gtest(bmm_test test/bmm_test.cpp)
  if (TARGET bmm_test)
    # Use the plain signature here because catkin_add_gtest used it internally
    target_link_libraries(bmm_test
      PRIVATE
        ${PROJECT_NAME}_lib
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        GTest::GTest
        GTest::Main
    )
    # No need to add include dirs: bmm_ros_lib exports them PUBLIC
  endif()
endif()
