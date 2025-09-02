cmake_minimum_required(VERSION 3.8)
project(ros_ceres_helper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++17 -g -ggdb -Wall -Wextra -Wpedantic -O3)
endif()

FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(glog REQUIRED)

add_definitions(-DSOPHUS_USE_BASIC_LOGGING)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${CERES_INCLUDES}
 ${SOPHUS_INCLUDES}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/ceres_poses.cpp
  src/ceres_rotations.cpp
  src/ceres_flags.cpp
  src/markers.cpp
  src/BasicOptimisationProblem.cpp
)

target_link_libraries(${PROJECT_NAME} Ceres::ceres gflags glog)

ament_target_dependencies(
    ${PROJECT_NAME}
    "rclcpp" 
    "tf2" 
    "tf2_ros"
    "geometry_msgs"
    "visualization_msgs"
)



install(TARGETS
    ${PROJECT_NAME}
  DESTINATION lib)

install(
    DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

ament_export_dependencies(
    Ceres
    Sophus
    rclcpp
    tf2
    tf2_ros
    geometry_msgs
    visualization_msgs
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


add_executable(test_ceres_spline src/test_ceres_spline.cpp)
target_link_libraries(test_ceres_spline ${PROJECT_NAME} Ceres::ceres gflags glog)
ament_target_dependencies(
    test_ceres_spline
    "rclcpp" 
    "tf2" 
    "tf2_ros"
    "geometry_msgs"
    "visualization_msgs"
)
# target_link_libraries(test_ceres_spline ${PROJECT_NAME} ${catkin_LIBRARIES}  Ceres::ceres)
install(TARGETS test_ceres_spline
  DESTINATION lib/${PROJECT_NAME})

add_executable(test_ceres_spline_sophus src/test_ceres_spline_sophus.cpp)
target_link_libraries(test_ceres_spline_sophus ${PROJECT_NAME} Ceres::ceres gflags glog)
ament_target_dependencies(
    test_ceres_spline_sophus
    "rclcpp" 
    "tf2" 
    "tf2_ros"
    "geometry_msgs"
    "visualization_msgs"
)
# target_link_libraries(test_ceres_spline_sophus ${PROJECT_NAME} ${catkin_LIBRARIES}  Ceres::ceres)
install(TARGETS test_ceres_spline_sophus
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
