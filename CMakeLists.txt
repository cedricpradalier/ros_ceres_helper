cmake_minimum_required(VERSION 3.8)
project(ros_ceres_helper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-std=c++17 -g -ggdb -Wall -Wextra -Wpedantic -O3)
endif()

FIND_PACKAGE(Ceres REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(gflags REQUIRED)
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
add_library(${PROJECT_NAME} SHARED
    # src/ceres_poses.cpp
    # src/ceres_rotations.cpp
  # src/markers.cpp
  src/ceres_flags.cpp
  src/BasicOptimisationProblem.cpp
)

add_library(${PROJECT_NAME}_poses STATIC
    src/ceres_poses.cpp
    src/ceres_rotations.cpp
  src/markers.cpp
)


target_link_libraries(${PROJECT_NAME}
    ${geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    tf2_ros::tf2_ros
    Sophus::Sophus
    Ceres::ceres
    gflags
    glog
    )    

target_link_libraries(${PROJECT_NAME}_poses ${PROJECT_NAME})

install(
    DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# this line to exports the library
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
 
ament_export_dependencies(
    Ceres
    Sophus
    gflags
    glog
    rclcpp
    tf2
    tf2_ros
    geometry_msgs
    visualization_msgs
)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} 
    EXPORT ${PROJECT_NAME} 
    DESTINATION lib 
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)


add_executable(test_ceres_spline src/test_ceres_spline.cpp)
target_link_libraries(test_ceres_spline ${PROJECT_NAME}_poses ${PROJECT_NAME} )
install(TARGETS test_ceres_spline
  DESTINATION lib/${PROJECT_NAME})

add_executable(test_ceres_spline_sophus src/test_ceres_spline_sophus.cpp)
target_link_libraries(test_ceres_spline_sophus ${PROJECT_NAME}_poses ${PROJECT_NAME} )
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
