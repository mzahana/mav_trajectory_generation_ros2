cmake_minimum_required(VERSION 3.8)
project(mav_trajectory_generation_ros2)

# Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED) # Assuming you have a vendor package for yaml-cpp in ROS 2

find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(glog REQUIRED)  # <-- Find glog

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolynomialSegment.msg"
  "msg/PolynomialSegment4D.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/PolynomialTrajectory4D.msg"
  "msg/Waypoint.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)
include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)


# If you have other ROS 2 dependencies, find them here:
# find_package(dependency_name REQUIRED)

# Libraries
add_library(${PROJECT_NAME}_lib
  src/motion_defines.cpp
  src/polynomial.cpp
  src/segment.cpp
  src/timing.cpp
  src/trajectory.cpp
  src/trajectory_sampling.cpp
  src/vertex.cpp
  src/io.cpp
  src/rpoly/rpoly_ak1.cpp
  src/ros_conversions.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib yaml_cpp_vendor Eigen3 geometry_msgs nav_msgs)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
)
# Link against glog
target_link_libraries(${PROJECT_NAME}_lib glog::glog ${cpp_typesupport_target})  # <-- Link glog

# Executables
add_executable(polynomial_timing_evaluation
  src/polynomial_timing_evaluation.cpp
)
target_link_libraries(polynomial_timing_evaluation ${PROJECT_NAME}_lib glog::glog)  # <-- Link glog


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


ament_export_dependencies(rosidl_default_runtime glog)  # <-- Export glog
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS polynomial_timing_evaluation
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

#
# go_to_waypoint node
#
add_executable(go_to_waypoint_node nodes/go_to_waypoint_node.cpp)
ament_target_dependencies(go_to_waypoint_node
                          nav_msgs
                          geometry_msgs
                          visualization_msgs
                          rclcpp 
                          Eigen3)

target_link_libraries(go_to_waypoint_node ${PROJECT_NAME}_lib)
target_include_directories(go_to_waypoint_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(go_to_waypoint_node ${cpp_typesupport_target})


install(TARGETS
  go_to_waypoint_node
  DESTINATION lib/${PROJECT_NAME}
)


###########################
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

##############################

ament_package()