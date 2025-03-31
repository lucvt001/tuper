cmake_minimum_required(VERSION 3.8)
project(formation_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(smarc_msgs REQUIRED)
find_package(sam_msgs REQUIRED)
find_package(tuper_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
)

set(ros_dependencies
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  smarc_msgs
  sam_msgs
  tuper_interfaces
  tf2
  tf2_geometry_msgs
)

add_executable(ping_synchronizer src/ping_synchronizer.cpp)
ament_target_dependencies(ping_synchronizer ${ros_dependencies})

add_executable(string_stamped_processing src/string_stamped_processing.cpp)
ament_target_dependencies(string_stamped_processing ${ros_dependencies})

add_executable(ground_truth_tf2 src/ground_truth_tf2.cpp)
ament_target_dependencies(ground_truth_tf2 ${ros_dependencies})

add_executable(fuse_distance_triangulation src/fuse_distance_triangulation.cpp)
ament_target_dependencies(fuse_distance_triangulation ${ros_dependencies})

add_executable(pid_server src/pid.cpp src/pid_server.cpp)
ament_target_dependencies(pid_server ${ros_dependencies})

add_executable(thrust_vector_relay src/thrust_vector_relay.cpp)
ament_target_dependencies(thrust_vector_relay ${ros_dependencies})

add_executable(thrust_vector_horizontal_relay src/thrust_vector_horizontal_relay.cpp)
ament_target_dependencies(thrust_vector_horizontal_relay ${ros_dependencies})

add_executable(thrust_cmd_relay src/thrust_cmd_relay.cpp)
ament_target_dependencies(thrust_cmd_relay ${ros_dependencies})

add_executable(relative_target_position_publisher src/relative_target_position_publisher.cpp)
ament_target_dependencies(relative_target_position_publisher ${ros_dependencies})

add_executable(relative_tf_listener src/relative_tf_listener.cpp)
ament_target_dependencies(relative_tf_listener ${ros_dependencies})

add_executable(get_differential_value src/get_differential_value.cpp)
ament_target_dependencies(get_differential_value ${ros_dependencies})

add_executable(formation_shape_broadcaster src/formation_shape_broadcaster.cpp)
ament_target_dependencies(formation_shape_broadcaster ${ros_dependencies} yaml-cpp)
target_link_libraries(formation_shape_broadcaster ${YAML_CPP_LIBRARIES})

install(TARGETS
  ping_synchronizer
  string_stamped_processing
  ground_truth_tf2
  fuse_distance_triangulation
  pid_server
  thrust_vector_relay
  thrust_vector_horizontal_relay
  thrust_cmd_relay
  relative_target_position_publisher
  relative_tf_listener
  get_differential_value
  formation_shape_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/
  DESTINATION include/
  FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
