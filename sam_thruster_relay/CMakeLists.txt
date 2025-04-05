cmake_minimum_required(VERSION 3.8)
project(sam_thruster_relay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sam_msgs REQUIRED)
find_package(smarc_msgs REQUIRED)

include_directories(
  include
)

set(ros_dependencies
  rclcpp
  std_msgs
  smarc_msgs
  sam_msgs
)

add_executable(thrust_vector_relay src/thrust_vector_relay.cpp)
ament_target_dependencies(thrust_vector_relay ${ros_dependencies})

add_executable(thrust_vector_horizontal_relay src/thrust_vector_horizontal_relay.cpp)
ament_target_dependencies(thrust_vector_horizontal_relay ${ros_dependencies})

add_executable(thrust_cmd_relay src/thrust_cmd_relay.cpp)
ament_target_dependencies(thrust_cmd_relay ${ros_dependencies})

install(TARGETS
  thrust_vector_relay
  thrust_vector_horizontal_relay
  thrust_cmd_relay
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
