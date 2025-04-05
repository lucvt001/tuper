cmake_minimum_required(VERSION 3.8)
project(tuper_sim_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(smarc_msgs REQUIRED)
find_package(sam_msgs REQUIRED)

include_directories(
  include
)

set(ros_dependencies
  rclcpp
  std_msgs
  smarc_msgs
  sam_msgs
)

add_executable(ping_synchronizer src/ping_synchronizer.cpp)
ament_target_dependencies(ping_synchronizer ${ros_dependencies})

add_executable(string_stamped_processing src/string_stamped_processing.cpp)
ament_target_dependencies(string_stamped_processing ${ros_dependencies})

install(TARGETS
  ping_synchronizer
  string_stamped_processing
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
