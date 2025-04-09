cmake_minimum_required(VERSION 3.8)
project(mqtt_msg_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
  include
)

set(ros_dependencies
  rclcpp
  std_msgs
  sensor_msgs
  nlohmann_json
)

add_executable(string_to_navsatfix src/string_to_navsatfix.cpp)
ament_target_dependencies(string_to_navsatfix ${ros_dependencies})

add_executable(int32_to_float32 src/int32_to_float32.cpp)
ament_target_dependencies(int32_to_float32 ${ros_dependencies})

install(TARGETS
  string_to_navsatfix
  int32_to_float32
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
