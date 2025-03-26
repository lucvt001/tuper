cmake_minimum_required(VERSION 3.8)
project(tuper_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(message_files
)

set(service_files
)

set(action_files
  "action/RunPid.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files}
  ${service_files}
  ${action_files}
  DEPENDENCIES action_msgs std_msgs
)

ament_package()
