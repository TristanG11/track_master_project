cmake_minimum_required(VERSION 3.8)
project(track_master_debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(msg_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetCmdType.srv"
  DEPENDENCIES std_msgs
)

install(
  DIRECTORY ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  ${PROJECT_NAME}/wheel_commands_debug.py
  ${PROJECT_NAME}/app_initializer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
