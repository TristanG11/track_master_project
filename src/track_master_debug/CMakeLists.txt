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

install(
  DIRECTORY ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  ${PROJECT_NAME}/wheel_commands_debug.py
  #${PROJECT_NAME}/data_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)
ament_package()
