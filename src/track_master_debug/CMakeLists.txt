cmake_minimum_required(VERSION 3.8)
project(track_master_debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
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
if(NOT TARGET ament_cmake_python_symlink_${PROJECT_NAME})
  ament_python_install_package(${PROJECT_NAME})
endif()

#install(
#  DIRECTORY ${PROJECT_NAME}
#  DESTINATION lib/${PROJECT_NAME}
#)

install(
  PROGRAMS
    track_master_debug/app_initializer.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  PROGRAMS
    track_master_debug/wheel_commands_debug.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
