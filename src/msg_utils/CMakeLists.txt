cmake_minimum_required(VERSION 3.8)
project(msg_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_rs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoolArray.msg"
  "msg/WheelCommands.msg"
  "msg/GpsVelocityHeading.msg"
  "msg/BatteryStatus.msg"
  "msg/MotorStatus.msg"
  "msg/FourMotorsStatus.msg"
  "msg/MotorFeedback.msg"
  "msg/FourMotorsFeedback.msg"
  DEPENDENCIES std_msgs 
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
