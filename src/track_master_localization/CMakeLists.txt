cmake_minimum_required(VERSION 3.8)
project(track_master_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

install(
  DIRECTORY ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Pour supporter Python et installer le script Python
install(
  PROGRAMS
  ${PROJECT_NAME}/kalman_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)


ament_package()
