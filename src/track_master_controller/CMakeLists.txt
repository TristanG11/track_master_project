cmake_minimum_required(VERSION 3.8)
project(track_master_controller)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(msg_utils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# === Build the executable ===
add_executable(track_master_controller_node
  src/track_master_controller_node.cpp
  src/track_master_controller.cpp
)
target_compile_features(track_master_controller_node PUBLIC cxx_std_17)

ament_target_dependencies(track_master_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  msg_utils
  tf2
  tf2_ros
  tf2_geometry_msgs 
)

install(TARGETS
  track_master_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Install headers ===
install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
# === Tests ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
