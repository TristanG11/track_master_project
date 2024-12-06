cmake_minimum_required(VERSION 3.5)
project(diffdrive_arduino)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(msg_utils REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/DiffDriveArduinoHardwareInterface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Lier les dépendances aux bibliothèques
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  msg_utils
  hardware_interface
  pluginlib
)

target_link_libraries(${PROJECT_NAME}
  Boost::system
)

# Export plugin for ros2
pluginlib_export_plugin_description_file(hardware_interface DiffDriveArduinoHardwareInterface.xml)

# Installation
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)


ament_package()
