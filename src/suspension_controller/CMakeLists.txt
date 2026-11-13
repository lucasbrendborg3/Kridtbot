cmake_minimum_required(VERSION 3.5)
project(suspension_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(Ceres REQUIRED)

# Build the plugin library
add_library(${PROJECT_NAME} SHARED
  src/suspension_controller.cpp
)

# Include headers
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  Eigen3
  sensor_msgs
  tf2
  Ceres
)

target_link_libraries(${PROJECT_NAME}
  ${CERES_LIBRARIES}
)


# Export the plugin description
pluginlib_export_plugin_description_file(controller_interface suspension_controller_plugins.xml)

# Install the shared library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/ DESTINATION include/)

ament_package()
