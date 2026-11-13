cmake_minimum_required(VERSION 3.16)
project(bno055_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  imu_bno055
)



# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  bno055_hardware_interface
  SHARED
  src/bno055_hardware.cpp
)
target_compile_features(bno055_hardware_interface PUBLIC cxx_std_17)
target_include_directories(bno055_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/bno055_hardware_interface>
)
ament_target_dependencies(
  bno055_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface bno055_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/bno055_hardware_interface
)

install(TARGETS bno055_hardware_interface
  EXPORT export_bno055_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

endif()

## EXPORTS
ament_export_targets(export_bno055_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()

