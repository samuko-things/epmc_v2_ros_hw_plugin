cmake_minimum_required(VERSION 3.16)
project(epmc_v2_ros_hw_plugin LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  epmc_v2_ros_hw_plugin
  SHARED
  src/epmc_v2_ros_hw_plugin.cpp
)

target_compile_features(epmc_v2_ros_hw_plugin PUBLIC cxx_std_17)

target_include_directories(epmc_v2_ros_hw_plugin PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(
  epmc_v2_ros_hw_plugin PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(epmc_v2_ros_hw_plugin PUBLIC serial)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface epmc_v2_ros_hw_plugin.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/epmc_v2_ros_hw_plugin
)

install(TARGETS epmc_v2_ros_hw_plugin
  EXPORT export_epmc_v2_ros_hw_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_epmc_v2_ros_hw_plugin HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
