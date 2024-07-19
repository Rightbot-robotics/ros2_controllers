cmake_minimum_required(VERSION 3.8)
project(gpio_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    controller_interface
    rightbot_hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    generate_parameter_library
    rightbot_interfaces
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  gpio_controller_parameters
  src/gpio_controller_parameters.yaml
)

add_library(gpio_controller SHARED
  src/gpio_controller.cpp
)
target_compile_features(gpio_controller PUBLIC cxx_std_17)
target_include_directories(gpio_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/gpio_controller>
)
target_link_libraries(gpio_controller PUBLIC
  gpio_controller_parameters
)
ament_target_dependencies(gpio_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# add_library(${PROJECT_NAME} SHARED
#   # src/parameter_handler.cpp
#   src/gpio_controller.cpp
# )
target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface gpio_controller_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_package()
