cmake_minimum_required(VERSION 3.10)
project(openflex_ker_joint_panel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/joint_angle_panel.cpp
  include/openflex_ker_joint_panel/joint_angle_panel.hpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Widgets)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib
  rclcpp
  rcl_interfaces
  rviz_common
  sensor_msgs
  std_srvs
)

pluginlib_export_plugin_description_file(rviz_common plugins/plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins/plugin_description.xml DESTINATION share/${PROJECT_NAME}/plugins)
install(FILES resource/${PROJECT_NAME} DESTINATION share/${PROJECT_NAME}/resource)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(pluginlib rclcpp rcl_interfaces rviz_common sensor_msgs std_srvs)
ament_package()
