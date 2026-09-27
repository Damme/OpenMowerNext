###
# map_recorder
###
add_library(map_recorder_component SHARED src/map_recorder/map_recorder_node.cpp)
target_compile_features(map_recorder_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(map_recorder_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(map_recorder_component "${cpp_typesupport_target}")
ament_target_dependencies(map_recorder_component
  rclcpp
  rclcpp_components
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav2_msgs
  std_msgs
  std_srvs
)

add_dependencies(map_recorder_component ${PROJECT_NAME})
rclcpp_components_register_nodes(map_recorder_component "open_mower_next::map_recorder::MapRecorderNode")

add_executable(map_recorder src/map_recorder/main.cpp)
target_link_libraries(map_recorder map_recorder_component)

INSTALL(TARGETS map_recorder DESTINATION lib/${PROJECT_NAME})
INSTALL(TARGETS map_recorder_component DESTINATION lib)