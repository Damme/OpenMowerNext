###
# ubx_gps - F9P NAV-PVT from str2str's TCP server -> gps/fix (Worx robot)
###
add_library(ubx_gps_component SHARED src/ubx_gps/ubx_gps_node.cpp)
target_include_directories(ubx_gps_component PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
ament_target_dependencies(ubx_gps_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(ubx_gps_component
  PLUGIN "open_mower_next::ubx_gps::UbxGpsNode"
  EXECUTABLE ubx_gps_node)
install(TARGETS ubx_gps_component DESTINATION lib)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(ubx_test test/ubx_test.cpp)
  if (TARGET ubx_test)
    target_include_directories(ubx_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  endif ()
endif ()
