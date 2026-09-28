###
# wheel_scale - adaptive wheel scale from RTK geometry (port of ROS1 xbot_positioning's calibration)
###
add_library(wheel_scale_component SHARED src/wheel_scale/wheel_scale_node.cpp)
target_compile_features(wheel_scale_component PUBLIC cxx_std_17)
target_include_directories(wheel_scale_component PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
ament_target_dependencies(wheel_scale_component rclcpp rclcpp_components nav_msgs sensor_msgs std_msgs)
rclcpp_components_register_nodes(wheel_scale_component "open_mower_next::wheel_scale::WheelScaleNode")
install(TARGETS wheel_scale_component DESTINATION lib)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(wheel_scale_test test/wheel_scale_test.cpp)
  if (TARGET wheel_scale_test)
    target_include_directories(wheel_scale_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  endif ()
endif ()
