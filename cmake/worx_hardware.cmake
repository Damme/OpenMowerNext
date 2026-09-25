###
# worx_hardware - ros2_control SystemInterface for the Worx mainboard (JSON over SPI)
###
find_package(hardware_interface REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# ROS-free protocol/link layer (unit tested on its own).
add_library(worx_link STATIC
  src/worx_hardware/worx_protocol.cpp
  src/worx_hardware/transport.cpp
  src/worx_hardware/worx_link.cpp
)
target_compile_features(worx_link PUBLIC cxx_std_17)
set_target_properties(worx_link PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(worx_link PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(worx_link PUBLIC nlohmann_json::nlohmann_json Threads::Threads)

add_library(worx_hardware SHARED src/worx_hardware/worx_system.cpp)
target_compile_features(worx_hardware PUBLIC cxx_std_17)
target_include_directories(worx_hardware PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(worx_hardware worx_link "${cpp_typesupport_target}")
ament_target_dependencies(worx_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  std_srvs
)
add_dependencies(worx_hardware ${PROJECT_NAME})
pluginlib_export_plugin_description_file(hardware_interface src/worx_hardware/plugins.xml)
install(TARGETS worx_hardware DESTINATION lib)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(worx_link_test test/worx_link_test.cpp)
  if (TARGET worx_link_test)
    target_link_libraries(worx_link_test worx_link)
  endif ()
endif ()
