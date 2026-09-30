###
# ina226 - INA226 I2C current/voltage monitor (Worx robot) publishing /power/ina226
###
add_library(ina226 STATIC src/ina226/ina226.cpp)
target_compile_features(ina226 PUBLIC cxx_std_17)
target_include_directories(ina226 PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)

set_target_properties(ina226 PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(ina226_component SHARED src/ina226/ina226_node.cpp)
target_link_libraries(ina226_component ina226)
ament_target_dependencies(ina226_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(ina226_component
  PLUGIN "open_mower_next::ina226::Ina226Node"
  EXECUTABLE ina226_node)
install(TARGETS ina226_component DESTINATION lib)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(ina226_test test/ina226_test.cpp)
  if (TARGET ina226_test)
    target_link_libraries(ina226_test ina226)
  endif ()
endif ()
