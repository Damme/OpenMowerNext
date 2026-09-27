###
# lsm6dsv_imu - LSM6DSV I2C IMU (Worx robot) publishing imu/data_raw
###
add_library(lsm6dsv STATIC src/lsm6dsv_imu/lsm6dsv.cpp)
target_compile_features(lsm6dsv PUBLIC cxx_std_17)
target_include_directories(lsm6dsv PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)

set_target_properties(lsm6dsv PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(lsm6dsv_imu_component SHARED src/lsm6dsv_imu/lsm6dsv_imu_node.cpp)
target_link_libraries(lsm6dsv_imu_component lsm6dsv)
ament_target_dependencies(lsm6dsv_imu_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(lsm6dsv_imu_component
  PLUGIN "open_mower_next::lsm6dsv_imu::Lsm6dsvImuNode"
  EXECUTABLE lsm6dsv_imu_node)
install(TARGETS lsm6dsv_imu_component DESTINATION lib)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(lsm6dsv_test test/lsm6dsv_test.cpp)
  if (TARGET lsm6dsv_test)
    target_link_libraries(lsm6dsv_test lsm6dsv)
  endif ()
endif ()
