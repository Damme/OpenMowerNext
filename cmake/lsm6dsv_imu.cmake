###
# lsm6dsv_imu - LSM6DSV I2C IMU (Worx robot) publishing imu/data_raw
###
add_library(lsm6dsv STATIC src/lsm6dsv_imu/lsm6dsv.cpp)
target_compile_features(lsm6dsv PUBLIC cxx_std_17)
target_include_directories(lsm6dsv PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)

add_executable(lsm6dsv_imu_node src/lsm6dsv_imu/lsm6dsv_imu_node.cpp)
target_link_libraries(lsm6dsv_imu_node lsm6dsv)
ament_target_dependencies(lsm6dsv_imu_node rclcpp sensor_msgs)
install(TARGETS lsm6dsv_imu_node DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(lsm6dsv_test test/lsm6dsv_test.cpp)
  if (TARGET lsm6dsv_test)
    target_link_libraries(lsm6dsv_test lsm6dsv)
  endif ()
endif ()
