###
# worx_test - measured test drives on the real robot (drive / turn / square)
###
add_executable(worx_test src/worx_test/worx_test.cpp)
target_compile_features(worx_test PUBLIC cxx_std_17)
ament_target_dependencies(worx_test rclcpp geometry_msgs sensor_msgs std_srvs)
INSTALL(TARGETS worx_test DESTINATION lib/${PROJECT_NAME})
