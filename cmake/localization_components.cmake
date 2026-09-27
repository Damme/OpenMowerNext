###
# localization_components - robot_localization's EKF and navsat_transform as components
###
add_library(localization_components SHARED src/localization_components/localization_components.cpp)
target_compile_features(localization_components PUBLIC cxx_std_17)
find_package(Eigen3 REQUIRED)
target_include_directories(localization_components PRIVATE ${EIGEN3_INCLUDE_DIRS})
# robot_localization's headers include these without exporting them.
find_package(geographic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(message_filters REQUIRED)
ament_target_dependencies(localization_components rclcpp rclcpp_components robot_localization nav_msgs sensor_msgs
  geometry_msgs geographic_msgs std_srvs tf2 tf2_ros tf2_geometry_msgs diagnostic_updater message_filters)
# rl_lib (the filters) is exported as a plain library name, which ament_target_dependencies drops.
find_library(RL_LIB rl_lib PATHS ${robot_localization_DIR}/../../../lib REQUIRED)
target_link_libraries(localization_components ${RL_LIB})
rclcpp_components_register_nodes(localization_components
  "open_mower_next::localization_components::Ekf"
  "robot_localization::NavSatTransform")
install(TARGETS localization_components DESTINATION lib)
