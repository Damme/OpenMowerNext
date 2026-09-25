###
# ftc_controller - FTC path controller + goal checker plugins for Nav2
###
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(ftc_controller SHARED src/ftc_controller/ftc_controller.cpp)
target_compile_features(ftc_controller PUBLIC cxx_std_17)
target_include_directories(ftc_controller PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src> ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(ftc_controller
  nav2_core nav2_costmap_2d nav2_util pluginlib rclcpp rclcpp_lifecycle geometry_msgs nav_msgs tf2 tf2_eigen tf2_geometry_msgs tf2_ros)
pluginlib_export_plugin_description_file(nav2_core src/ftc_controller/plugins.xml)
install(TARGETS ftc_controller DESTINATION lib)
