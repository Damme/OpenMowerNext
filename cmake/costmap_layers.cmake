#
# costmap_layers - Nav2 costmap layer plugins (RimCostLayer)
#
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

add_library(costmap_layers SHARED src/costmap_layers/rim_cost_layer.cpp)
target_compile_features(costmap_layers PUBLIC cxx_std_17)
target_include_directories(costmap_layers PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
ament_target_dependencies(costmap_layers nav2_costmap_2d pluginlib rclcpp)
pluginlib_export_plugin_description_file(nav2_costmap_2d src/costmap_layers/plugins.xml)
install(TARGETS costmap_layers DESTINATION lib)
