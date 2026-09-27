###
# lean_smac - Smac lattice planner that releases its search memory after each plan
###
find_package(nav2_smac_planner REQUIRED)
find_package(ompl REQUIRED)

add_library(lean_smac_planner SHARED src/lean_smac/lean_smac_planner.cpp)
target_compile_features(lean_smac_planner PUBLIC cxx_std_17)
# Only the lattice library: nav2_smac_planner's hybrid, 2D and lattice libraries each
# define the same static members (NodeHybrid::obstacle_heuristic_*), so loading all
# three costs memory and destroys those objects several times at exit.
find_library(SMAC_LATTICE_LIB nav2_smac_planner_lattice PATHS ${nav2_smac_planner_DIR}/../../../lib NO_DEFAULT_PATH REQUIRED)
target_include_directories(lean_smac_planner PRIVATE ${nav2_smac_planner_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})
target_link_libraries(lean_smac_planner ${SMAC_LATTICE_LIB})
ament_target_dependencies(lean_smac_planner
  nav2_core nav2_costmap_2d nav2_util pluginlib rclcpp rclcpp_lifecycle nav_msgs geometry_msgs visualization_msgs
  tf2 tf2_ros)
pluginlib_export_plugin_description_file(nav2_core src/lean_smac/plugins.xml)
install(TARGETS lean_smac_planner DESTINATION lib)
