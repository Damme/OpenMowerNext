###
# lean_smac - Smac lattice planner that releases its search memory after each plan
###
find_package(nav2_smac_planner REQUIRED)

add_library(lean_smac_planner SHARED src/lean_smac/lean_smac_planner.cpp)
target_compile_features(lean_smac_planner PUBLIC cxx_std_17)
ament_target_dependencies(lean_smac_planner nav2_smac_planner nav2_core pluginlib)
pluginlib_export_plugin_description_file(nav2_core src/lean_smac/plugins.xml)
install(TARGETS lean_smac_planner DESTINATION lib)
