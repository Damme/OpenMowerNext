###
# coverage_server
###
set_target_properties(coverage_planner PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(coverage_server_component SHARED src/coverage_server/coverage_server_node.cpp)
target_compile_features(coverage_server_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(coverage_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(coverage_server_component
  "${cpp_typesupport_target}"
  coverage_planner)
ament_target_dependencies(coverage_server_component
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
  visualization_msgs
)
add_dependencies(coverage_server_component ${PROJECT_NAME})
rclcpp_components_register_nodes(coverage_server_component "open_mower_next::coverage_server::CoverageServerNode")

add_executable(coverage_server src/coverage_server/main.cpp)
target_link_libraries(coverage_server coverage_server_component)

INSTALL(TARGETS coverage_server DESTINATION lib/${PROJECT_NAME})
INSTALL(TARGETS coverage_server_component DESTINATION lib)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(coverage_server_utils_test test/coverage_server_utils_test.cpp)
  if (TARGET coverage_server_utils_test)
    target_include_directories(coverage_server_utils_test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    )
    target_link_libraries(coverage_server_utils_test
      "${cpp_typesupport_target}"
      coverage_planner
    )
    add_dependencies(coverage_server_utils_test ${PROJECT_NAME})
    ament_target_dependencies(coverage_server_utils_test
      geometry_msgs
      nav_msgs
      tf2_geometry_msgs
    )
  endif ()
endif ()
