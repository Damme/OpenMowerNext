###
# mower_logic - mowing mission behaviour tree executor
###
find_package(nav2_msgs REQUIRED)

add_library(mower_logic_core STATIC src/mower_logic/mission.cpp)
target_compile_features(mower_logic_core PUBLIC cxx_std_17)
target_include_directories(mower_logic_core PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(mower_logic_core "${cpp_typesupport_target}")
ament_target_dependencies(mower_logic_core nav_msgs)
add_dependencies(mower_logic_core ${PROJECT_NAME})

add_library(mower_logic_component SHARED
  src/mower_logic/mower_logic_node.cpp
  src/mower_logic/context.cpp
  src/mower_logic/bt_nodes.cpp
)
set_target_properties(mower_logic_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_link_libraries(mower_logic_component mower_logic_core "${cpp_typesupport_target}")
ament_target_dependencies(mower_logic_component
  rclcpp rclcpp_action rclcpp_components behaviortree_cpp ament_index_cpp nav2_msgs nav_msgs geometry_msgs sensor_msgs
  std_msgs std_srvs tf2 tf2_ros)
rclcpp_components_register_nodes(mower_logic_component "open_mower_next::mower_logic::MowerLogicNode")

add_executable(mower_logic src/mower_logic/main.cpp)
target_link_libraries(mower_logic mower_logic_component)
install(TARGETS mower_logic DESTINATION lib/${PROJECT_NAME})
install(TARGETS mower_logic_component DESTINATION lib)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(mower_logic_mission_test test/mower_logic_mission_test.cpp)
  if (TARGET mower_logic_mission_test)
    target_link_libraries(mower_logic_mission_test mower_logic_core)
  endif ()
endif ()
