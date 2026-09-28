###
# web_ui - control page on the robot's VPN address (Boost.Beast HTTP + WebSocket)
###
find_package(Boost REQUIRED)

# The page is compiled in: nothing to install or find at runtime.
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/web_ui/index.html)
file(READ ${CMAKE_CURRENT_SOURCE_DIR}/src/web_ui/index.html WEB_UI_INDEX_HTML)
configure_file(src/web_ui/index_html.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/web_ui/index_html.cpp @ONLY)

add_library(web_ui_component SHARED
  src/web_ui/web_ui_node.cpp
  src/web_ui/web_server.cpp
  ${CMAKE_CURRENT_BINARY_DIR}/web_ui/index_html.cpp)
target_compile_features(web_ui_component PUBLIC cxx_std_17)
target_include_directories(web_ui_component PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(web_ui_component "${cpp_typesupport_target}" Boost::boost nlohmann_json::nlohmann_json
  Threads::Threads)
ament_target_dependencies(web_ui_component rclcpp rclcpp_action rclcpp_components geometry_msgs nav_msgs
  sensor_msgs std_msgs std_srvs)
# Size (Pi memory): exported Beast/asio template names alone were ~1.9 MB of symbol
# strings; class_loader finds the component through its registration, not by symbol.
# Nothing here is speed critical: -Os.
set_target_properties(web_ui_component PROPERTIES CXX_VISIBILITY_PRESET hidden VISIBILITY_INLINES_HIDDEN ON)
target_compile_options(web_ui_component PRIVATE -Os -ffunction-sections -fdata-sections)
target_link_options(web_ui_component PRIVATE -Wl,--gc-sections)
add_dependencies(web_ui_component ${PROJECT_NAME})
rclcpp_components_register_nodes(web_ui_component "open_mower_next::web_ui::WebUiNode")
install(TARGETS web_ui_component DESTINATION lib)
