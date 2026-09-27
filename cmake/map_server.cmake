###
# map_server_node
###
add_library(map_server_component SHARED
        src/map_server/map_server_node.hpp
        src/map_server/map_server_node.cpp
        src/map_server/geo_json_map.cpp
        src/map_server/geo_json_map.hpp
        src/map_server/polygon_iterator.hpp
        src/map_server/some_gaussian_filter.hpp
        src/map_server/polygon_utils.hpp)
target_compile_features(map_server_component PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(map_server_component PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<INSTALL_INTERFACE:include>
)
target_link_libraries(map_server_component "${cpp_typesupport_target}")


ament_target_dependencies(map_server_component
        rclcpp_components
        std_msgs
        geometry_msgs
        nav_msgs
        foxglove_msgs
        visualization_msgs
        rclcpp
        robot_localization
        tf2
        tf2_geometry_msgs
        unique_identifier_msgs
)

target_link_libraries(map_server_component
        nlohmann_json::nlohmann_json
        ${GeographicLib_LIBRARIES}
)

add_dependencies(map_server_component ${PROJECT_NAME})
rclcpp_components_register_nodes(map_server_component "open_mower_next::map_server::MapServerNode")

add_executable(map_server_node src/map_server/node_main.cpp)
target_link_libraries(map_server_node map_server_component)

INSTALL(TARGETS map_server_node DESTINATION lib/${PROJECT_NAME})
INSTALL(TARGETS map_server_component DESTINATION lib)