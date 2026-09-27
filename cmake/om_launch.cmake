###
# om_launch - process supervisor for launch manifests (no ROS libraries)
# om_container - hosts a manifest process's components (replaces the python
#   launch, component_container, ros2_control_node and the controller spawners)
###
find_package(yaml-cpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

add_executable(om_launch src/om_launch/om_launch.cpp)
target_include_directories(om_launch PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(om_launch yaml-cpp)

add_executable(om_container src/om_launch/om_container.cpp)
target_include_directories(om_container PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
target_link_libraries(om_container yaml-cpp)
ament_target_dependencies(om_container
  rclcpp rclcpp_components class_loader ament_index_cpp controller_manager controller_manager_msgs realtime_tools)

install(TARGETS om_launch om_container DESTINATION lib/${PROJECT_NAME})

# URDF and controller parameters for the manifests, generated at build time
# (the robot runs no xacro/python).
set(OM_GENERATED ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(GLOB OM_XACRO ${CMAKE_CURRENT_SOURCE_DIR}/description/*.xacro)
add_custom_command(
  OUTPUT ${OM_GENERATED}/worx_spidev.urdf ${OM_GENERATED}/worx_fake.urdf
         ${OM_GENERATED}/worx_controllers.yaml ${OM_GENERATED}/worx_controllers_sim.yaml
  COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/scripts/om_generate.py ${CMAKE_CURRENT_SOURCE_DIR} ${OM_GENERATED}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/om_generate.py ${OM_XACRO}
          ${CMAKE_CURRENT_SOURCE_DIR}/config/controllers.yaml ${CMAKE_CURRENT_SOURCE_DIR}/config/hardware/worx.yaml
  COMMENT "Generating Worx URDF and controller parameters")
add_custom_target(om_generated ALL DEPENDS ${OM_GENERATED}/worx_spidev.urdf)
install(DIRECTORY ${OM_GENERATED}/ DESTINATION share/${PROJECT_NAME}/generated)
