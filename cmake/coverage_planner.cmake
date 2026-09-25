###
# coverage_planner - ROS-agnostic Boost.Geometry coverage planner by Daniel Wiegert
###
find_package(Boost REQUIRED)

add_library(coverage_planner STATIC
  src/coverage_planner/coverage_planner.cpp
)
target_compile_features(coverage_planner PUBLIC cxx_std_17)
# Boost.Geometry asserts are debug-only noise here; the planner guards every op itself.
target_compile_definitions(coverage_planner PRIVATE NDEBUG)
set_target_properties(coverage_planner PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(coverage_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(coverage_planner PUBLIC Boost::boost)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(coverage_planner_test test/coverage_planner_test.cpp)
  if (TARGET coverage_planner_test)
    target_link_libraries(coverage_planner_test coverage_planner)
  endif ()
endif ()
