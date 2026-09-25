#pragma once

#include "coverage_planner/coverage_planner.hpp"

#include "open_mower_next/msg/coverage_path.hpp"
#include "open_mower_next/msg/map.hpp"
#include "open_mower_next/srv/area_coverage.hpp"
#include "open_mower_next/srv/polygon_coverage.hpp"

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace open_mower_next::coverage_server
{

class CoverageServerNode : public rclcpp::Node
{
public:
  explicit CoverageServerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CoverageServerNode();

private:
  ::coverage_planner::Params planner_params_;

  rclcpp::Service<open_mower_next::srv::AreaCoverage>::SharedPtr area_coverage_service_;

  rclcpp::Subscription<open_mower_next::msg::Map>::SharedPtr map_subscription_;
  open_mower_next::msg::Map current_map_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  void declarePlannerParameters();

  void handleAreaCoverageRequest(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<open_mower_next::srv::AreaCoverage::Request> request,
    std::shared_ptr<open_mower_next::srv::AreaCoverage::Response> response);

  void mapCallback(const open_mower_next::msg::Map::SharedPtr msg);

  bool findExclusionsInPolygon(
    const geometry_msgs::msg::PolygonStamped & field_polygon,
    std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons, std::string & message);

  msg::Area::SharedPtr findAreaById(const std::string & area_id);

  visualization_msgs::msg::MarkerArray createVisualizationMarkers(
    const std::vector<open_mower_next::msg::CoveragePath> & paths, const std::string & frame_id);
};

}  // namespace open_mower_next::coverage_server
