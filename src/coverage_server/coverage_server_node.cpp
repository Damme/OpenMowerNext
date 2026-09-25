#include "coverage_server_node.hpp"

#include "utils.h"

#include <cmath>

namespace open_mower_next::coverage_server
{
CoverageServerNode::CoverageServerNode(const rclcpp::NodeOptions & options)
: Node("coverage_server", options)
{
  RCLCPP_INFO(get_logger(), "Starting Coverage Server node");

  declarePlannerParameters();

  RCLCPP_INFO(
    get_logger(), "Configured with tool_width=%.3f, overlap=%.2f, clearance=%.3f, fill_mode=%s",
    planner_params_.tool_width, planner_params_.overlap, planner_params_.clearance,
    planner_params_.fill_mode.c_str());

  area_coverage_service_ = create_service<open_mower_next::srv::AreaCoverage>(
    "area_coverage", std::bind(
                       &CoverageServerNode::handleAreaCoverageRequest, this, std::placeholders::_1,
                       std::placeholders::_2, std::placeholders::_3));

  // Setup map subscription
  map_subscription_ = create_subscription<open_mower_next::msg::Map>(
    "/mowing_map", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal),
    std::bind(&CoverageServerNode::mapCallback, this, std::placeholders::_1));

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "coverage/path", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));
  visualization_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "coverage/visualization", rclcpp::QoS(10).durability(rclcpp::DurabilityPolicy::TransientLocal));

  RCLCPP_INFO(get_logger(), "Coverage Server initialized and ready");
}

CoverageServerNode::~CoverageServerNode()
{
  RCLCPP_INFO(get_logger(), "Shutting down Coverage Server node");
}

// Same names and defaults as the ROS1 coverage_planner node, except
// outline_count: -1 so the request's headland_loops is used.
void CoverageServerNode::declarePlannerParameters()
{
  auto & p = planner_params_;
  p.lane_skip = static_cast<int>(declare_parameter("lane_skip", 2));
  p.optimize_sweep_angle = declare_parameter("optimize_sweep_angle", true);
  p.blade_offset_x = declare_parameter("blade_offset_x", 0.0);
  p.blade_offset_y = declare_parameter("blade_offset_y", 0.0);
  p.tool_width = declare_parameter("tool_width", 0.22);
  p.overlap = declare_parameter("overlap", 0.35);
  p.clearance = declare_parameter("clearance", 0.04);
  p.obstacle_clearance = declare_parameter("obstacle_clearance", -1.0);
  p.outline_count = static_cast<int>(declare_parameter("outline_count", -1));
  p.fill_mode = declare_parameter("fill_mode", std::string("request"));
  p.path_spacing = declare_parameter("path_spacing", 0.1);
  p.body_width = declare_parameter("body_width", 0.39);
  p.body_length = declare_parameter("body_length", 0.58);
  p.min_turn_radius = declare_parameter("min_turn_radius", 0.0);
  p.loop_transition = declare_parameter("loop_transition", 6.0);
  p.edge_side = declare_parameter("edge_side", std::string("right"));
  p.axle_from_rear = declare_parameter("axle_from_rear", 0.11);
}

void CoverageServerNode::mapCallback(const open_mower_next::msg::Map::SharedPtr msg)
{
  current_map_ = *msg;
  RCLCPP_INFO(get_logger(), "Received updated map with %zu areas", msg->areas.size());
}

void CoverageServerNode::handleAreaCoverageRequest(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<open_mower_next::srv::AreaCoverage::Request> request,
  std::shared_ptr<open_mower_next::srv::AreaCoverage::Response> response)
{
  (void)request_header;

  if (request->area_id.empty()) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area ID cannot be empty";
    return;
  }

  RCLCPP_INFO(
    get_logger(), "Received area coverage request for area ID: %s", request->area_id.c_str());

  const auto area = findAreaById(request->area_id);

  if (area == nullptr) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area not found";
    return;
  }

  if (area.get()->type != open_mower_next::msg::Area::TYPE_OPERATION) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area is not of type OPERATION";
    return;
  }

  const auto & area_polygon = area.get()->area;

  if (!utils::isValid(area_polygon.polygon)) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_AREA;
    response->message = "Area has an invalid polygon";
    return;
  }

  // Find exclusions if requested
  std::vector<geometry_msgs::msg::PolygonStamped> exclusion_polygons;
  if (request->with_exclusions) {
    std::string exclusion_message;
    if (!findExclusionsInPolygon(area_polygon, exclusion_polygons, exclusion_message)) {
      response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_EXCLUSION;
      response->message = exclusion_message;
      return;
    }
  }

  ::coverage_planner::Request plan_request;
  plan_request.outline = utils::toPoints(area_polygon.polygon);
  for (const auto & exclusion : exclusion_polygons) {
    plan_request.holes.push_back(utils::toPoints(exclusion.polygon));
  }
  plan_request.angle_rad = request->swath_angle * M_PI / 180.0;
  plan_request.outline_count = request->headland_loops;

  const auto result = ::coverage_planner::plan(
    plan_request, planner_params_,
    [this](::coverage_planner::LogLevel level, const std::string & text) {
      switch (level) {
        case ::coverage_planner::LogLevel::ERROR:
          RCLCPP_ERROR(get_logger(), "%s", text.c_str());
          break;
        case ::coverage_planner::LogLevel::WARN:
          RCLCPP_WARN(get_logger(), "%s", text.c_str());
          break;
        default:
          RCLCPP_INFO(get_logger(), "%s", text.c_str());
          break;
      }
    });

  const auto & frame_id = area_polygon.header.frame_id;
  response->area_id = request->area_id;
  response->coverage_geometry = utils::toMsg(result.mowable, frame_id);

  if (result.mowable.empty()) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_INVALID_EXCLUSION;
    response->message = "Exclusions remove the entire requested area";
    return;
  }

  if (result.paths.empty()) {
    response->code = open_mower_next::srv::AreaCoverage::Response::CODE_UNKNOWN_ERROR;
    response->message = "Planner produced no paths (area narrower than the clearances?)";
    return;
  }

  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp = now();

  for (const auto & planned : result.paths) {
    response->paths.push_back(utils::toMsg(planned, header));
  }
  response->path = utils::concatenate(response->paths, header);

  response->message = "Coverage path generated successfully";
  response->code = open_mower_next::srv::AreaCoverage::Response::CODE_SUCCESS;

  path_pub_->publish(response->path);
  visualization_pub_->publish(createVisualizationMarkers(response->paths, frame_id));

  RCLCPP_INFO(
    get_logger(), "Generated %zu paths (%zu poses) for area ID: %s", response->paths.size(),
    response->path.poses.size(), request->area_id.c_str());
}

msg::Area::SharedPtr CoverageServerNode::findAreaById(const std::string & area_id)
{
  for (const auto & area : current_map_.areas) {
    if (area.id == area_id) {
      RCLCPP_INFO(
        get_logger(), "Found area with ID: %s, name: %s", area_id.c_str(), area.name.c_str());

      return std::make_shared<msg::Area>(area);
    }
  }

  return nullptr;
}

bool CoverageServerNode::findExclusionsInPolygon(
  const geometry_msgs::msg::PolygonStamped & field_polygon,
  std::vector<geometry_msgs::msg::PolygonStamped> & exclusion_polygons, std::string & message)
{
  exclusion_polygons.clear();
  message.clear();

  for (const auto & area : current_map_.areas) {
    if (area.type == open_mower_next::msg::Area::TYPE_EXCLUSION) {
      if (!utils::isValid(area.area.polygon)) {
        message = "Exclusion area " + area.id + " has an invalid polygon";
        return false;
      }

      if (!utils::intersects(field_polygon.polygon, area.area.polygon)) {
        RCLCPP_INFO(
          get_logger(), "Skipping exclusion area outside field: %s, name: %s", area.id.c_str(),
          area.name.c_str());
        continue;
      }

      geometry_msgs::msg::PolygonStamped exclusion;
      exclusion.header = field_polygon.header;
      exclusion.polygon = area.area.polygon;
      exclusion_polygons.push_back(exclusion);
      RCLCPP_INFO(
        get_logger(), "Applying exclusion area with ID: %s, name: %s", area.id.c_str(),
        area.name.c_str());
    }
  }

  RCLCPP_INFO(get_logger(), "Found %zu exclusion areas", exclusion_polygons.size());
  return true;
}

// One LINE_STRIP per pass, namespaced by kind.
visualization_msgs::msg::MarkerArray CoverageServerNode::createVisualizationMarkers(
  const std::vector<open_mower_next::msg::CoveragePath> & paths, const std::string & frame_id)
{
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker delete_marker;
  delete_marker.header.frame_id = frame_id;
  delete_marker.header.stamp = now();
  delete_marker.ns = "all";
  delete_marker.id = 0;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_marker);

  for (size_t i = 0; i < paths.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    switch (paths[i].kind) {
      case open_mower_next::msg::CoveragePath::KIND_PERIMETER:
        marker.ns = "perimeter";
        marker.color.g = 1.0f;
        break;
      case open_mower_next::msg::CoveragePath::KIND_OBSTACLE:
        marker.ns = "obstacle";
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        break;
      default:
        marker.ns = "fill";
        marker.color.r = 1.0f;
        break;
    }
    marker.color.a = 1.0f;
    marker.header.frame_id = frame_id;
    marker.header.stamp = now();
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.02;
    for (const auto & pose : paths[i].path.poses) {
      marker.points.push_back(pose.pose.position);
    }
    markers.markers.push_back(marker);
  }

  return markers;
}

}  // namespace open_mower_next::coverage_server
