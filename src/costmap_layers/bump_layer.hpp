#pragma once
// Obstacles mower_logic found by bumping into them (~/bump_obstacles, a
// PointCloud2 in the costmap's frame): LETHAL at each point.
//
// Unlike nav2's ObstacleLayer every message REPLACES the previous set:
// mower_logic owns the list, and a mark it forgets (perimeter bump turned into
// an edge correction, forgotten in the web UI, new mission) or leaves out
// under the robot is gone from the costmap with the next message. The
// ObstacleLayer (clearing: false) kept every cell it was ever sent: on
// 2026-09-29 the robot stood in such a forgotten mark and no transit or
// docking plan could start ("Start occupied").
//
// No grid of its own (the global costmap is 1.5 M cells): a list of points.

#include <nav2_costmap_2d/layer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace open_mower_next::costmap_layers
{

class BumpLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
                    double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override {}
  // mower_logic's list is the truth: clearing the costmap must not drop marks
  // (they would only come back with the next message anyway).
  bool isClearable() override { return false; }

private:
  void onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  std::mutex mutex_;
  std::vector<std::pair<double, double>> points_;
  // Area that changed since the last updateBounds (old and new points).
  bool dirty_ = false;
  double dirty_min_x_ = 0, dirty_min_y_ = 0, dirty_max_x_ = 0, dirty_max_y_ = 0;
  std::string global_frame_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Logger logger_{rclcpp::get_logger("bump_layer")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace open_mower_next::costmap_layers
