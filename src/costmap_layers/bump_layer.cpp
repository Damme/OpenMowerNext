#include "costmap_layers/bump_layer.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <limits>

namespace open_mower_next::costmap_layers
{

void BumpLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) throw std::runtime_error("BumpLayer: node expired");
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/mower_logic/bump_obstacles")));
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();
  const auto topic = node->get_parameter(name_ + ".topic").as_string();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  global_frame_ = layered_costmap_->getGlobalFrameID();
  // Latched: a restarted Nav2 gets the current list at once (mower_logic also
  // republishes every second).
  sub_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) { onCloud(m); });
  current_ = true;
}

void BumpLayer::onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (msg->header.frame_id != global_frame_) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 10000, "BumpLayer: points in '%s', costmap in '%s' - ignored",
                         msg->header.frame_id.c_str(), global_frame_.c_str());
    return;
  }
  std::vector<std::pair<double, double>> pts;
  pts.reserve(msg->width * msg->height);
  for (sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"); x != x.end(); ++x, ++y) {
    pts.emplace_back(*x, *y);
  }
  std::lock_guard<std::mutex> l(mutex_);
  if (pts == points_) return;  // republished unchanged: nothing to redraw
  double min_x = std::numeric_limits<double>::max(), min_y = min_x;
  double max_x = std::numeric_limits<double>::lowest(), max_y = max_x;
  auto grow = [&](const std::vector<std::pair<double, double>> & v) {
    for (const auto & [px, py] : v) {
      min_x = std::min(min_x, px);
      min_y = std::min(min_y, py);
      max_x = std::max(max_x, px);
      max_y = std::max(max_y, py);
    }
  };
  grow(points_);  // cells that were marked must be redrawn (cleared) too
  grow(pts);
  if (dirty_) {
    min_x = std::min(min_x, dirty_min_x_);
    min_y = std::min(min_y, dirty_min_y_);
    max_x = std::max(max_x, dirty_max_x_);
    max_y = std::max(max_y, dirty_max_y_);
  }
  points_ = std::move(pts);
  if (min_x <= max_x) {
    dirty_ = true;
    dirty_min_x_ = min_x;
    dirty_min_y_ = min_y;
    dirty_max_x_ = max_x;
    dirty_max_y_ = max_y;
  }
}

void BumpLayer::updateBounds(double, double, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> l(mutex_);
  if (!dirty_) return;
  dirty_ = false;
  // One cell of margin: points sit anywhere inside their cell.
  const double m = layered_costmap_->getCostmap()->getResolution();
  *min_x = std::min(*min_x, dirty_min_x_ - m);
  *min_y = std::min(*min_y, dirty_min_y_ - m);
  *max_x = std::max(*max_x, dirty_max_x_ + m);
  *max_y = std::max(*max_y, dirty_max_y_ + m);
}

void BumpLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;
  // The master grid was reset inside the window: write every point that falls in it.
  std::lock_guard<std::mutex> l(mutex_);
  for (const auto & [px, py] : points_) {
    unsigned int mx = 0, my = 0;
    if (!master_grid.worldToMap(px, py, mx, my)) continue;
    const int i = static_cast<int>(mx), j = static_cast<int>(my);
    if (i < min_i || i >= max_i || j < min_j || j >= max_j) continue;
    master_grid.setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  }
}

}  // namespace open_mower_next::costmap_layers

PLUGINLIB_EXPORT_CLASS(open_mower_next::costmap_layers::BumpLayer, nav2_costmap_2d::Layer)
