#include "costmap_layers/rim_cost_layer.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace open_mower_next::costmap_layers
{

void RimCostLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) throw std::runtime_error("RimCostLayer: node expired");
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("from_cost", rclcpp::ParameterValue(152));
  declareParameter("to_cost", rclcpp::ParameterValue(static_cast<int>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)));
  enabled_ = node->get_parameter(name_ + ".enabled").as_bool();
  from_ = static_cast<unsigned char>(node->get_parameter(name_ + ".from_cost").as_int());
  to_ = static_cast<unsigned char>(node->get_parameter(name_ + ".to_cost").as_int());
  current_ = true;
}

void RimCostLayer::updateBounds(double, double, double, double *, double *, double *, double *)
{
  // Only rewrites cells the other layers touched; doesn't grow the update window.
}

void RimCostLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) return;
  unsigned char * costs = master_grid.getCharMap();
  const unsigned int size_x = master_grid.getSizeInCellsX();
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      unsigned char & c = costs[static_cast<unsigned int>(j) * size_x + static_cast<unsigned int>(i)];
      if (c == from_) c = to_;
    }
  }
}

}  // namespace open_mower_next::costmap_layers

PLUGINLIB_EXPORT_CLASS(open_mower_next::costmap_layers::RimCostLayer, nav2_costmap_2d::Layer)
