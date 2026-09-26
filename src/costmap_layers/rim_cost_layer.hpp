#pragma once
// Rewrites one cost value to another in the master costmap. Used for the rim
// around the mowing areas (and inside exclusions): map_server paints it with
// grid.edge_band_value, which the static layer turns into a cost just below
// INSCRIBED_INFLATED_OBSTACLE; this layer lifts it to exactly 253. Planners and
// collision checkers then keep the robot CENTRE (GPS antenna) out of the rim but
// let the footprint overhang into it - the recorded boundary is where the
// antenna drove, the rim (half the mower's width) is where its body was.

#include <nav2_costmap_2d/layer.hpp>

namespace open_mower_next::costmap_layers
{

class RimCostLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y, double * max_x,
                    double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override {}
  bool isClearable() override { return false; }

private:
  unsigned char from_ = 251, to_ = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
};

}  // namespace open_mower_next::costmap_layers
