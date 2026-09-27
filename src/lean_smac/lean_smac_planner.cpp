// Smac lattice planner that gives its search memory back after every plan.
//
// nav2_smac_planner (Jazzy) keeps the A* graph of the last search until the
// next plan starts, and reserves the obstacle heuristic queue for every cell
// of the costmap (16 bytes x 1.5 M cells = 25 MB at 5 cm on the Worx map),
// kept for the heuristic cache. On the Pi Zero 2 (512 MB) a failed search on
// the big map held ~100 MB this way. Here both are released when createPlan
// returns (or throws); with cache_obstacle_heuristic false the heuristic
// table (4 bytes per cell) goes too.
#include <nav2_smac_planner/smac_planner_lattice.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace open_mower_next::lean_smac
{

class LeanSmacPlannerLattice : public nav2_smac_planner::SmacPlannerLattice
{
public:
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override
  {
    struct Release
    {
      LeanSmacPlannerLattice * self;
      ~Release() { self->releaseSearchMemory(); }
    } release{this};
    return SmacPlannerLattice::createPlan(start, goal, cancel_checker);
  }

private:
  void releaseSearchMemory()
  {
    std::lock_guard<std::mutex> lock(_mutex);  // as createPlan / the parameter callback
    // setCollisionChecker() swaps the graph for an empty one (it is what
    // createPlan does first anyway); the costmap size is unchanged here.
    _a_star->setCollisionChecker(&_collision_checker);
    if (!_search_info.cache_obstacle_heuristic) {
      nav2_smac_planner::ObstacleHeuristicQueue().swap(nav2_smac_planner::NodeHybrid::obstacle_heuristic_queue);
      nav2_smac_planner::LookupTable().swap(nav2_smac_planner::NodeHybrid::obstacle_heuristic_lookup_table);
    } else {
      // The queue is the frontier the cached heuristic continues from: keep
      // its contents, drop only the unused reservation.
      nav2_smac_planner::NodeHybrid::obstacle_heuristic_queue.shrink_to_fit();
    }
  }
};

}  // namespace open_mower_next::lean_smac

PLUGINLIB_EXPORT_CLASS(open_mower_next::lean_smac::LeanSmacPlannerLattice, nav2_core::GlobalPlanner)
