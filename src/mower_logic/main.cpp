#include "mower_logic/mower_logic_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    open_mower_next::mower_logic::MowerLogicNode mower_logic{rclcpp::NodeOptions()};
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(mower_logic.get_node_base_interface());
    exec.spin();
  }
  rclcpp::shutdown();
  return 0;
}
