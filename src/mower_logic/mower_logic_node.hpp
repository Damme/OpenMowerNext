// mower_logic as a component: the node, its services and the behaviour tree
// ticked in its own thread. Used by the mower_logic executable and by
// om_container (open_mower_next::mower_logic::MowerLogicNode).
#pragma once

#include "mower_logic/context.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

namespace open_mower_next::mower_logic
{

class MowerLogicNode
{
public:
  explicit MowerLogicNode(const rclcpp::NodeOptions & options);
  ~MowerLogicNode();

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
  {
    return node_->get_node_base_interface();
  }
  rclcpp::Node::SharedPtr node() const { return node_; }

private:
  void run(const std::string & tree_file, double rate, bool log_tree);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<Context> ctx_;
  std::vector<rclcpp::ServiceBase::SharedPtr> services_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  std::atomic<bool> stop_{false};
  std::thread tick_thread_;
};

}  // namespace open_mower_next::mower_logic
