// robot_localization's EKF and navsat_transform as rclcpp components, so they
// can share a process (om_launch manifests) instead of costing one each.
// robot_localization (Jazzy) ships them only as executables.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <robot_localization/navsat_transform.hpp>
#include <robot_localization/ros_filter_types.hpp>

namespace open_mower_next::localization_components
{

// As robot_localization's ekf_node: construct, then initialize() (which needs
// the node to be owned by a shared_ptr already).
class Ekf
{
public:
  explicit Ekf(const rclcpp::NodeOptions & options)
  : filter_(std::make_shared<robot_localization::RosEkf>(withDefaultName(options)))
  {
    filter_->initialize();
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
  {
    return filter_->get_node_base_interface();
  }

private:
  // RosFilter takes its node name from the first argument (ekf_node passes
  // "ekf_filter_node"); a __node remap still renames it.
  static rclcpp::NodeOptions withDefaultName(rclcpp::NodeOptions options)
  {
    auto args = options.arguments();
    args.insert(args.begin(), "ekf_filter_node");
    options.arguments(args);
    return options;
  }

  std::shared_ptr<robot_localization::RosEkf> filter_;
};

}  // namespace open_mower_next::localization_components

RCLCPP_COMPONENTS_REGISTER_NODE(open_mower_next::localization_components::Ekf)
RCLCPP_COMPONENTS_REGISTER_NODE(robot_localization::NavSatTransform)
