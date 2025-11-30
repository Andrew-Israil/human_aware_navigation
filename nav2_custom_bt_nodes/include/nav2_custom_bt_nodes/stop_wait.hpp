#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__STOP_WAIT_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__STOP_WAIT_HPP_

#include <string>
#include <memory>
#include <limits>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_custom_bt_nodes
{

/**
 * @brief A BT::DecoratorNode that ticks its child everytime when the length of
 * the new path is smaller than the old one by the length given by the user.
 */
class StopAndWait : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::StopAndWait
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  StopAndWait(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("cancel_control"),
      BT::InputPort<int>("waiting_time")
    };
  }

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

private:
  bool cancel_control = false;
  int waiting_time;
  rclcpp::Node::SharedPtr node_;
  bool first_time_ = true;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__DECORATOR__STOP_WAIT_HPP_