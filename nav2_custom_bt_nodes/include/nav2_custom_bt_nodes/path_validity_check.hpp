#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__PATH_VALIDITY_CHECK_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__PATH_VALIDITY_CHECK_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "conflict_type_msgs/srv/get_conflict_type.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

namespace nav2_custom_bt_nodes
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the IsPathValid
 * service returns true and FAILURE otherwise
 */
class PathValidityCheck : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_custom_BT_nodes::IsPathValidCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  PathValidityCheck(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  PathValidityCheck() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Function to read parameters and initialize class variables
   */
  void initialize();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to Check"),
      BT::InputPort<std::chrono::milliseconds>("server_timeout"),
      BT::OutputPort<bool>("cancel_control"),
      BT::OutputPort<double>("waiting_time")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
  rclcpp::Client<conflict_type_msgs::srv::GetConflictType>::SharedPtr conflict_client_;
  // The timeout value while waiting for a responce from the
  // is path valid service
  std::chrono::milliseconds server_timeout_;
  bool initialized_;
  bool cancel_control;
  double waiting_time;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__PATH_VALIDITY_CHECK_HPP_