#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CANCEL_CONTROL_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CANCEL_CONTROL_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_custom_bt_nodes
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS when the cancel control condition 
 * from path_validity_check is true and FAILURE otherwise
 */
class CancelControlCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_custom_bt_nodes::CancelControlCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
    CancelControlCondition(const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    CancelControlCondition() = delete;

        /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;


    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return {
        BT::InputPort<bool>("cancel_control")
        };
    }

private:
  rclcpp::Node::SharedPtr node_;
  bool cancel_control = false;
  bool last_status;
};
}// namespace nav2_custom_bt_nodes

#endif //NAV2_BEHAVIOR_TREE__PLUGINS__CANCEL_CONTROL_CONDITION_HPP_