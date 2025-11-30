#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__IS_PATH_EMPTY_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__IS_PATH_EMPTY_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"

namespace nav2_custom_bt_nodes
{
class IsPathEmpty : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_custom_bt_nodes::IsPathEmpty
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
    IsPathEmpty(const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    IsPathEmpty() = delete;

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
        BT::InputPort<nav_msgs::msg::Path>("path"),
        };
    }

private:
    rclcpp::Node::SharedPtr node_;
    nav_msgs::msg::Path path;
};
} // namespace nav2_custom_bt_nodes

#endif //NAV2_BEHAVIOR_TREE__PLUGINS__IS_PATH_EMPTY_HPP_