#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__IS_PATH_UPDATED_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__IS_PATH_UPDATED_CONDITION_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point.h"

namespace nav2_custom_bt_nodes
{
/**
 * @brief A BT::ConditionNode that counts the number of path updates
 */
class IsPathUpdated : public BT::ConditionNode
{
public:

    IsPathUpdated(const std::string & condition_name,
        const BT::NodeConfiguration & conf);
    
    IsPathUpdated() = delete;

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
            //BT::InputPort<nav_msgs::msg::Path>("path", "Path sent to the controller"),
            // BT::InputPort<int>("current_update_count", 0, "Planner calls counter"),
            // BT::OutputPort<int>("new_update_count"),
        };
    }

private:

    //nav_msgs::msg::Path old_path_;
    // int counter_ = 0;
    rclcpp::Node::SharedPtr node_;
    std_msgs::msg::Int32 updates_count_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr updates_count_pub_;

};
}// namespace nav2_custom_bt_nodes

#endif //NAV2_BEHAVIOR_TREE__PLUGINS__IS_PATH_UPDATED_CONDITION_HPP_