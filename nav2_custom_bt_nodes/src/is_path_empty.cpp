#include "nav2_custom_bt_nodes/is_path_empty.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_custom_bt_nodes
{
    IsPathEmpty::IsPathEmpty(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
    : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    }

    BT::NodeStatus IsPathEmpty::tick()
    {
        getInput("path", path);

        if(path.poses.empty())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Empty path");
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_bt_nodes::IsPathEmpty>("IsPathEmpty");
}