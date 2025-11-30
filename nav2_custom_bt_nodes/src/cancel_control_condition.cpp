#include "nav2_custom_bt_nodes/cancel_control_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_custom_bt_nodes
{
    CancelControlCondition::CancelControlCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
    : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        last_status = false;
    }

    BT::NodeStatus CancelControlCondition::tick()
    {
        getInput("cancel_control", cancel_control);

        if(last_status != cancel_control)
        {
            last_status = cancel_control;

            if(cancel_control){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cancel_condition_true");
                return BT::NodeStatus::SUCCESS;
            }
        }

        return BT::NodeStatus::FAILURE;
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_bt_nodes::CancelControlCondition>("CancelControlCondition");
}