#include <string>
#include <memory>
#include <vector>

#include "nav2_custom_bt_nodes/stop_wait.hpp"

namespace nav2_custom_bt_nodes
{
StopAndWait::StopAndWait(
    const std::string & name,
    const BT::NodeConfiguration & conf)
    : BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

inline BT::NodeStatus StopAndWait::tick()
{
    getInput("cancel_control", cancel_control);
    getInput("waiting_time", waiting_time);

    // if (status() == BT::NodeStatus::IDLE /*|| status() == BT::NodeStatus::SKIPPED*/) {
    //     first_time_ = true;
    // }

    // setStatus(BT::NodeStatus::RUNNING);

    if(cancel_control /*&& !first_time_*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "cancel control and wait for %d", waiting_time);
        const BT::NodeStatus child_state = child_node_->executeTick();
        switch (child_state) {
            //case BT::NodeStatus::SKIPPED:
            case BT::NodeStatus::RUNNING:
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STOP AND WAIT RUNNING");
                return child_state;
            case BT::NodeStatus::SUCCESS:
                resetChild();
                cancel_control = false;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STOP AND WAIT SUCCEEDED");
                return child_state;
            case BT::NodeStatus::FAILURE:
                resetChild();
                cancel_control = false;
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "STOP AND WAIT FAILED");
                return child_state;
            default:
                return BT::NodeStatus::FAILURE;
        }
    }

    first_time_ = false;
    return BT::NodeStatus::SUCCESS;
}
} // namespace nav2_custom_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    // BT::NodeBuilder builder =
    // [](const std::string & name, const BT::NodeConfiguration & config)
    // {
    //   return std::make_unique<nav2_custom_bt_nodes::StopAndWait>(name, "stop_wait", config);
    // };
  factory.registerNodeType<nav2_custom_bt_nodes::StopAndWait>("StopAndWait");
}