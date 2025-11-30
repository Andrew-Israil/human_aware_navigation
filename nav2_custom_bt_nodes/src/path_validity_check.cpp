#include "nav2_custom_bt_nodes/path_validity_check.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_custom_bt_nodes
{

PathValidityCheck::PathValidityCheck(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  initialized_(false)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  client_ = node_->create_client<nav2_msgs::srv::IsPathValid>("path_validity_check");
  conflict_client_ = node_->create_client<conflict_type_msgs::srv::GetConflictType>("conflict_type");
  speed_limit_pub_ = node_->create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", 1);

  server_timeout_ = config().blackboard->template get<std::chrono::milliseconds>("server_timeout");
}

void PathValidityCheck::initialize()
{
  getInput<std::chrono::milliseconds>("server_timeout", server_timeout_);
  initialized_ = true;
}

BT::NodeStatus PathValidityCheck::tick()
{
  if (!initialized_) {
    initialize();
  }

  nav_msgs::msg::Path path;
  getInput("path", path);

  auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();

  request->path = path;
  auto response = client_->async_send_request(request);

  cancel_control = false;
  waiting_time = 0;
  BT::NodeStatus status = BT::NodeStatus::FAILURE;

  if (rclcpp::spin_until_future_complete(node_, response, server_timeout_) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = response.get();
    if (result->is_valid) {
      //reset speed limit to maximum in case of valid path
      //otherwise it will be set by conflict_type_server
      nav2_msgs::msg::SpeedLimit speed;
      speed.percentage = false;
      speed.speed_limit = 2;
      speed_limit_pub_->publish(speed);
      cancel_control = false;
      status = BT::NodeStatus::SUCCESS;

    } else {
      nav_msgs::msg::Path invalid_path;
      for(int i=0; i<result->invalid_pose_indices.size();i++)
      {
       invalid_path.poses.push_back(path.poses[result->invalid_pose_indices[i]]);
       //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "invalid_index:%ld", result.get()->invalid_pose_indices[i]);
      }

      auto get_conflict_request = std::make_shared<conflict_type_msgs::srv::GetConflictType::Request>();
      get_conflict_request->invalid_path = invalid_path;
      auto get_conflict_response = conflict_client_->async_send_request(get_conflict_request);

      if (rclcpp::spin_until_future_complete(node_, get_conflict_response, server_timeout_) ==
            rclcpp::FutureReturnCode::SUCCESS)
      {
        auto get_conflict_result = get_conflict_response.get();

        cancel_control = get_conflict_result->cancel_control;
        waiting_time = get_conflict_result->waiting_time;
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "waiting for:%lf", waiting_time);

        if(! get_conflict_result->replan)
         status = BT::NodeStatus::SUCCESS; 
      }
    }
  }
  
  setOutput("cancel_control", cancel_control);
  setOutput("waiting_time", waiting_time);
  return status;
}

}  // namespace nav2_custom_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_bt_nodes::PathValidityCheck>("PathValidityCheck");
}