#include "nav2_custom_bt_nodes/is_path_updated.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace nav2_custom_bt_nodes
{
    IsPathUpdated::IsPathUpdated(const std::string & condition_name,
        const BT::NodeConfiguration & conf)
        : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        updates_count_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/path_updates_count",1);
        //updates_count_.data = 0;
    }
    
    BT::NodeStatus IsPathUpdated::tick()
    {
        // nav_msgs::msg::Path new_path;
        // getInput("path", new_path);

        // if(new_path.poses.empty())
        // return BT::NodeStatus::SUCCESS;

        // if(old_path_.poses.empty())
        // {
        //     old_path_ = new_path;
        // }
        // else 
        // {
        //     geometry_msgs::msg::PoseStamped current_pose = new_path.poses[0];
        //     unsigned int closest_point_index = 0;
        //     float current_distance = std::numeric_limits<float>::max();
        //     float closest_distance = current_distance;
        //     geometry_msgs::msg::Point current_point = current_pose.pose.position;
        //     for (unsigned int i = 0; i < old_path_.poses.size(); ++i) {
        //     geometry_msgs::msg::Point path_point = old_path_.poses[i].pose.position;

        //     current_distance = nav2_util::geometry_utils::euclidean_distance(
        //         current_point,
        //         path_point);

        //     if (current_distance < closest_distance) {
        //         closest_point_index = i;
        //         closest_distance = current_distance;
        //     }
        //     }

        //     double old_path_length =  nav2_util::geometry_utils::calculate_path_length(old_path_, closest_point_index);
        //     double new_path_length = nav2_util::geometry_utils::calculate_path_length(new_path, 0);
        //     if(abs(new_path_length - old_path_length) / old_path_length >= 0.01)
        //         RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "new actual path");
        //     old_path_ = new_path;
        // }
        // else if(new_path != old_path_ && old_path_.poses.size() != 0 &&
        //  new_path.poses.size() != 0 &&
        //  old_path_.poses.back().pose == new_path.poses.back().pose) 
        // {
        //     updates_count_.data++;
        //     updates_count_pub_->publish(updates_count_);
        // }
        updates_count_.data = 1;
        updates_count_pub_->publish(updates_count_);
        return BT::NodeStatus::SUCCESS;
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_bt_nodes::IsPathUpdated>("IsPathUpdated");
}