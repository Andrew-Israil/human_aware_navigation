#include "nav2_custom_bt_nodes/detour_faster_condition.hpp"
#include <chrono>
#include <memory>
#include <string>


namespace nav2_custom_bt_nodes
{
    DetourFasterCondition::DetourFasterCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
    : BT::ConditionNode(condition_name, conf)
    {
        node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
        closest_point_index_sub_ = node_->create_subscription<std_msgs::msg::Int32>(  
            "closest_point_index",  1,  
            std::bind(&DetourFasterCondition::closestPointIndexCallback, this, std::placeholders::_1)  
        );
    }

   //get the closest point on the actual path to the vehicle's current position
    void DetourFasterCondition::closestPointIndexCallback(const std_msgs::msg::Int32 & msg)
    {
        msg_received_ = true;
        closest_point_index_ = msg.data;
    }

    bool DetourFasterCondition::isDetourFaster(nav_msgs::msg::Path & old_path,
                                               nav_msgs::msg::Path & new_path,
                                               double waiting_time,
                                               double average_velocity,
                                               double angular_velocity)
    {
        double old_path_angle = atan2(old_path.poses[closest_point_index_+1].pose.position.y -
                                      old_path.poses[closest_point_index_].pose.position.y,
                                      old_path.poses[closest_point_index_+1].pose.position.x -
                                      old_path.poses[closest_point_index_].pose.position.x);
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "old_path_angle = %f",old_path_angle);

        double new_path_angle = atan2(new_path.poses[1].pose.position.y -
                                      new_path.poses[0].pose.position.y,
                                      new_path.poses[1].pose.position.x -
                                      new_path.poses[0].pose.position.x);
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "new_path_angle = %f",new_path_angle);

        //calculate the heading change from old to new path
        double delta_angle = abs(new_path_angle - old_path_angle);
            if(delta_angle > M_PI){
                delta_angle = (2*M_PI) - delta_angle;
            }

        double old_path_travel_time = waiting_time +
        nav2_util::geometry_utils::calculate_path_length(old_path, closest_point_index_) / average_velocity;

        double new_path_travel_time = (delta_angle / angular_velocity) + 
        nav2_util::geometry_utils::calculate_path_length(new_path, 0) / average_velocity;

        return new_path_travel_time < old_path_travel_time;
    }

    BT::NodeStatus DetourFasterCondition::tick()
    {
        double waiting_time, average_velocity, angular_velocity;
        nav_msgs::msg::Path old_path, new_path;

        getInput("waiting_time", waiting_time);
        getInput("average_velocity", average_velocity);
        getInput("angular_velocity", angular_velocity);
        getInput("old_path", old_path);
        getInput("new_path", new_path);

        if(new_path.poses.empty())
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "No detour found");
            return BT::NodeStatus::FAILURE;  
        }

         while (!msg_received_)  
        {  
            rclcpp::spin_some(node_);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "waiting for closest point index");
        } 

        if(isDetourFaster(old_path, new_path, waiting_time, average_velocity, angular_velocity))
        {
            setOutput("path", new_path);
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Detour is faster");
            return BT::NodeStatus::SUCCESS;
        }

        setOutput("path", old_path);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Detour is slower");
        return BT::NodeStatus::FAILURE;
    }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_custom_bt_nodes::DetourFasterCondition>("DetourFasterCondition");
}