#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__DETOUR_FASTER_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__DETOUR_FASTER_CONDITION_HPP_

#include <string>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "std_msgs/msg/int32.hpp"

namespace nav2_custom_bt_nodes
{
/**
 * @brief A BT::ConditionNode that returns SUCCESS when the time to take a detour around a person
 *  is faster than waiting for him/her to cross the vehicle path, and FAILURE otherwise
 */
class DetourFasterCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_custom_bt_nodes::DetourFasterCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
    DetourFasterCondition(const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    DetourFasterCondition() = delete;

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
        BT::InputPort<double>("waiting_time", "waiting time for the pedestrian passage"),
        BT::InputPort<double>("average_velocity"),
        BT::InputPort<double>("angular_velocity"),
        BT::InputPort<nav_msgs::msg::Path>("old_path", "Planned path before the pedestrian interruption"),
        BT::InputPort<nav_msgs::msg::Path>("new_path", "Newly planned path after the pedestrian interruption"),
        BT::OutputPort<nav_msgs::msg::Path>("path", "Selected path to be followed")
        };
    }

private:
    bool isDetourFaster(nav_msgs::msg::Path & old_path, nav_msgs::msg::Path & new_path,
                        double waiting_time, double average_velocity, double angular_velocity);
    void closestPointIndexCallback(const std_msgs::msg::Int32 & msg); // to get the index of the closest point to the vehicle pose along the path

    unsigned int closest_point_index_;
    bool msg_received_ = false;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr closest_point_index_sub_;

};
}// namespace nav2_custom_bt_nodes

#endif //NAV2_BEHAVIOR_TREE__PLUGINS__DETOUR_FASTER_CONDITION_HPP_