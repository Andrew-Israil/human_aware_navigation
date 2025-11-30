#ifndef PATH_VALIDITY_CHECK_SERVER_HPP_
#define PATH_VALIDITY_CHECK_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_subscriber.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "std_msgs/msg/int32.hpp"

namespace path_validity_check_server
{
    class PathValidityCheck : public nav2_util::LifecycleNode
    {
        public:

        PathValidityCheck(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        ~PathValidityCheck();

        protected:
        /**
         * @brief Configure member variables and initializes planner
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
        /**
         * @brief Activate member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
        /**
         * @brief Deactivate member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
        /**
         * @brief Reset member variables
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
        /**
         * @brief Called when in shutdown state
         * @param state Reference to LifeCycle node state
         * @return SUCCESS or FAILURE
         */
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

        /**
     * @brief The service callback to determine if the path is still valid and returns the invalid poses indicies
     * @param request to the service
     * @param response from the service
     */
        void isPathValid(
            const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
            std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response);

        void robot_position_callback(const nav_msgs::msg::Odometry & msg);

        rclcpp::Service<nav2_msgs::srv::IsPathValid>::SharedPtr service;
        std::unique_ptr<nav2_costmap_2d::CostmapSubscriber> global_costmap_sub_;
        std::unique_ptr<nav2_costmap_2d::FootprintSubscriber> global_footprint_sub_;
        std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_{nullptr};
        std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>>
        collision_checker_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
        bool use_radius;
        geometry_msgs::msg::PoseStamped robot_pose;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_position_sub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr closest_point_index_publisher_;
    };
}
#endif 