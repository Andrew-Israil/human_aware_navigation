// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav_msgs/msg/path.hpp"
// #include "nav2_costmap_2d/costmap_2d_ros.hpp"
// #include "nav2_costmap_2d/cost_values.hpp"
// #include "nav2_msgs/srv/is_path_valid.hpp"
// #include "nav2_costmap_2d/footprint_collision_checker.hpp"
// #include "nav2_util/geometry_utils.hpp"

// #include <mutex>
// #include <memory>
// #include <chrono>
// #include <string>
// #include <utility>
#include "path_validity_check_server/path_validity_check_server.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace path_validity_check_server
{
    PathValidityCheck::PathValidityCheck(const rclcpp::NodeOptions & options)
    : nav2_util::LifecycleNode("path_validity_check_server", "", options)
    {
        use_radius = false;
    }

    PathValidityCheck::~PathValidityCheck(){}

    nav2_util::CallbackReturn
    PathValidityCheck::on_configure(const rclcpp_lifecycle::State & /*state*/)
    {
        auto node = shared_from_this();
        service = node->create_service<nav2_msgs::srv::IsPathValid>("path_validity_check", 
                        std::bind(&PathValidityCheck::isPathValid, this, _1, _2));

        robot_position_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", rclcpp::SystemDefaultsQoS(), std::bind(&PathValidityCheck::robot_position_callback, this, _1));
        
        tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            get_node_base_interface(),
            get_node_timers_interface());
        tf_->setCreateTimerInterface(timer_interface);
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

        global_costmap_sub_ = std::make_unique<nav2_costmap_2d::CostmapSubscriber>(
        node, std::string("global_costmap/costmap_raw"));

        global_footprint_sub_ = std::make_unique<nav2_costmap_2d::FootprintSubscriber>(
        node, std::string("global_costmap/published_footprint"), *tf_, std::string("base_link"), 0.1);

        closest_point_index_publisher_ = node->create_publisher<std_msgs::msg::Int32>("closest_point_index",1);
      
        RCLCPP_INFO(get_logger(), "path_validity_check_server configured");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    void PathValidityCheck::robot_position_callback(const nav_msgs::msg::Odometry & msg)
    {
        robot_pose.pose.position.x = msg.pose.pose.position.x;
        robot_pose.pose.position.y = msg.pose.pose.position.y;
    }

    void PathValidityCheck::isPathValid(const std::shared_ptr<nav2_msgs::srv::IsPathValid::Request> request,
    std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
    {
        //RCLCPP_INFO(get_logger(), "path_validity_check_server called");
        response->is_valid = true;

        if (request->path.poses.empty()) {
            response->is_valid = false;
            return;
        }

        costmap_ = global_costmap_sub_->getCostmap();
        if(costmap_ == nullptr)
         RCLCPP_INFO(get_logger(), "costmap_ = nullptr");

        if (!use_radius) {
            collision_checker_ =
            std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<std::shared_ptr<nav2_costmap_2d::Costmap2D>>>(
            costmap_);
        }

        unsigned int closest_point_index = 0;
        float current_distance = std::numeric_limits<float>::max();
        float closest_distance = current_distance;
        geometry_msgs::msg::Point current_point = robot_pose.pose.position;
        for (unsigned int i = 0; i < request->path.poses.size(); ++i) {
            geometry_msgs::msg::Point path_point = request->path.poses[i].pose.position;

            current_distance = nav2_util::geometry_utils::euclidean_distance(
                current_point,
                path_point);

            if (current_distance < closest_distance) {
                closest_point_index = i;
                closest_distance = current_distance;
            }
        }

        std_msgs::msg::Int32 index_msg;
        index_msg.data = closest_point_index;
        closest_point_index_publisher_->publish(index_msg);

        /**
         * The lethal check starts at the closest point to avoid points that have already been passed
         * and may have become occupied. The method for collision detection is based on the shape of
         * the footprint.
         */
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
        unsigned int mx = 0;
        unsigned int my = 0;
        //RCLCPP_INFO(get_logger(), "got mutex successfully");
        unsigned int first_index = 0;
        unsigned int cost = nav2_costmap_2d::FREE_SPACE;
        for (unsigned int i = closest_point_index; i < request->path.poses.size(); ++i) {
            auto & position = request->path.poses[i].pose.position;
            if (use_radius) {
                if (costmap_->worldToMap(position.x, position.y, mx, my)) {
                cost = costmap_->getCost(mx, my);
                } else {
                cost = nav2_costmap_2d::LETHAL_OBSTACLE;
                }
            } else {
                //nav2_costmap_2d::Footprint footprint;
                std::vector<geometry_msgs::msg::Point> footprint;
                std_msgs::msg::Header footprint_header;
                if(global_footprint_sub_->getFootprintInRobotFrame(footprint,footprint_header))
                {
                    auto theta = tf2::getYaw(request->path.poses[i].pose.orientation);
                    cost = static_cast<unsigned int>(collision_checker_->footprintCostAtPose(
                    position.x, position.y, theta, footprint));
                } else {
                    RCLCPP_INFO(get_logger(), "failed to get cost at footprint");
                }
                
            }

            if (/*use_radius &&*/
                (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
                cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
            {
                response->is_valid = false;
                response->invalid_pose_indices.push_back(i);
                if(first_index == 0)
                first_index = i;
            } /*else if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
                response->is_valid = false;
                response->invalid_pose_indices.push_back(i);
                if(first_index == 0)
                first_index = i;
            } */
        }
        if(!response->is_valid){
        RCLCPP_INFO(get_logger(), "path validty check service, invalid path");
        RCLCPP_INFO(get_logger(), "first_index:%u",first_index);
        RCLCPP_INFO(get_logger(), "invalid_index:%d",response->invalid_pose_indices[0]);
        }
    }

    nav2_util::CallbackReturn
    PathValidityCheck::on_activate(const rclcpp_lifecycle::State & /*state*/)
    {
        createBond();
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    PathValidityCheck::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
    {
        destroyBond();
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    PathValidityCheck::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
    {
        transform_listener_.reset();
        tf_.reset();
        global_costmap_sub_.reset();
        global_footprint_sub_.reset();
        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn
    PathValidityCheck::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
    {
        return nav2_util::CallbackReturn::SUCCESS;
    }
} //end path_validity_check_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(path_validity_check_server::PathValidityCheck)