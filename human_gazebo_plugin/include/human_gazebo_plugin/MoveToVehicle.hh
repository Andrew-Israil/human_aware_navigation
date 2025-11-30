#ifndef GAZEBO_PLUGINS_MOVETOVEHICLE_HH_
#define GAZEBO_PLUGINS_MOVETOVEHICLE_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "rclcpp/rclcpp.hpp"
#include "people_msgs/msg/person.hpp"
#include "people_msgs/msg/people.hpp"
#include "nav_msgs/msg/odometry.hpp"


namespace gazebo
{
  class GZ_PLUGIN_VISIBLE MoveToVehicle : public ModelPlugin
  {
    /// \brief Constructor
    public: MoveToVehicle();

    /// \brief Load the actor plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: virtual void Reset();

    /// \brief Function that is called every update cycle.
    /// \param[in] _info Timing information
    private: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Helper function to choose a new target location
    private: void ChooseNewTarget();

    /// \brief Helper function to avoid obstacles. This implements a very
    /// simple vector-field algorithm.
    /// \param[in] _pos Direction vector that should be adjusted according
    /// to nearby obstacles.
    private: void HandleObstacles(ignition::math::Vector3d &_pos);

    /// \brief Pointer to the parent actor.
    private: physics::ActorPtr actor;

    /// \brief Pointer to the world, for convenience.
    private: physics::WorldPtr world;

    /// \brief Pointer to the sdf element.
    private: sdf::ElementPtr sdf;

    /// \brief Velocity of the actor
    private: ignition::math::Vector3d velocity;

    /// \brief List of connections
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief Current target location
    private: ignition::math::Vector3d target;

    /// \brief Target location weight (used for vector field)
    private: double targetWeight = 1.0;

    /// \brief Obstacle weight (used for vector field)
    private: double obstacleWeight = 1.0;

    /// \brief Time scaling factor. Used to coordinate translational motion
    /// with the actor's walking animation.
    private: double animationFactor = 1.0;

    /// \brief Time of the last update.
    private: common::Time lastUpdate;

    /// \brief List of models to ignore. Used for vector field
    private: std::vector<std::string> ignoreModels;

    /// \brief Custom trajectory info.
    private: physics::TrajectoryInfoPtr trajectoryInfo;

    private: void odom_callback(const nav_msgs::msg::Odometry & msg);

    //to publish actor position
    std::shared_ptr<rclcpp::Node> node;  
    rclcpp::Publisher<people_msgs::msg::Person>::SharedPtr actor_pos_pub;
    rclcpp::Publisher<people_msgs::msg::People>::SharedPtr people_pos_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::string node_name;
    std::string pub_topic_name;

    double x_map_offset, y_map_offset;
    std::vector<ignition::math::Vector3d> waypoints;
    ignition::math::Vector3d collision_point;
    ignition::math::Vector3d vehicle_pos_;
    double vehicle_speed_ = 0;
  };
}
#endif