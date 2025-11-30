#ifndef GAZEBO_PLUGINS_ACTORPLUGIN_HH_
#define GAZEBO_PLUGINS_ACTORPLUGIN_HH_

#include <string>
#include <vector>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "rclcpp/rclcpp.hpp"
#include "people_msgs/msg/person.hpp"
#include "people_msgs/msg/people.hpp"


namespace gazebo
{
  class GZ_PLUGIN_VISIBLE ActorPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: ActorPlugin();

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

    /// \brief add gaussian noise to person ground thruth.
    /// \param[in] point_ value to which noise is added.
    private: double add_gaussian_noise(double point, double variance, double mean);

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

    //to publish actor position
    std::shared_ptr<rclcpp::Node> node;  
    rclcpp::Publisher<people_msgs::msg::Person>::SharedPtr actor_pos_pub;
    rclcpp::Publisher<people_msgs::msg::People>::SharedPtr people_pos_pub;
    std::string node_name;
    std::string pub_topic_name;

    double x_map_offset, y_map_offset;
    std::vector<ignition::math::Vector3d> waypoints;
    double delay_to_target;
  };
}
#endif