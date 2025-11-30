#ifndef GAZEBO_PLUGINS_WORKERPLUGIN_HH_
#define GAZEBO_PLUGINS_WORKERPLUGIN_HH_

#include <functional>
#include <string>
#include <vector>

#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/util/system.hh"
#include "ignition/math/Vector3.hh"
#include "rclcpp/rclcpp.hpp"
#include "people_msgs/msg/person.hpp"

namespace gazebo
{
    class WorkerPlugin : public ModelPlugin
    {
        public: WorkerPlugin();
        /// \brief Load the actor plugin.
        /// \param[in] _model Pointer to the parent model.
        /// \param[in] _sdf Pointer to the plugin's SDF elements.
        public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        /// \brief Function that is called every update cycle.
        /// \param[in] _info Timing information
        //private: void OnUpdate(const common::UpdateInfo &_info);
        public: void OnUpdate();

         /// \brief Helper function to choose a new target location
        private: void ChooseNewTarget();

        // Pointer to the model
        private: physics::ModelPtr model;

        // // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        /// \brief List of connections
        //private: std::vector<event::ConnectionPtr> connections;

        /// \brief Pointer to the sdf element.
        private: sdf::ElementPtr sdf;

        /// \brief Velocity of the actor
        private: ignition::math::Vector3d velocity;

        /// \brief Current target location
        private: ignition::math::Vector3d target;

        /// \brief Time of the last update.
        private: common::Time lastUpdate;

        std::vector<ignition::math::Vector3d> waypoints;
        double x_map_offset, y_map_offset;
        double delay_to_target;

        //to publish actor position
        std::shared_ptr<rclcpp::Node> node;  
        rclcpp::Publisher<people_msgs::msg::Person>::SharedPtr person_pos_pub;
        std::string node_name;
        std::string pub_topic_name;


    };
}

#endif