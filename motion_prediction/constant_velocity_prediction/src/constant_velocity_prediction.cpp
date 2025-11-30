#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "people_msgs/msg/people.hpp"
#include "people_msgs/msg/people_prediction.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class CVMPrediction : public rclcpp::Node
{
   public: 

    CVMPrediction() : Node("cvm_prediction")
    {
        people_sub_ = this->create_subscription<people_msgs::msg::People>("/people", 1, std::bind(&CVMPrediction::people_callback, this, _1));
        prediction_pub_ = this->create_publisher<people_msgs::msg::PeoplePrediction>("/people_prediction", 1);
        ego_position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS(), std::bind(&CVMPrediction::ego_position_callback, this, _1));
        ego_path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/ego_predicted_path", 1);
        collisions_pub_ = this->create_publisher<nav_msgs::msg::Path>("/predicted_collisions", 1);
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&CVMPrediction::cmd_vel_callback, this, _1));

        time_resolution = 0.5;
        n_steps = 10; //number  of prediction steps

        collision_radius = 2.88;//person radius + vehicle circular footprint radius
        footprint_offset_x = -1.0;
        footprint_offset_y = 0.0;
        footprint_center_yaw = atan2(footprint_offset_y, footprint_offset_x);
        footprint_center_pos_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/footprint_center",1);
        min_angular_velocity_threshold = 0.01;
    }

   private: 
   
// predict the people trajectories and check for any collsion points
    void people_callback(people_msgs::msg::People::SharedPtr msg)
    {
        people_msgs::msg::People people = *msg;
        people_msgs::msg::Person person;
        people_msgs::msg::People people_one_timestep;
        people_msgs::msg::Person person_one_timestep;
        people_msgs::msg::PeoplePrediction predictions;

        if(!ego_path.poses.empty())
        {
        //create a collision point to each person
        nav_msgs::msg::Path collision_points;
        for(int i=0; i<static_cast<int>(people.people.size()); ++i)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = rclcpp::Time(0);
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            collision_points.poses.push_back(pose);     
        }


        if(people.header.stamp == rclcpp::Time(0))
        people.header.stamp = this->now();

        for(int i=0; i<n_steps; ++i)
        {
            people_one_timestep.people.clear();
            rclcpp::Duration delta_time = rclcpp::Duration::from_seconds(i * time_resolution);

            for(int j=0; j<static_cast<int>(people.people.size()); j++)
            {
                person = people.people.at(j);

                person_one_timestep.position.x = person.position.x + (delta_time.seconds() * person.velocity.x);
                person_one_timestep.position.y = person.position.y + (delta_time.seconds() * person.velocity.y);

                person_one_timestep.velocity = person.velocity;    

                people_one_timestep.header.frame_id = people.header.frame_id;
                people_one_timestep.header.stamp.sec = people.header.stamp.sec + delta_time.seconds();
                people_one_timestep.header.stamp.nanosec = people.header.stamp.nanosec + delta_time.nanoseconds();

                people_one_timestep.people.push_back(person_one_timestep);

                //check for collision at this step
                if(i>0)
                /*no need to check for collision at the person's current location
                since it already has the highest cost */
                {
                double distance = hypot((person_one_timestep.position.x - ego_path.poses[i].pose.position.x),
                                        (person_one_timestep.position.y - ego_path.poses[i].pose.position.y));
                /*The purpose is to find the first collision point in each person path --> avoiding earlier timestep to be overwritten
                  If there is no collision with a person the timestamp of this point remains zero*/
                if(distance <= collision_radius && collision_points.poses[j].header.stamp == rclcpp::Time(0))
                {
                    collision_points.poses[j].header.stamp = people_one_timestep.header.stamp;
                    collision_points.poses[j].pose.position.x = person_one_timestep.position.x;
                    collision_points.poses[j].pose.position.y = person_one_timestep.position.y;
                }
                } 
            }
            predictions.predicted_people.push_back(people_one_timestep);
        }
        
        prediction_pub_->publish(predictions);
        collisions_pub_->publish(collision_points);
        }
    }

// get the control outputs to estimate the vehicle's trajectory
    void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
    {
        linear_velocity = msg.linear.x;
        angular_velocity = msg.angular.z;
    }

    void ego_position_callback(const nav_msgs::msg::Odometry & msg)
    {
        //odom frame is alligned with the map frame in the gazebo environment, so no transformation needed

        //get the center point of the vehicle's footprint in base frame
        double footprint_center_offset = hypot(footprint_offset_x,footprint_offset_y); 

        rclcpp::Time time(msg.header.stamp);
        tf2::Quaternion q(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        tf2::Vector3 position(
            msg.pose.pose.position.x + (footprint_center_offset*cos(yaw + footprint_center_yaw)),
            msg.pose.pose.position.y + (footprint_center_offset*sin(yaw + footprint_center_yaw)),
            0
            );

        tf2::Vector3 velocity(
            linear_velocity*cos(yaw),
            linear_velocity*sin(yaw),
            0
            );

        ego_path.poses.clear();
        geometry_msgs::msg::PoseStamped pose_one_timestep;

        for(int i=0; i<n_steps; ++i)
        {
            rclcpp::Duration delta_time = rclcpp::Duration::from_seconds(i * time_resolution);
            if(abs(angular_velocity) <= min_angular_velocity_threshold)
            // Constant Velocity Model
            {
                pose_one_timestep.pose.position.x = position.getX() + (delta_time.seconds() * velocity.getX());
                pose_one_timestep.pose.position.y = position.getY() + (delta_time.seconds() * velocity.getY());
            }
            else
            // Constant Turn Rate and Velocity Model
            {
                double delta_theta = angular_velocity * delta_time.seconds();
                //in base frame
                double delta_x = (linear_velocity * sin(delta_theta)) / angular_velocity;
                double delta_y = (linear_velocity * (1-cos(delta_theta))) / angular_velocity;
                //in map frame
                double delta_x_map = (delta_x * cos(yaw)) - (delta_y * sin(yaw));
                double delta_y_map = (delta_x * sin(yaw)) + (delta_y * cos(yaw));
                pose_one_timestep.pose.position.x = position.getX() + delta_x_map;
                pose_one_timestep.pose.position.y = position.getY() + delta_y_map;
            }

            pose_one_timestep.header.frame_id = "map";
            pose_one_timestep.header.stamp = time + delta_time;

            ego_path.poses.push_back(pose_one_timestep);
        }
        ego_path.header.frame_id = "map";
        ego_path.header.stamp = this->now();
        ego_path_pub_->publish(ego_path);

        //reset cmd_vel
        linear_velocity = 0.0;
        angular_velocity = 0.0;

        //publish footprint pose for conflict detection
        geometry_msgs::msg::Point footprint_pos;
        footprint_pos.x = position.getX();
        footprint_pos.y = position.getY();
        footprint_center_pos_pub_->publish(footprint_pos);
    }

    rclcpp::Subscription<people_msgs::msg::People>::SharedPtr people_sub_;
    rclcpp::Publisher<people_msgs::msg::PeoplePrediction>::SharedPtr prediction_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_position_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ego_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr collisions_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr footprint_center_pos_pub_;

    int n_steps;
    double time_resolution;
    nav_msgs::msg::Path ego_path;
    double collision_radius;
    double footprint_offset_x, footprint_offset_y, footprint_center_yaw; //distance between the footprint center and the base_link
    double linear_velocity, angular_velocity;
    double min_angular_velocity_threshold;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CVMPrediction>());
  rclcpp::shutdown();
  return 0;
}
