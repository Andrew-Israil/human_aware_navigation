#include "rclcpp/rclcpp.hpp"
#include "conflict_type_msgs/srv/get_conflict_type.hpp"
#include "people_msgs/msg/people_prediction.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <memory>
#include <mutex>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ConflictTypeServer : public rclcpp::Node
{
    public:

    ConflictTypeServer() : Node("conflict_type_server")
    {
        service = this->create_service<conflict_type_msgs::srv::GetConflictType>("conflict_type", 
                        std::bind(&ConflictTypeServer::get_conflict_type, this, _1, _2));
        prediction_sub_ = this->create_subscription<people_msgs::msg::PeoplePrediction>("/people_prediction", rclcpp::SystemDefaultsQoS(), 
                      std::bind(&ConflictTypeServer::prediction_callback, this, _1));
        footprint_center_pos_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/footprint_center", rclcpp::SystemDefaultsQoS(),
             std::bind(&ConflictTypeServer::footprint_center_callback, this, _1)
        );
        speed_limit_pub_ = this->create_publisher<nav2_msgs::msg::SpeedLimit>("/speed_limit", 1);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",1);
        collision_radius = 2.88;
        half_vehicle_length = 1.6;
        v_max = 2;
        a_max = 0.5;
        r = 0; //fraction of v_max, determines the vehicle's velocity before reaching d_min (in crossing_paths) 
        min_delta_angle = 0.175; // 10 degrees
        max_delta_angle = 2.967; // 170 degrees
    }

    private:

    void footprint_center_callback(const geometry_msgs::msg::Point &msg)
    {
        ego_position_x = msg.x;
        ego_position_y = msg.y;
    }

    void prediction_callback(people_msgs::msg::PeoplePrediction::SharedPtr msg)
    {
        predictions = *msg;
    }

    void get_conflict_type(const std::shared_ptr<conflict_type_msgs::srv::GetConflictType::Request>request,
                        std::shared_ptr<conflict_type_msgs::srv::GetConflictType::Response>response)
    {
        double path_angle;
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "conflict service called");
        if(request->invalid_path.poses.size()>1){

            //get the vehicle heading at the first invalid pose
            int i = 1;
            double delta_x = 0;
            double delta_y = 0;
            while(i < request->invalid_path.poses.size())
            {
                delta_x = request->invalid_path.poses[i].pose.position.x -
                            request->invalid_path.poses[0].pose.position.x;
                delta_y = request->invalid_path.poses[i].pose.position.y -
                            request->invalid_path.poses[0].pose.position.y;

                if(hypot(delta_x, delta_y) >= 1)
                    break;
                i++;
            }
            path_angle = atan2(delta_y, delta_x);

        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Emty path");
            response->replan = false;
            response->cancel_control = false;
            return;
        }

        //detrmine the person with who the conflict is occuring and get his path angle
        std::lock_guard<std::mutex> guard(predictions_mutex);
        people_msgs::msg::People people_k;
        people_msgs::msg::Person person_k;
        double person_path_angle;
        double person_distance_to_conflict;
        double person_velocity;
        bool person_found = false;
        for(int i=0; i<(static_cast<int>(predictions.predicted_people.size())) && !person_found; ++i)
        {
            people_k = predictions.predicted_people[i];
            for(int j=0; j<static_cast<int>(people_k.people.size()); j++)
            {
                person_k = people_k.people[j];
                if(hypot((person_k.position.x - request->invalid_path.poses[0].pose.position.x),
                  (person_k.position.y - request->invalid_path.poses[0].pose.position.y)) <= collision_radius)
                {
                    //means the conflict is with this person
                    person_path_angle = atan2(person_k.velocity.y, person_k.velocity.x);
                    person_velocity = hypot(person_k.velocity.y, person_k.velocity.x);
                    person_distance_to_conflict = hypot((predictions.predicted_people[0].people[j].position.x -
                                                        request->invalid_path.poses[0].pose.position.x),
                                                        (predictions.predicted_people[0].people[j].position.y -
                                                        request->invalid_path.poses[0].pose.position.y));
                    person_found = true;
                    break;
                }
            }
        }

        nav2_msgs::msg::SpeedLimit speed;
        speed.percentage = false;
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;

        if(!person_found)
        {
            response->replan = true;
            response->cancel_control = false;
            speed.speed_limit = v_max;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "no_conflict_with_human");

        } else {

            double distance_to_conflict = hypot((ego_position_x-request->invalid_path.poses[0].pose.position.x),
                        (ego_position_y-request->invalid_path.poses[0].pose.position.y)) - half_vehicle_length;
            double d_min = pow(v_max,2)/(2*a_max); //minimum distance to stop when at v_max
            double d_prox = d_min + pow((v_max*(1-r)),2)/(2*a_max); // minimum distance to start decceleration before reaching d_min
        
           
            if(person_velocity < 0.09){

                if(distance_to_conflict <= d_prox)
                {
                    response->waiting_time = 0;
                    response->replan = true;
                    response->cancel_control = false;
                    speed.speed_limit = 0.1;//v_max;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "standing_human->replan");
                } else {
                    response->replan = false;
                    response->cancel_control = false;
                    speed.speed_limit = v_max;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "standing_human->far_away");
                }

            } else {

                double delta_angle = abs(path_angle - person_path_angle);
                if(delta_angle > M_PI){
                    delta_angle = (2*M_PI) - delta_angle;
                }

                if(delta_angle > min_delta_angle && delta_angle < max_delta_angle){ //greater than 10 degrees and less than 170 degrees

                    response->replan = false; //detour is calculated by the BT call after slowing and stopping
                    
                    if(distance_to_conflict <= d_min)
                    {
                        speed.speed_limit = 0.1;
                        response->cancel_control = true;
                        response->waiting_time = (person_distance_to_conflict) / person_velocity;
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "crossing_path->cancel_control");
                    } else if(distance_to_conflict <= d_prox) {
                        speed.speed_limit = v_max*(distance_to_conflict/d_prox);
                        response->cancel_control = false;
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "crossing_path->slowing_down");
                    } 

                } else if(delta_angle <= min_delta_angle && distance_to_conflict > d_min){    
                    response->replan = false;
                    response->cancel_control = false;
                    speed.speed_limit = v_max;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "aligned_path->same_direction->can_follow");
                } else {
                    response->replan = true;
                    response->cancel_control = false;
                    speed.speed_limit = 0.0;
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "aligned_path->replan");
                }
            }
        }
        speed_limit_pub_->publish(speed);
    }

    rclcpp::Service<conflict_type_msgs::srv::GetConflictType>::SharedPtr service;
    rclcpp::Subscription<people_msgs::msg::PeoplePrediction>::SharedPtr prediction_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr footprint_center_pos_sub_;
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_pub_;

    double collision_radius;
    people_msgs::msg::PeoplePrediction predictions;
    double ego_position_x, ego_position_y; 
    std::mutex predictions_mutex;
    double half_vehicle_length;
    double v_max, a_max, r;
    double min_delta_angle, max_delta_angle;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConflictTypeServer>());
  rclcpp::shutdown();
  return 0;
}