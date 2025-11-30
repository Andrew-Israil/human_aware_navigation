#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "people_msgs/msg/people.hpp"

#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PeoplePublisher : public rclcpp::Node
{
    public:

    PeoplePublisher() : Node("people_publisher")
    {
        person_sub_ = this->create_subscription<people_msgs::msg::Person>("/person_pos", 10, std::bind(&PeoplePublisher::person_callback, this, _1));
        //actor2_sub_ = this->create_subscription<people_msgs::msg::Person>("actor2_pos", 10, std::bind(&PeoplePublisher::person_callback, this, _1));
        people_pub_ = this->create_publisher<people_msgs::msg::People>("/people", 1);
        timer_ = this->create_wall_timer(500ms, std::bind(&PeoplePublisher::timer_callback, this));

        ego_position_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::SystemDefaultsQoS(), std::bind(&PeoplePublisher::ego_position_callback, this, std::placeholders::_1));
    }

    private:

    void person_callback(people_msgs::msg::Person::SharedPtr msg)
    {

        bool found = false;
        if(people.people.empty())
        {
           people.people.push_back(*msg);
        }
        else
        {
            for(int i=0; i<static_cast<int>(people.people.size()); i++)
            {
                if(people.people.at(i).name == msg->name)
                {
                    people.people.at(i).position.x = msg->position.x;
                    people.people.at(i).position.y = msg->position.y;
                    people.people.at(i).velocity.x = msg->velocity.x;
                    people.people.at(i).velocity.y = msg->velocity.y;
                    found = true;
                    break;
                }
            }
            if(!found)
            people.people.push_back(*msg);
        }
        // people.header.frame_id = "map";
        // people.header.stamp = this->now();
        // people_pub_->publish(people);
    }


// used only to filter out people at a distance greater than 10m in simulation
    // void person_callback(people_msgs::msg::Person::SharedPtr msg)
    // {
    //     //only for realistic detection in simulation
    //     double distance = hypot((position_x - msg->position.x), (position_y - msg->position.y)); 
    //     // if(distance > 10){ return;}

    //     bool found = false;
    //     if(people.people.empty() && distance <= 10)
    //     {
    //        people.people.push_back(*msg);
    //     }
    //     else
    //     {
    //         for(int i=0; i<static_cast<int>(people.people.size()); i++)
    //         {
    //             if(people.people.at(i).name == msg->name)
    //             {
    //                 if(distance <= 10)
    //                 {
    //                     people.people.at(i).position.x = msg->position.x;
    //                     people.people.at(i).position.y = msg->position.y;
    //                     people.people.at(i).velocity.x = msg->velocity.x;
    //                     people.people.at(i).velocity.y = msg->velocity.y;
    //                 } else {
    //                     people.people.erase(std::next(people.people.begin(), i));
    //                 }
    //                 found = true;
    //                 break;
    //             }
    //         }
    //         if(!found && distance <= 10)
    //         people.people.push_back(*msg);
    //     }
    //     // people.header.frame_id = "map";
    //     // people.header.stamp = this->now();
    //     // people_pub_->publish(people);
    // }

//people are published at a frequencecy of 2Hz
    void timer_callback()
    {
        // if(!people.people.empty() && people.people.at(0).position.x > 17 && position_y > 20/*position_x > 15*/)
        //if(position_x > 12 && position_x < 19)
        //if(position_y > 20) // condition used only for limiting people detectability
        //{
        people.header.frame_id = "map";
        people.header.stamp = this->now();
        people_pub_->publish(people);
        //}
    }

    void ego_position_callback(const nav_msgs::msg::Odometry & msg)
    {
        position_x = msg.pose.pose.position.x;
        position_y = msg.pose.pose.position.y;
    }

    rclcpp::Subscription<people_msgs::msg::Person>::SharedPtr person_sub_;
    //rclcpp::Subscription<people_msgs::msg::Person>::SharedPtr actor2_sub_;
    rclcpp::Publisher<people_msgs::msg::People>::SharedPtr people_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    people_msgs::msg::People people;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_position_sub_;
    double position_x, position_y;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PeoplePublisher>());
  rclcpp::shutdown();
  return 0;
}