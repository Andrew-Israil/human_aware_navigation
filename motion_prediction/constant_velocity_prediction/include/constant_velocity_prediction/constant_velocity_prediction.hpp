#ifndef CONSTANT_VELOCITY_PREDICTION_H_
#define CONSTANT_VELOCITY_PREDICTION_H_
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <people_msgs/msg/people.hpp>  
#include <people_msgs/PeoplePrediction.h>
#include <visualization_msgs/MarkerArray.h>

namespace motion_prediction{

class CVMPrediction : public rclcpp::Node
{

public:

 CVMPrediction();
 void people_callback(people_msgs::People msg);

private:
  rclcpp::Subscription<people_msgs::People>::SharedPtr people_sub_;
  rclcpp::Publisher<people_msgs::PeoplePrediction>::SharedPtr prediction_pub_;

  int n_steps;
  double time_resolution;
};

}

#endif