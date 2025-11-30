#ifndef HUMAN_DYNAMIC_LAYER_HPP_
#define HUMAN_DYNAMIC_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "std_msgs/msg/string.hpp"
#include "people_msgs/msg/people.hpp"
#include "people_msgs/msg/people_prediction.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_costmap_2d/footprint.hpp"

#define COVARIANCE_SIDE 0.25
//#define COVARIANCE_BACK 0.5

namespace nav2_human_dynamic_layer_costmap_plugin
{

class HumanDynamicLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  HumanDynamicLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();
  virtual bool isClearable() {return false;}

private:
    void prediction_callback(people_msgs::msg::PeoplePrediction::SharedPtr msg);
    void collisions_callback(nav_msgs::msg::Path::SharedPtr msg);
    double calc_cutoff_radius(double cutoff_value, double amplitude, double variance);
    double calc_gaussian(double pos_x, double pos_y, double origin_x,
                        double origin_y, double amplitude, double variance_side,
                        double variance_front, double variance_back, double skew);
    void define_human_space(double person_position_x, double person_position_y, double person_angle,
                            double person_velocity, unsigned char * map_layer, nav2_costmap_2d::Costmap2D & master_grid, double cost_factor, int index,
                            unsigned char * map_copy);

    rclcpp::Subscription<people_msgs::msg::PeoplePrediction>::SharedPtr prediction_sub_; //subscribe to people prdicted steps topic
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr collisions_sub_;              //subscribe to vehicle predicted path

    people_msgs::msg::PeoplePrediction predictions;                                      //holds the people predicted steps
    nav_msgs::msg::Path collision_points_global;

    int interpolation_steps_num;
    //rclcpp::Duration interpolation_delta_time;
    double time_resolution;

    int max_cost, min_cost;                                                             //maximum and minimum cost for person space
    double gradient_cost_factor, variance_factor;
    double person_radius;

    double last_step_cost_percentage, first_step_cost_percentage;                    /*percentage relative to the highest cost
                                                                                      (at the collision point)*/

    unsigned char * map_copy;

    std::vector<geometry_msgs::msg::Point> transformed_footprint_;
};

}
#endif