#include "nav2_human_dynamic_layer_costmap_plugin/human_dynamic_layer.hpp"

#include "string"
#include <chrono>
#include <cmath>
using namespace std::chrono;
using namespace std::chrono_literals;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_human_dynamic_layer_costmap_plugin
{

HumanDynamicLayer::HumanDynamicLayer()
{
}

void HumanDynamicLayer::onInitialize()
{
    auto node = node_.lock(); 
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "DynamicLayer: initialized");
    matchSize();
    current_ = true;

    prediction_sub_ = node->create_subscription<people_msgs::msg::PeoplePrediction>("/people_prediction", rclcpp::SystemDefaultsQoS(), 
                      std::bind(&HumanDynamicLayer::prediction_callback, this, std::placeholders::_1));
    collisions_sub_ = node->create_subscription<nav_msgs::msg::Path>("/predicted_collisions", rclcpp::SystemDefaultsQoS(),
                        std::bind(&HumanDynamicLayer::collisions_callback, this, std::placeholders::_1));
                        
    time_resolution = 0.5; //in seconds, the same as the prediction time resolution

    max_cost = 254;
    min_cost = 50;
    gradient_cost_factor = 0.001; //linear magnitude decay factor
    //variance_factor = 0.01;//0.02;
    person_radius = 1.2; //personal zone (in meter)

    last_step_cost_percentage = 0.95;
    first_step_cost_percentage = 0.9;
}

void HumanDynamicLayer::prediction_callback(people_msgs::msg::PeoplePrediction::SharedPtr msg)
{
  predictions = *msg;
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Human predictions received");
}

void HumanDynamicLayer::collisions_callback(nav_msgs::msg::Path::SharedPtr msg)
{
  collision_points_global = *msg;
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Collision points received");
}

void HumanDynamicLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw, double * min_x,
  double * min_y, double * max_x, double * max_y)
{
  nav2_costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);
}  

void HumanDynamicLayer::onFootprintChanged()
{
  RCLCPP_DEBUG(rclcpp::get_logger(
      "nav2_costmap_2d"), "HumanDynamicLayer::onFootprintChanged(): num footprint points: %lu",
    layered_costmap_->getFootprint().size());
}


double HumanDynamicLayer::calc_cutoff_radius(double cutoff_value, double amplitude, double variance)
{
  if(cutoff_value <= 0)
  cutoff_value = 1;
  return sqrt(-2*variance * log(cutoff_value/amplitude));
}

//calculate the cost value at the cell
double HumanDynamicLayer::calc_gaussian(double pos_x, double pos_y, double origin_x,
                                  double origin_y, double amplitude, double variance_side,
                                  double variance_front, double variance_back, double skew)
{
  double dx = pos_x-origin_x;
  double dy = pos_y-origin_y;
  double distance = sqrt(dx*dx+dy*dy);
  double angle_abs = atan2(dy,dx); // angle of the grid cell in the map frame
  double angle = abs(angle_abs - skew); // angle in the person local frame
  if(angle > M_PI){
    angle = (2*M_PI) - angle;
  }
  double mx = cos(angle) * distance;
  double my = sin(angle) * distance;
  double f1, f2;

  if(angle < M_PI/2) //front side (elliptical shape)
  {
    f1 = pow(mx, 2.0)/(2.0 * variance_front);
    f2 = pow(my, 2.0)/(2.0 * variance_side);
    return amplitude * exp(-(f1 + f2));
  }
  else // rear side (circural shape)
  {
    f1 = pow(mx, 2.0)/(2.0 * variance_back);
    f2 = pow(my, 2.0)/(2.0 * variance_back);
    return amplitude * exp(-(f1 + f2));
  }
}

//assiegn the cost distribution at the prection step
void HumanDynamicLayer::define_human_space(
  double person_position_x, double person_position_y, double person_angle,
  double person_velocity, unsigned char * map_layer, nav2_costmap_2d::Costmap2D & master_grid, 
  double cost_factor, int index, unsigned char * map_copy)
{
  int map_size_x = master_grid.getSizeInCellsX(), map_size_y = master_grid.getSizeInCellsY();
  double map_resolution = master_grid.getResolution();

  double variance_front, variance_side, variance_back;
  if(person_velocity > 0.09) //assymetric guassian distribution for moving person
  {
    variance_front = 4*person_velocity;
    variance_side = COVARIANCE_SIDE * variance_front;
    variance_back = variance_side;// standard deviation is the same for x and y
  }
  else //symetric guassian distribution for standing person
  {
    variance_front = 1.2;
    variance_side = variance_front;
    variance_back = variance_front;
  }

/*reduce the distribution magnitude at each step through
  gradient_cost_factor for no collision (linear decay)
  cost_factor for collision occurence*/
  int max_cost_current = max_cost * cost_factor * (1 - (index * gradient_cost_factor));
  int min_cost_current = min_cost * cost_factor * (1 - (index * gradient_cost_factor));
  int lethal_cost_current = LETHAL_OBSTACLE * cost_factor * (1 - (index * gradient_cost_factor));

  double cutoff_radius_front = std::max(calc_cutoff_radius(min_cost_current, max_cost_current, variance_side),
                                        calc_cutoff_radius(min_cost_current, max_cost_current, variance_front)); 
  double cutoff_radius_back = calc_cutoff_radius(min_cost_current, max_cost_current, variance_back);

  double cutoff_radius_max = std::max(cutoff_radius_front, cutoff_radius_back);
  int cutoff_radius_max_cells = std::min(static_cast<int>(cutoff_radius_max / map_resolution),
                                          std::max(map_size_x, map_size_y)
                                        );

  int origin_x = static_cast<int>(person_position_x / map_resolution);
  int origin_y = static_cast<int>(person_position_y / map_resolution);

  int x_map_min = std::max(0, (origin_x - cutoff_radius_max_cells));
  int x_map_max = std::min(map_size_x, (origin_x + cutoff_radius_max_cells));
  int y_map_min = std::max(0, (origin_y - cutoff_radius_max_cells));
  int y_map_max = std::min(map_size_y, (origin_y + cutoff_radius_max_cells));

  for(unsigned int i=x_map_min; i<x_map_max; i++)
  {
    for(unsigned int j=y_map_min; j<y_map_max; j++)
    {
      int index = master_grid.getIndex(i, j);
      map_copy[index] = master_grid.getCost(i, j);

      if(( hypot((i*map_resolution) - person_position_x, (j*map_resolution) - person_position_y) < person_radius ))
      {
        //assign highest cost to the personal zone
        map_layer[index] = std::max(map_layer[index], static_cast<unsigned char>(lethal_cost_current));
      }
      else
      {
        double cost = calc_gaussian(i*map_resolution, j*map_resolution, person_position_x, person_position_y,
                                    max_cost_current, variance_side, variance_front, variance_back, person_angle);
        if(cost < min_cost)
          map_layer[index] = std::max(map_layer[index], static_cast<unsigned char>(0));
        else if(static_cast<unsigned char>(cost) > LETHAL_OBSTACLE)
          map_layer[index] = LETHAL_OBSTACLE;
        else 
          map_layer[index] = std::max(map_layer[index], static_cast<unsigned char>(cost));
      }
    }
  }
}


void HumanDynamicLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
  int max_i,
  int max_j)
{
  min_i = 0;
  min_j = 0;
  max_i = master_grid.getSizeInCellsX();
  max_j = master_grid.getSizeInCellsY();

  static bool first_iteration = true;
 
  if(first_iteration)
  {
    /*initial cost of the cells that will be updated by the human layer
     are stored in the map_copy */
    map_copy = new unsigned char[max_i*max_j];
    for(unsigned int i=min_i; i<max_i; ++i)
    {
      for(unsigned int j=min_j; j<max_j; ++j)
      {
        int index = master_grid.getIndex(i, j);
        costmap_[index] = FREE_SPACE;
        map_copy[index] = FREE_SPACE;
      }
    }
    first_iteration = false;
  }  
  else
  {
    for(unsigned int i=min_i; i<max_i; ++i)
    {
      for(unsigned int j=min_j; j<max_j; ++j)
      {
        int index = master_grid.getIndex(i, j);

        if(costmap_[index] != FREE_SPACE)
        {
        /*clear the old values of the previous  update done by this layer
        and restore the initial value from the map_copy */
          master_grid.setCost(i, j, map_copy[index]);
          map_copy[index] = FREE_SPACE;
          costmap_[index] = FREE_SPACE;
        }
      }
    }
  }
  
  nav_msgs::msg::Path collision_points = collision_points_global;

  people_msgs::msg::People people_k; //people in one timestep
  people_msgs::msg::People people_kplus1; //people in the next timestep

  people_msgs::msg::Person person_k; //person in one timestep
  people_msgs::msg::Person person_kplus1; //person in the next timestep
  
  double angle_k; //angle in one timestep
  double angle_kplus1; //angle in the next timestep
  
  double person_x, person_y, person_angle; //person pose
  double person_velocity; //person velocity magnitude


  /*calculate the variance of the cost factor distribution along the predicted path 
    the highest cost factor =1 is located at the collison point
    the variance in front of the collision point is higher than behind it to motivate the passage 
    behind the collision point */
  std::vector<std::vector<double>>min_variances(collision_points.poses.size(),std::vector<double>(2));
  for(int i=0; i< static_cast<int>(min_variances.size()); i++)
  {
    /*check if there is a collison with the detected persons
    if yes, calculate the distance between the collison point and the first and last point of the predicted human steps */
    if(collision_points.poses[i].header.stamp != rclcpp::Time(0))
    {
      double distance_back = hypot((predictions.predicted_people[0].people[i].position.x -collision_points.poses[i].pose.position.x),
                                   (predictions.predicted_people[0].people[i].position.y -collision_points.poses[i].pose.position.y));
      double distance_front = hypot((predictions.predicted_people.back().people[i].position.x -collision_points.poses[i].pose.position.x),
                                   (predictions.predicted_people.back().people[i].position.y -collision_points.poses[i].pose.position.y));
      
      //calculate the variance in front and behind the collision point
      if(distance_back > 0.05 && distance_front > 0.05) // collison point is not at the first or last step
      {
        //variance in the rear is set so the lowest cost is 90% of the highest cost at the collision point
        min_variances[i][0] = pow(distance_back, 2)/(-2*log(first_step_cost_percentage));
        //variance in the front is set so the lowest cost is 95% of the highest cost at the collision point
        min_variances[i][1] = 2*pow(distance_front, 2)/(-2*log(last_step_cost_percentage));
      }
      else if(distance_front <= 0.05) // collision is at the last prediction step
      {
        double distance = std::max(distance_back, distance_front);
        min_variances[i][0] = pow(distance, 2)/(-2*log(first_step_cost_percentage));
        min_variances[i][1] = min_variances[i][0];
      }
      
    }
  }

  for(int i=0; i<(static_cast<int>(predictions.predicted_people.size())); ++i)
  {
    people_k = predictions.predicted_people.at(i);

    rclcpp::Duration delta_timestamp = rclcpp::Time(people_kplus1.header.stamp) - rclcpp::Time(people_k.header.stamp);

    for(int j=0; j<static_cast<int>(people_k.people.size()); ++j)
    {
      person_k = people_k.people.at(j);
      angle_k = atan2(person_k.velocity.y, person_k.velocity.x);

      person_velocity = sqrt(pow(person_k.velocity.x, 2) + pow(person_k.velocity.y, 2));

      double cost_factor;
      int index;
      if(collision_points.poses[j].header.stamp != rclcpp::Time(0)) //collision case
      {
        cost_factor = calc_gaussian(person_k.position.x, person_k.position.y, collision_points.poses[j].pose.position.x, 
        collision_points.poses[j].pose.position.y, 1.0, min_variances[j][1], min_variances[j][1], min_variances[j][0], angle_k);
        index = 0;
      }
      else if(i<8){ // 4 seconds was enough for the case of no collision
        cost_factor = 1;
        index = i;
      } else {continue;}

      define_human_space(person_k.position.x - master_grid.getOriginX(), person_k.position.y - master_grid.getOriginY(), 
      angle_k, person_velocity, costmap_, master_grid, cost_factor, index, map_copy);
 
    }
  }
  setConvexPolygonCost(transformed_footprint_, nav2_costmap_2d::FREE_SPACE); //do not add cost at the vehicle's footprint
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  predictions.predicted_people.clear();
}  

}//end of nav2_hhuman_dynamic_layer_costmap_plugin namespace
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_human_dynamic_layer_costmap_plugin::HumanDynamicLayer, nav2_costmap_2d::Layer)