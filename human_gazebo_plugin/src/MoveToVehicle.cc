#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "human_gazebo_plugin/MoveToVehicle.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(MoveToVehicle)

#define WALKING_ANIMATION "walking"

/////////////////////////////////////////////////
MoveToVehicle::MoveToVehicle()
{
}

/////////////////////////////////////////////////
void MoveToVehicle::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&MoveToVehicle::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  //Read the velocity
  if (_sdf->HasElement("velocity"))
    this->velocity = _sdf->Get<double>("velocity");
  else
    this->velocity = 0.8;

  //Read the collision point with the vehicle
  if (_sdf->HasElement("collision_point"))
    this->collision_point = _sdf->Get<ignition::math::Vector3d>("collision_point");
  else
    this->collision_point = this->actor->WorldPose().Pos();

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get<std::string>());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  this->node_name = this->actor->GetName() + "node";
  this->pub_topic_name = this->actor->GetName() + "_pos";
  this->node = std::make_shared<rclcpp::Node>(node_name);
  this->actor_pos_pub = this->node->create_publisher<people_msgs::msg::Person>("/person_pos", 10);
  this->people_pos_pub = this->node->create_publisher<people_msgs::msg::People>("/people", 1);
  this->odom_sub_ = this->node->create_subscription<nav_msgs::msg::Odometry>("/odom", 1,
                    std::bind(&MoveToVehicle::odom_callback, this, std::placeholders::_1));

  this->x_map_offset = 21.15;//10.6;
  this->y_map_offset = 24.95;//10.2;

  ignition::math::Vector3d initial_pos(this->actor->WorldPose().Pos());
  this->waypoints.push_back(initial_pos);
  this->waypoints.push_back(this->target);
}

/////////////////////////////////////////////////
void MoveToVehicle::Reset()
{
  //this->velocity = 0.8;
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(5, 5, 1.2138);

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }
}


/////////////////////////////////////////////////
void MoveToVehicle::odom_callback(const nav_msgs::msg::Odometry & msg)
{
  gzerr << "Odom received";
    vehicle_pos_.Set(msg.pose.pose.position.x, msg.pose.pose.position.y, 1.2138);
    vehicle_speed_ = -msg.twist.twist.linear.x;
}


/////////////////////////////////////////////////
void MoveToVehicle::ChooseNewTarget()
{
  //ignition::math::Vector3d newTarget(this->target);
  // while ((newTarget - this->target).Length() < 2.0)
  // {
  //   newTarget.X(ignition::math::Rand::DblUniform(5.0, 10.0));
  //   newTarget.Y(ignition::math::Rand::DblUniform(5.0, 10.0));

  //   for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  //   {
  //     double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
  //         - newTarget).Length();
  //     if (dist < 2.0)
  //     {
  //       newTarget = this->target;
  //       break;
  //     }
  //   }
  // }
  //this->target = newTarget;
  static std::size_t index = 0;
  this->target = this->waypoints[index];
  if(index < this->waypoints.size()-1)
    index++;
  else
    index = 0;
}

/////////////////////////////////////////////////
void MoveToVehicle::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 4.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void MoveToVehicle::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d actor_dist_col = this->collision_point - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();
  ignition::math::Vector3d last_pos = pose.Pos();

  double distance = pos.Length();
  double actor_time_col = actor_dist_col.Length() / this->velocity.X();

  ignition::math::Vector3d v_act;
  ignition::math::Vector3d vehicle_dist_col = this->collision_point - vehicle_pos_;
  if(vehicle_speed_ > 0){
    gzerr << "Vehicle speed is" << vehicle_speed_ <<".\n";
    double vehicle_time_col = vehicle_dist_col.Length() / vehicle_speed_;

    if(actor_time_col > vehicle_time_col){
      v_act = 0;
    } else{
      v_act = this->velocity;
    }
  } else { v_act = 0; }
  

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 0.3)
  {
    this->ChooseNewTarget();
    pos = this->target - pose.Pos();
  }

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  //this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*0.001);
  }
  else
  {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }

  // Make sure the actor stays within bounds
  pose.Pos().X(std::max((-1*x_map_offset), std::min(40.0, pose.Pos().X())));
  pose.Pos().Y(std::max((-1*y_map_offset), std::min(40.0, pose.Pos().Y())));
  pose.Pos().Z(1.2138);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
  
  //ignition::math::Vector3d act_velocity = pos * v_act;
  ignition::math::Vector3d displacement  = pose.Pos() - last_pos;
  people_msgs::msg::People people;
  people_msgs::msg::Person person;
  person.name = this->actor->GetName();
  //person.velocity.x = act_velocity.X();
  person.velocity.x = displacement.X()/dt;
  person.velocity.y = displacement.Y()/dt;
  //person.velocity.y = act_velocity.Y();
  person.position.x = pose.Pos().X();// + this->x_map_offset;
  person.position.y = pose.Pos().Y();// + this->y_map_offset;;
  this->actor_pos_pub->publish(person);

  // people.header.frame_id = "map";
  // people.header.stamp = this->node->now();
  // people.people.push_back(person);
  // this->people_pos_pub->publish(people);
}