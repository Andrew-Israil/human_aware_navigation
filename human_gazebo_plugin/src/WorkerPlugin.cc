#include "human_gazebo_plugin/WorkerPlugin.hh"
#include <ignition/math.hh>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(WorkerPlugin);

WorkerPlugin::WorkerPlugin(){};

void WorkerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    // Store the pointer to the model
    this->model = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    // this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
    //       std::bind(&WorkerPlugin::OnUpdate, this, std::placeholders::_1)));
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&WorkerPlugin::OnUpdate, this));

    this->lastUpdate = this->model->GetWorld()->RealTime();
    //this->lastUpdate = 0;

    if (_sdf->HasElement("target"))
        this->target = _sdf->Get<ignition::math::Vector3d>("target");
    else
        this->target = ignition::math::Vector3d(5, 5, 1.2138);

    //Read the velocity
    if (_sdf->HasElement("velocity"))
        this->velocity = _sdf->Get<double>("velocity");
    else
        this->velocity = 0.8;
    
    if (_sdf->HasElement("delay_to_target"))
        this->delay_to_target = _sdf->Get<double>("delay_to_target");
    else
        this->delay_to_target = 0.0;
    
    this->x_map_offset = 21.15;//10.6;
    this->y_map_offset = 24.95;//10.2;

    ignition::math::Vector3d initial_pos(this->model->WorldPose().Pos());
    this->waypoints.push_back(initial_pos);
    this->waypoints.push_back(this->target);

    this->node_name = this->model->GetName() + "node";
    this->pub_topic_name = this->model->GetName() + "_pos";
    this->node = std::make_shared<rclcpp::Node>(node_name);
    this->person_pos_pub = this->node->create_publisher<people_msgs::msg::Person>("/person_pos", 10);
}

void WorkerPlugin::ChooseNewTarget()
{
  static std::size_t index = 0;
  this->target = this->waypoints[index];
  if(index < this->waypoints.size()-1)
    index++;
  else
    index = 0;
}

// void WorkerPlugin::OnUpdate(const common::UpdateInfo &_info)
// {
//   // Time delta
//   double dt = (_info.simTime - this->lastUpdate).Double();
//   static double counter = 0.0;

//   ignition::math::Pose3d pose = this->model->WorldPose();
//   ignition::math::Vector3d pos = this->target - pose.Pos();
//   ignition::math::Vector3d rpy = pose.Rot().Euler();
//   ignition::math::Vector3d last_pos = pose.Pos();

//   double distance = pos.Length();

//     gzdbg << "x:" << rpy.X();
//     gzdbg << "y:" << rpy.Y();
//     gzdbg << "z:" << rpy.Z();

//   // Choose a new target position if the actor has reached its current
//   // target.
//   if (distance < 0.3)
//   {
//     counter += dt;
//     if(counter >= delay_to_target){
//       this->ChooseNewTarget();
//       pos = this->target - pose.Pos();
//       counter = 0;
//     }
    
//   }

//   // Normalize the direction vector, and apply the target weight
//   pos = pos.Normalize();

//   // Compute the yaw orientation
//   ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
//   yaw.Normalize();

//   // Rotate in place, instead of jumping.
//   if (std::abs(yaw.Radian()) > IGN_DTOR(10))
//   {
//     pose.Rot() = ignition::math::Quaterniond(0, 0, rpy.Z()+
//         yaw.Radian()*0.001);
//   }
//   else
//   {
//     pose.Pos() += pos * this->velocity * dt;
//     pose.Rot() = ignition::math::Quaterniond(0, 0, rpy.Z()+yaw.Radian());
//   }

//   // Make sure the actor stays within bounds
//   pose.Pos().X(std::max((-1*x_map_offset), std::min(40.0, pose.Pos().X())));
//   pose.Pos().Y(std::max((-1*y_map_offset), std::min(40.0, pose.Pos().Y())));
//   //pose.Pos().Z(1.2138);

//   this->model->SetWorldPose(pose, false, false);
//   this->lastUpdate = _info.simTime;
// }

void WorkerPlugin::OnUpdate()
{
  // Time delta
  double dt = (this->model->GetWorld()->RealTime() - this->lastUpdate).Double();
  static double counter = 0.0;

  ignition::math::Pose3d pose = this->model->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();
  ignition::math::Vector3d last_pos = pose.Pos();

  double distance = pos.Length();

  // Choose a new target position if the actor has reached its current
  // target.
  if (distance < 0.3)
  {
    counter += dt;
    if(counter >= delay_to_target){
      this->ChooseNewTarget();
      pos = this->target - pose.Pos();
      counter = 0;
    }
    
  }

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize();

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(0, 0, rpy.Z()+
        yaw.Radian()*0.001);
  }
  else
  {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(0, 0, rpy.Z()+yaw.Radian());
  }

  // Make sure the actor stays within bounds
  //pose.Pos().X(std::max((-1*x_map_offset), std::min(40.0, pose.Pos().X())));
  //pose.Pos().Y(std::max((-1*y_map_offset), std::min(40.0, pose.Pos().Y())));
  //pose.Pos().Z(1.2138);
  this->model->SetGravityMode(false);
  this->model->SetWorldPose(pose);
  this->lastUpdate = this->model->GetWorld()->RealTime();

  people_msgs::msg::Person person;
  person.name = this->model->GetName();
  person.position.x = pose.Pos().X();
  person.position.y = pose.Pos().Y();
  this->person_pos_pub->publish(person);
}
