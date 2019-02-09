#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <iostream>
#include <functional>

#include <ignition/math.hh>
#include "gazebo/physics/physics.hh"
#include "ActorPluginMod.hh"  

#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include "geometry_msgs/Twist.h"

namespace gazebo
{
  // class WorldPluginTutorial : public WorldPlugin
  // {
  //   /// \brief Constructor
  //     public: WorldPluginTutorial();

  //     /// \brief Destructor
  //     public: virtual ~WorldPluginTutorial();

  //     public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf );
  //     /// \brief Update the controller
      
  //     protected: virtual void UpdateChild();
  // }; 
 
  // GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)

  // #define WALKING_ANIMATION "walking"
  // ////////////////////////////////////////////////////////////////////////////////
  // // Constructor
  // WorldPluginTutorial::WorldPluginTutorial()
  // {
  // }

  // ////////////////////////////////////////////////////////////////////////////////
  // // Destructor
  // WorldPluginTutorial::~WorldPluginTutorial()
  // {
  // }

  // ////////////////////////////////////////////////////////////////////////////////
  // // Load the controller
  // void WorldPluginTutorial::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
  // {
  //   // Make sure the ROS node for Gazebo has already been initalized
    // if (!ros::isInitialized())
    // {
    //   ROS_FATAL_STREAM_NAMED("template", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
    //     << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    //   return;
    // }
    //  ROS_INFO("Hello World!");

  // }


  // ////////////////////////////////////////////////////////////////////////////////
  // // Update the controller
  // void WorldPluginTutorial::UpdateChild()
  // {
  // }
#define WALKING_ANIMATION "walking"
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)
/////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Initialize ros, if it has not already bee initialized.
if (!ros::isInitialized())
{
  int argc = 0;
  char **argv = NULL;
  ros::init(argc, argv, "gazebo_client",
      ros::init_options::NoSigintHandler);
}
ROS_INFO("Hello World!");

// Create our ROS node. This acts in a similar manner to
// the Gazebo node
this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

// Create a named topic, and subscribe to it.
ros::SubscribeOptions so =
  ros::SubscribeOptions::create<geometry_msgs::Twist>("/set_model_state", 1, boost::bind(&ActorPlugin::OnRosMsg, this, _1), ros::VoidPtr(), &this->rosQueue);
this->rosSub = this->rosNode->subscribe(so);

// Spin up the queue helper thread.
this->rosQueueThread =
  std::thread(std::bind(&ActorPlugin::QueueThread, this));

  this->sdf = _sdf;
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

  this->Reset();

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.0;

  // Read in the obstacle weight
  // if (_sdf->HasElement("obstacle_weight"))
  //   this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  // else
  //   this->obstacleWeight = 1.0;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 1.0;

  // Add our own name to models we should ignore when avoiding obstacles.
  // this->ignoreModels.push_back(this->actor->GetName());

  // // Read in the other obstacles to ignore
  // if (_sdf->HasElement("ignore_obstacles"))
  // {
  //   sdf::ElementPtr modelElem =
  //     _sdf->GetElement("ignore_obstacles")->GetElement("model");
  //   while (modelElem)
  //   {
  //     this->ignoreModels.push_back(modelElem->Get<std::string>());
  //     modelElem = modelElem->GetNextElement("model");
  //   }
  // }
}

/////////////////////////////////////////////////
void ActorPlugin::Reset()
{
  this->velocity = 0.8;
  this->lastUpdate = 0;

  if (this->sdf && this->sdf->HasElement("target"))
    this->target = this->sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(5, 5, 0);

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
void ActorPlugin::ChooseNewTarget()
{
  ignition::math::Vector3d newTarget(this->target);
  // while ((newTarget - this->target).Length() < 1)
  // {
    newTarget.X(pos_g.x);
    newTarget.Y(pos_g.y);
    //std::cout<<pos<<std::endl;
    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
          - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
  //}
  this->target = newTarget;
}

/////////////////////////////////////////////////
// void ActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
// {
//   for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
//   {
//     physics::ModelPtr model = this->world->ModelByIndex(i);
//     if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
//           model->GetName()) == this->ignoreModels.end())
//     {
//       ignition::math::Vector3d offset = model->WorldPose().Pos() -
//         this->actor->WorldPose().Pos();
//       double modelDist = offset.Length();
//       if (modelDist < 4.0)
//       {
//         double invModelDist = this->obstacleWeight / modelDist;
//         offset.Normalize();
//         offset *= invModelDist;
//         _pos -= offset;
//       }
//     }
//   }
// }

/////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();


  double distance = pos.Length();

  // Choose a new target position if the actor has reached its current
  // target.
  //std::cout << distance << std::endl;
  // if (distance < 1.3)
  // {
  //   this->ChooseNewTarget();
  //   pos = this->target - pose.Pos();
  // }
  ignition::math::Vector3d newTarget(this->target);
  // while ((newTarget - this->target).Length() < 1)
  // {
  newTarget.X(pos_g.x);
  newTarget.Y(pos_g.y);
  std::cout<<pos_g<<std::endl;
  this->target = newTarget;

  //Normalize the direction vector, and apply the target weight
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
        yaw.Radian()*0.1);
  }
  else
  {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }

  // Make sure the actor stays within bounds
  pose.Pos().X(std::max(-10000000.0, std::min(100000000.0, pose.Pos().X())));
  pose.Pos().Y(std::max(-10000000.0, std::min(100000000.0, pose.Pos().Y())));
  pose.Pos().Z(1.0);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
    (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}

}







//GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
//}
