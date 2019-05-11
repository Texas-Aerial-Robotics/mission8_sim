#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <gazebo_msgs/ModelStates.h> 
#include <geometry_msgs/Pose.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::PoseStamped actorstate;

void model_cb(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
  gazebo_msgs::ModelStates current_states = *msg;
  int actorArrPos = 999;

  //search for the drone
  for (int i=0; i< current_states.name.size(); i++)
  {
  	if(current_states.name[i] == "actor")
  	{
  		actorArrPos = i;
  		break;
  	}
  	
  }
  if (actorArrPos == 999)
  {
  	std::cout << "actor is not in world" << std::endl;
  }else{
  	//assign actor's pose to pose stamped
  	actorstate.pose = current_states.pose[actorArrPos];
  	actorstate.header.stamp = ros::Time::now();
    std::cout <<  actorstate << std::endl;

    //do transformed stamped stuff
    transformStamped.header.stamp = ros::Time::now();
  	transformStamped.header.frame_id = "world";
  	// not sure if correct child frame...maybe it's "head" of the actor?
  	transformStamped.child_frame_id = actor;
 	transformStamped.transform.translation.x = msg->x;
  	transformStamped.transform.translation.y = msg->y;
  	transformStamped.transform.translation.z = 0.0;
  	tf2::Quaternion q;
  	q.setRPY(0, 0, msg->theta);
  	transformStamped.transform.rotation.x = q.x();
  	transformStamped.transform.rotation.y = q.y();
  	transformStamped.transform.rotation.z = q.z();
  	transformStamped.transform.rotation.w = q.w();

  	br.sendTransform(transformStamped);

}
int main(int argc, char **argv) {
	ros::init(argc, argv, "acotr_tf2_broadcaster");
	ros::NodeHandle n;
	ros::Subscriber currentHeading = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 10, model_cb);
	ros::Rate rate(20.0);

	ros::spin()
	return 0;
};