#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"


#include <stdlib.h>

void dalekCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
	{
	
		
		ROS_INFO("Location@[%f, %f,  %f]", msg->pose[1].position.x, msg->pose[1].position.y, msg->pose[1].position.z);
			
	}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "dalekEars");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("gazebo/model_states", 1000, dalekCallback);

	

	ros::spin();

	return 0;
}

