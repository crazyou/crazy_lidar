#include "ros/ros.h"
#include "std_srvs/Empty.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_client");
	if(argc!=1)
	{
		ROS_INFO("usage: motor_client");
		return 1;
	}

	ros::NodeHandle n;
	ros::ServiceClient start_motor_client = n.serviceClient<std_srvs::Empty>("start_motor");
	ros::ServiceClient stop_motor_client = n.serviceClient<std_srvs::Empty>("stop_motor");
	
	std_srvs::Empty srv;
	if(start_motor_client.call(srv))
	{
		ROS_INFO("motor start");
	}
	else
	{
		ROS_ERROR("motor start failed.");
		return 1;
	}
	ros::Duration(5).sleep();
	if(stop_motor_client.call(srv))
	{
		ROS_INFO("motor stop");
	}
	else
	{
		ROS_ERROR("motor stop failed.");
		return 1;
	}
	
	
	return 0;
}


