#include<ros/ros.h>
#include<tf/transform_listener.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "hostess_tracker");

	ros::NodeHandle nh;

	ros::service::waitForService()
}
