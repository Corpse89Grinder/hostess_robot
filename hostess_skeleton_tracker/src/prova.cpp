#include <ros/ros.h>
#include <std_msgs/String.h>
#include <boost/bind.hpp>

void toggleCallback(ros::NodeHandle&, const std_msgs::String::ConstPtr&);
void stringCallback(const std_msgs::String::ConstPtr&);
void initStringSubscriber(ros::NodeHandle&);

ros::Subscriber stringSub;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "prova");
	ros::NodeHandle nh;


	ros::Subscriber toggleSub = nh.subscribe<std_msgs::String>("/toggle", 1, boost::bind(&toggleCallback, boost::ref(nh), _1));
	initStringSubscriber(nh);

	ros::spin();

	return EXIT_SUCCESS;
}

void toggleCallback(ros::NodeHandle& nh, const std_msgs::String::ConstPtr& msg)
{
	if(msg->data == "ciaon")
	{
		initStringSubscriber(nh);
	}
	else if(msg->data == "ciaoff")
	{
		stringSub.shutdown();
	}
}

void stringCallback(const std_msgs::String::ConstPtr& msg)
{
	std::cout << msg->data << std::endl;
}

void initStringSubscriber(ros::NodeHandle& nh)
{
	stringSub = nh.subscribe<std_msgs::String>("/string", 1, stringCallback);
}
