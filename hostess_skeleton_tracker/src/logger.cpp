#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <string>
#include <fstream>

void loggerCallback(const std_msgs::String::ConstPtr&);

std::vector<std::string> lines;

bool started = false;
std::string current_filename = "";

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hostess_logger");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("logger", 10, loggerCallback);

	ros::spin();

	return EXIT_SUCCESS;
}

void loggerCallback(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data == "start")
	{
		started = true;
		lines.clear();
		current_filename = "";
		return;
	}

	if(msg->data == "succeeded")
	{
		std::ofstream stream;
		stream.open(current_filename.c_str(), std::ios::app);

		for(int i = 0; i < lines.size(); i++)
		{
			stream << lines[i] << std::endl;
		}

		stream.close();

		current_filename = "";
	}
	else
	{
		lines.push_back(msg->data);

		if(lines.size() == 1)
		{
			current_filename = lines[0] + ".txt";
		}
	}
}
