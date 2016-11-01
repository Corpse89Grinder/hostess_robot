#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <vector>
#include <sstream>
#include <fstream>
#include <ctime>

void loggerCallback(const std_msgs::String::ConstPtr&);

std::vector<std::string> lines;

bool started = false;
std::stringstream current_filename;

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
		current_filename.str() = "";
		return;
	}

	if(msg->data == "succeeded")
	{
		std::ofstream stream;
		stream.open(current_filename.str().c_str(), std::ios::app);

		for(int i = 0; i < lines.size(); i++)
		{
			stream << lines[i] << std::endl;
		}

		stream << std::endl;

		stream.close();
	}
	else
	{
		lines.push_back(msg->data);

		if(lines.size() == 1)
		{
			time_t now = time(0);
			std::tm* time = localtime(&now);

			current_filename << ros::package::getPath("hostess_full") << "/log/";
			current_filename << lines[0] << "_";
			current_filename << 1900 + time->tm_year << "-" << 1 + time->tm_mon << "-" << time->tm_mday << "_";
			current_filename << 1 + time->tm_hour << "-" << 1 + time->tm_min << "-" << 1 + time->tm_sec;
		}
	}
}
