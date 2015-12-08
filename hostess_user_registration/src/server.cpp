#include <cob_people_detection/addDataAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<cob_people_detection::addDataAction> Server;

void execute(const cob_people_detection::addDataGoalConstPtr& goal, Server* as)
{
	ROS_INFO("Action received!");
	cob_people_detection::addDataFeedback feedback;

	for(int i = 1; i <= goal->continuous_mode_images_to_capture; i++)
	{
		feedback.images_captured = i;
		as->publishFeedback(feedback);
		ros::Rate(10).sleep();
	}

	as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_server");
  ros::NodeHandle n;
  Server server(n, "face_calibration", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
