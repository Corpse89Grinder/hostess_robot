#include <cob_people_detection/addDataAction.h>
#include <cob_people_detection/deleteDataAction.h>
#include <cob_people_detection/loadModelAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<cob_people_detection::addDataAction> AddDataServer;
typedef actionlib::SimpleActionServer<cob_people_detection::deleteDataAction> DeleteDataServer;
typedef actionlib::SimpleActionServer<cob_people_detection::loadModelAction> LoadModelServer;

void executeAdd(const cob_people_detection::addDataGoalConstPtr& goal, AddDataServer* as)
{
	ROS_INFO("Action add received!");
	cob_people_detection::addDataFeedback feedback;

	for(int i = 1; i <= goal->continuous_mode_images_to_capture; i++)
	{
		feedback.images_captured = i;
		as->publishFeedback(feedback);
		ros::Rate(10).sleep();
	}

	as->setSucceeded();
}

void executeDelete(const cob_people_detection::deleteDataGoalConstPtr& goal, DeleteDataServer* as)
{
	ROS_INFO("Action delete received!");

	as->setSucceeded();
}

void executeLoad(const cob_people_detection::loadModelGoalConstPtr& goal, LoadModelServer* as)
{
	ROS_INFO("Action load model received!");

	as->setSucceeded();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "action_server");
	ros::NodeHandle nh;

	AddDataServer addDataServer(nh, "add_user", boost::bind(&executeAdd, _1, &addDataServer), false);
	DeleteDataServer deleteDataServer(nh, "delete_user", boost::bind(&executeDelete, _1, &deleteDataServer), false);
	LoadModelServer loadModelServer(nh, "load_model", boost::bind(&executeLoad, _1, &loadModelServer), false);

	addDataServer.start();
	deleteDataServer.start();
	loadModelServer.start();

	ros::spin();

	return EXIT_SUCCESS;
}
