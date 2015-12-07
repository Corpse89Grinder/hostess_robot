#include <cob_people_detection/addDataAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<cob_people_detection::addDataAction> Server;

void execute(const cob_people_detection::addDataGoalConstPtr& goal, Server* as)
{
  // Do lots of awesome groundbreaking robot stuff here
  as->setSucceeded();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "action_server");
  ros::NodeHandle n;
  Server server(n, "prova", boost::bind(&execute, _1, &server), false);
  server.start();
  ros::spin();
  return 0;
}
