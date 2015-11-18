#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

void commandsCallback(geometry_msgs::Twist twist);

bool printed = false;

int sequence = -1;
int currentSequence = sequence;

double ratio;

int counter = 15;

ros::Publisher pub;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ar_tracker");

	ros::NodeHandle nh;

	std::string marker_t, cmd_t, vel_t;
	std::string frame_id("camera_depth_frame");

	nh.getParam("camera_frame_id", frame_id);
	ros::param::param<std::string>("~/command_topic", cmd_t, "commands");
	ros::param::param<std::string>("~/velocity_topic", vel_t, "/kobra/locomotion_cmd_vel");


    //ros::Subscriber markerSub = nh.subscribe(marker_t, 1, markerCallback);
    ros::Subscriber commandsSub = nh.subscribe(cmd_t, 1, commandsCallback);

    tf::TransformListener listener;

    pub = nh.advertise<geometry_msgs::Twist>(vel_t, 1);

    while(nh.ok())
    {
    	tf::StampedTransform transform;

    	for(int i = 0; i < 15; i++)
    	{
    		std::ostringstream oss;
    		oss << "torso_" << i;

			try
			{
				listener.lookupTransform(frame_id, oss.str(), ros::Time(0), transform);

				std::cout << transform.getOrigin().z() << std::endl;
			}
			catch(tf::TransformException &ex)
			{
				//ROS_ERROR("%s",ex.what());
				continue;
			}
    	}

    	ros::spinOnce();
    }

    ros::shutdown();
    return 0;
}

/*void markerCallback(ar_pose::ARMarker marker)
{


	if(distance >= 0 && distance <= 1)
	{
		ratio = 1;
	}
	else if(distance > 1 && distance <= 2)
	{
		ratio = 1 - (distance - 1);
	}
	else if(distance > 2)
	{
		ratio = 0;
	}

	if(ratio < 0)
	{
		ratio = 0;
	}
	else if(ratio > 1)
	{
		ratio = 1;
	}

	currentSequence = marker.header.seq;

	return;
}*/

void commandsCallback(geometry_msgs::Twist received)
{
	geometry_msgs::Twist twist;

	if(counter == 15)
	{
		twist.linear.x = 0;
		twist.angular.z = 0;
	}

	if(currentSequence == sequence)
	{
		counter++;
	}
	else
	{
		counter = 0;
		sequence = currentSequence;

		twist.linear.x = received.linear.x * ratio;
		twist.angular.z = received.angular.z * ratio;
	}

	pub.publish(twist);

	return;
}
