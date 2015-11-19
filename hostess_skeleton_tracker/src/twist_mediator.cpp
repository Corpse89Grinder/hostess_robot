#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <cob_perception_msgs/DetectionArray.h>
#include <cob_perception_msgs/Detection.h>
#include <sstream>

void commandsCallback(geometry_msgs::Twist);
void identityCallback(cob_perception_msgs::DetectionArray);

bool printed = false;
bool engaged = false;

int sequence = -1;
int currentSequence = sequence;

double ratio;

int counter = 15;

std::string user_to_track;

ros::Publisher twistPub;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ar_tracker");

	ros::NodeHandle nh;

	std::string marker_t, cmd_t, vel_t, id_t;
	std::string frame_id("camera_depth_frame");

	nh.getParam("camera_frame_id", frame_id);
	ros::param::param<std::string>("~/command_topic", cmd_t, "/commands");
	ros::param::param<std::string>("~/velocity_topic", vel_t, "/kobra/locomotion_cmd_vel");
	ros::param::param<std::string>("~/identity_topic", id_t, "/cob_people_detection/face_recognizer/face_recognitions");

    //ros::Subscriber markerSub = nh.subscribe(marker_t, 1, markerCallback);
    ros::Subscriber commandsSub = nh.subscribe(cmd_t, 1, commandsCallback);
    ros::Subscriber identitySub = nh.subscribe(id_t, 1, identityCallback);

    twistPub = nh.advertise<geometry_msgs::Twist>(vel_t, 1);

    while(!engaged)
    {
    	ros::spinOnce();
    }

    tf::TransformListener listener;

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

				//transform.getOrigin().x() distanza dal sensore in metri
				//transform.getOrigin().y() sinistra o destra (credo siano angoli o radianti)
				//transform.getOrigin().z() sopra o sotto (idem come sopra)

				std::cout << transform.getOrigin().x() << std::endl;
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

	twistPub.publish(twist);

	return;
}

void identityCallback(cob_perception_msgs::DetectionArray msg)
{
	std::vector<cob_perception_msgs::Detection> identities;

	if(!identities.empty())
	{
		if(identities[0].label != "Unknown")
		{
			user_to_track = identities[0].label;

			//tf::Vector3 position = tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)

			tf::Transform buffer;
			tf::poseStampedMsgToTF(msg.pose, buffer);

			tf_br.sendTransform(tf::StampedTransform(transform_auxiliar, ros::Time::now(), "openni_camera", "graspingPoint"));

			tf::StampedTransform transform;

			for(int i = 0; i < 15; i++)
			{
				std::ostringstream oss;
				oss << "head_" << i;

				try
				{
					listener.lookupTransform(frame_id, oss.str(), ros::Time(0), transform);

					//transform.getOrigin().x() distanza dal sensore in metri
					//transform.getOrigin().y() sinistra o destra (credo siano angoli o radianti)
					//transform.getOrigin().z() sopra o sotto (idem come sopra)

					std::cout << transform.getOrigin().x() << std::endl;
				}
				catch(tf::TransformException &ex)
				{
					//ROS_ERROR("%s",ex.what());
					continue;
				}
			}

			engaged = true;
		}
	}
}
