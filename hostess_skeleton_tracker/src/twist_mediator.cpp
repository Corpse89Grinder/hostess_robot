#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
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
    //ros::Subscriber commandsSub = nh.subscribe(cmd_t, 1, commandsCallback);
    ros::Subscriber identitySub = nh.subscribe(id_t, 1, identityCallback);

    twistPub = nh.advertise<geometry_msgs::Twist>(vel_t, 1);

    ros::spin();
}

void identityCallback(cob_perception_msgs::DetectionArray msg)
{
	static tf::TransformBroadcaster br;

	std::vector<cob_perception_msgs::Detection> identities = msg.detections;

	for(int i = 0; i < identities.size(); i++)
	{
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(identities[i].pose.pose.position.x, identities[i].pose.pose.position.y, identities[i].pose.pose.position.z));
		transform.setRotation(tf::Quaternion(1, 1, 1, 1));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_optical_frame", identities[i].label));

		std::cout << "Mandato" << std::endl;
	}

	return;
}
