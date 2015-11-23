#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <cob_perception_msgs/DetectionArray.h>
#include <cob_perception_msgs/Detection.h>
#include <sstream>

void identityCallback(cob_perception_msgs::DetectionArray);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ar_tracker");

	ros::NodeHandle nh;

	std::string id_t;
	std::string frame_id("camera_depth_frame");

	nh.getParam("camera_frame_id", frame_id);
	ros::param::param<std::string>("~/identity_topic", id_t, "/cob_people_detection/face_recognizer/face_recognitions");

    ros::Subscriber identitySub = nh.subscribe(id_t, 1, identityCallback);

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
		transform.setRotation(tf::Quaternion(rand() % 50, rand() % 50, rand() % 50, 1));
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), identities[i].header.frame_id, identities[i].label));
	}

	return;
}
