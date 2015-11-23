#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <cob_perception_msgs/DetectionArray.h>
#include <cob_perception_msgs/Detection.h>
#include <sstream>

void identityCallback(cob_perception_msgs::DetectionArray);

ros::Time received;
std::vector<tf::StampedTransform> face_transforms;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ar_tracker");

	ros::NodeHandle nh;

	std::string id_t;
	std::string frame_id("camera_depth_frame");

	nh.getParam("camera_frame_id", frame_id);
	ros::param::param<std::string>("~/identity_topic", id_t, "/cob_people_detection/face_recognizer/face_recognitions");

    ros::Subscriber identitySub = nh.subscribe(id_t, 1, identityCallback);

    tf::TransformListener listener;

    ros::Rate rate(10.0);

    tf::TransformBroadcaster br;

    while(nh.ok())
    {
    	tf::StampedTransform transform;

    	//std::vector<tf::StampedTransform> transforms;

    	for(int i = 1; i <= 15; i++)
    	{
    		std::ostringstream oss;
    		oss << "head_" << i;

    		try
    		{
    			listener.lookupTransform("/camera_depth_frame", oss.str(), received, transform);

    			for(int j = 0; j < face_transforms.size(); j++)
    			{
    				double dist = std::sqrt(std::pow(transform.getOrigin().getX() - face_transforms[j].getOrigin().getX(), 2) + std::pow(transform.getOrigin().getY() - face_transforms[j].getOrigin().getY(), 2) + std::pow(transform.getOrigin().getZ() - face_transforms[j].getOrigin().getZ(), 2));
    				std::cout << face_transforms[j].child_frame_id_ << " " << dist << std::endl;
    			}

    			//transforms.push_back(transform);
    		}
    		catch(tf::TransformException ex)
    		{
    			continue;
    		}
    	}

    	//for(int i = 0; i < transforms.size(); i++)
    	//{
    	//	std::ostringstream oss;
		//	oss << "testa_" << i + 1;
//
    	//	br.sendTransform(tf::StampedTransform(transforms[i], ros::Time::now(), "/camera_depth_frame", oss.str()));
    	//}

    	rate.sleep();

    	ros::spinOnce();
    }
}

void identityCallback(cob_perception_msgs::DetectionArray msg)
{
	static tf::TransformBroadcaster br;

	std::vector<cob_perception_msgs::Detection> identities = msg.detections;
	std::vector<tf::StampedTransform> transforms;

	for(int i = 0; i < identities.size(); i++)
	{
		if(identities[i].detector != "face")
		{
			continue;
		}

		tf::Transform transform;
		transform.setOrigin(tf::Vector3(identities[i].pose.pose.position.x, identities[i].pose.pose.position.y, identities[i].pose.pose.position.z));
		transform.setRotation(tf::Quaternion(identities[i].pose.pose.orientation.x, identities[i].pose.pose.orientation.y, identities[i].pose.pose.orientation.z, identities[i].pose.pose.orientation.w));
		transforms.push_back(tf::StampedTransform(transform, ros::Time::now(), identities[i].header.frame_id, identities[i].label));

		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), identities[i].header.frame_id, identities[i].label));
	}

	if(!transforms.empty())
	{
		received = ros::Time::now();
		face_transforms = transforms;
	}

	return;
}
