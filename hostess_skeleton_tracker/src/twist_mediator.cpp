#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <cob_perception_msgs/DetectionArray.h>
#include <cob_perception_msgs/Detection.h>
#include <sstream>
#include <map>
#include <limits>

void userToTrackIdentityCallback(std_msgs::String);
void userToTrackFaceCallback(cob_perception_msgs::DetectionArray);
void faceCallback(cob_perception_msgs::DetectionArray);

std::string user_to_track;
bool got_user = false;
bool auto_engage = false;

std::string frame_id("camera_depth_optical_frame");

ros::Time received;

std::map<std::string, int> faces_counter;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ar_tracker");

	ros::NodeHandle nh;

	std::string face_topic("/cob_people_detection/face_recognizer/face_recognitions");
	std::string identity_topic("/hostess_skeleton_tracker/user_to_track");

	nh.getParam("camera_frame_id", frame_id);
	nh.getParam("face_topic", face_topic);
	nh.getParam("identity_topic", identity_topic);

	ros::Subscriber identitySub;

    if(!auto_engage)
    {
    	identitySub = nh.subscribe(identity_topic, 1, userToTrackIdentityCallback);
    }
    else
    {
    	identitySub = nh.subscribe(face_topic, 1, userToTrackFaceCallback);
    }

    ros::Rate rate(10.0);

    while(!got_user)
    {
    	ros::spinOnce();

    	rate.sleep();
    }

    identitySub.shutdown();

    std::cout << "Tracking user: " << user_to_track << std::endl;

    tf::TransformListener listener;

    tf::TransformBroadcaster br;

    ros::Subscriber faceSub = nh.subscribe(face_topic, 1, faceCallback);

    while(nh.ok())
    {
    	ros::spinOnce();

    	rate.sleep();

    	tf::StampedTransform transform;

    	std::vector<tf::StampedTransform> transforms;

    	for(int i = 1; i <= 15; i++)
    	{
    		std::ostringstream oss;
    		oss << "recognition_head_" << i;

    		try
    		{
    			listener.lookupTransform(user_to_track, oss.str(), ros::Time(0), transform);

    			transforms.push_back(transform);
    		}
    		catch(tf::TransformException &ex)
    		{
    			continue;
    		}
    	}

    	double min = std::numeric_limits<double>::max();
    	std::string index;

    	for(int i = 0; i < transforms.size(); i++)
    	{
    		double current = std::sqrt(std::pow(transforms[i].getOrigin().getX(), 2) + std::pow(transforms[i].getOrigin().getY(), 2) + std::pow(transforms[i].getOrigin().getZ(), 2));

    		if(current < min)
    		{
    			min = current;
    			index = transforms[i].child_frame_id_;
    		}
    	}

    	if(!transforms.empty())
    	{
    		std::cout << "Scheletro più vicino: " << index << std::endl;
    		//TODO parsing di index per trackare il torso giusto
    	}
    }
}

void userToTrackIdentityCallback(std_msgs::String msg)
{
	user_to_track = msg.data;
	got_user = true;
}

void userToTrackFaceCallback(cob_perception_msgs::DetectionArray msg)
{
	std::vector<cob_perception_msgs::Detection> identities = msg.detections;

	for(int i = 0; i < identities.size(); i++)
	{
		if(identities[i].detector == "face" && identities[i].label != "Unknown")
		{
			faces_counter[identities[i].label]++;
		}
	}

	for(std::map<std::string, int>::iterator iter = faces_counter.begin(); iter != faces_counter.end(); ++iter)
	{
		if(iter->second >= 100)
		{
			user_to_track = iter->first;
			got_user = true;
		}
	}
}

void faceCallback(cob_perception_msgs::DetectionArray msg)
{
	static tf::TransformBroadcaster br;
	static tf::TransformListener listener;

	std::vector<cob_perception_msgs::Detection> identities = msg.detections;
	ros::Time rcv = msg.header.stamp;
	ros::Time now = ros::Time::now();

	std::vector<tf::StampedTransform> transforms;

	for(int i = 0; i < identities.size(); i++)
	{
		if(identities[i].detector == "face" && identities[i].label == user_to_track)
		{
			tf::StampedTransform transform;

			transform.setOrigin(tf::Vector3(identities[i].pose.pose.position.x, identities[i].pose.pose.position.y, identities[i].pose.pose.position.z));
			transform.setRotation(tf::Quaternion(identities[i].pose.pose.orientation.x, identities[i].pose.pose.orientation.y, identities[i].pose.pose.orientation.z, identities[i].pose.pose.orientation.w));

			transform.frame_id_ = identities[i].header.frame_id;
			transform.child_frame_id_ = user_to_track;
			transform.stamp_ = now;

			transforms.push_back(transform);

			for(int j = 1; j <= 15; j++)
			{
				std::ostringstream oss;
				oss << "head_" << j;

				try
				{
					listener.lookupTransform(frame_id, oss.str(), rcv, transform);

					oss.str("");
					oss << "recognition_head_" << j;

					transform.child_frame_id_ = oss.str();
					transform.stamp_ = now;

					transforms.push_back(transform);
				}
				catch(tf::TransformException &ex)
				{
					continue;
				}
			}

			for(int j = 0; j < transforms.size(); j++)
			{
				br.sendTransform(transforms[j]);
			}
		}
	}

	return;
}
