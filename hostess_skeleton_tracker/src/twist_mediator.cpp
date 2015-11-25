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
#include <string>

void userToTrackIdentityCallback(std_msgs::String);
void userToTrackFaceCallback(cob_perception_msgs::DetectionArray);

std::string user_to_track = "Unknown_1";
bool got_user = true;
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

    while(nh.ok())
    {
    	tf::StampedTransform transform;

    	std::vector<tf::StampedTransform> transforms;

    	for(int i = 1; i <= 15; i++)
    	{
    		std::ostringstream oss;
    		oss << "head_" << i;

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
    		std::cout << "Scheletro più vicino: " << index << ", distanza: " << min << std::endl;
    		//TODO parsing di index per trackare il torso giusto
    	}

    	rate.sleep();
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
