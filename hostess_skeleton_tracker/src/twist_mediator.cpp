#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <limits>

//Maximum distance from skeleton head and face recognition points in space
#define DISTANCE_THRESHOLD 0.15

void lookForEveryHeadTransform(tf::TransformListener&, std::vector<tf::StampedTransform>&, std::string);
bool findClosestHeadToFace(std::vector<tf::StampedTransform>&, std::string&);
bool lookForSpecificBodyTransform(tf::TransformListener&, std::string, std::string, tf::StampedTransform&, ros::Time&);
int retrieveIndexFromFrame(std::string);

int main(int argc, char** argv)
{
	ros::init(argc, argv, "twist_mediator");

	ros::NodeHandle nh;
	std::string user_to_track;

	std::string frame_id("camera_depth_frame");
	nh.getParam("camera_frame_id", frame_id);

	ROS_INFO("Waiting for user identity.");

    while(!ros::param::get("user_to_track", user_to_track) && nh.ok())
    {
    	ros::Duration(1).sleep();
    }

    tf::TransformListener listener;

    ros::Time last_stamp = ros::Time::now();
    int skeleton_to_track = 0;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/kobra/command_vel", 1);

    while(nh.ok())
    {
    	ROS_INFO("Looking for %s's face.", user_to_track.c_str());

    	std::string skeleton_to_track_frame;

		while(nh.ok())							//Search continuously for a skeleton head very close to the designated user face.
		{
			std::vector<tf::StampedTransform> transforms;

			lookForEveryHeadTransform(listener, transforms, user_to_track);

			if(findClosestHeadToFace(transforms, skeleton_to_track_frame))
			{
				skeleton_to_track = retrieveIndexFromFrame(skeleton_to_track_frame);
				ros::param::set("skeleton_to_track", skeleton_to_track);
				break;
			}

			geometry_msgs::Twist twist;
			twist.linear.x = twist.angular.z = 0;
			pub.publish(twist);

			ros::Rate(30).sleep();
		}

		ROS_INFO("User %s and skeleton %s associated. Start tracking.", user_to_track.c_str(), skeleton_to_track_frame.c_str());

		while(ros::param::get("skeleton_to_track", skeleton_to_track) && nh.ok())
		{
			if(skeleton_to_track == 0)
			{
				ROS_INFO("User %s and skeleton %s association lost. Stop tracking.", user_to_track.c_str(), skeleton_to_track_frame.c_str());
				break;
			}

			tf::StampedTransform transform;

			if(lookForSpecificBodyTransform(listener, frame_id, skeleton_to_track_frame, transform, last_stamp))
			{
				double distance = std::sqrt(std::pow(transform.getOrigin().getX(), 2) + std::pow(transform.getOrigin().getY(), 2) + std::pow(transform.getOrigin().getZ(), 2));

				//TODO Ho la distanza, in base ad essa restituisco la percentuale di velocità del robot.

				geometry_msgs::Twist twist;

				if(distance < 1.35)
				{
					//Mi allontano
					twist.linear.x = 0.7;
				}
				else if(distance > 1.65)
				{
					//Mi avvicino
					twist.linear.x = -0.7;
				}

				if(transform.getOrigin().getY() > 0.2)
				{
					//Giro a sinistra
					twist.angular.z = 0.5;
				}
				else if(transform.getOrigin().getY() < -0.2)
				{
					//Giro a destra
					twist.angular.z = -0.5;
				}

				pub.publish(twist);
			}
			else
			{
				//Non ho ancora la distanza, se la situazione perdura per molto c'è qualcosa che non va
			}

			ros::Rate(30).sleep();
		}
    }

    ros::shutdown();

    return EXIT_SUCCESS;
}

void lookForEveryHeadTransform(tf::TransformListener& listener, std::vector<tf::StampedTransform>& transforms, std::string user_to_track)
{
	for(int i = 1; i <= 15; i++)
	{
		std::ostringstream oss;
		oss << "head_" << i;

		try
		{
			tf::StampedTransform transform;

			//TODO sistemare i tempi, posso chiedere 2 tempi diversi per i 2 frames da comparare (ultima face e tf passate).
			listener.lookupTransform(user_to_track, oss.str(), ros::Time(0), transform);

			transforms.push_back(transform);
		}
		catch(tf::TransformException &ex)
		{
			continue;
		}
	}
}

bool findClosestHeadToFace(std::vector<tf::StampedTransform>& transforms, std::string& skeleton_to_track_frame)
{
	double min = std::numeric_limits<double>::max();

	for(int i = 0; i < transforms.size(); i++)
	{
		double current = std::sqrt(std::pow(transforms[i].getOrigin().getX(), 2) + std::pow(transforms[i].getOrigin().getY(), 2) + std::pow(transforms[i].getOrigin().getZ(), 2));

		if(current < min)
		{
			min = current;
			skeleton_to_track_frame = transforms[i].child_frame_id_;
		}
	}

	if(min < DISTANCE_THRESHOLD)	//Right skeleton found.
	{
		return true;
	}
	else
	{
		skeleton_to_track_frame.clear();
		return false;
	}
}

bool lookForSpecificBodyTransform(tf::TransformListener& listener, std::string frame_id, std::string body_to_track_frame, tf::StampedTransform& transform, ros::Time& last_stamp)
{
	try
	{
		listener.lookupTransform(frame_id, body_to_track_frame, ros::Time(0), transform);

		if(transform.stamp_ != last_stamp)
		{
			last_stamp = transform.stamp_;

			return true;
		}
		else
		{
			return false;
		}
	}
	catch(tf::TransformException &ex)
	{
		return false;
	}
}

int retrieveIndexFromFrame(std::string frame)
{
	return atoi(frame.substr(frame.rfind("_") + 1, frame.length()).c_str());
}
