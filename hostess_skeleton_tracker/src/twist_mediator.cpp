#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <limits>

//Maximum distance from skeleton head and face recognition points in space
#define DISTANCE_THRESHOLD 15

void lookForEveryHeadTransform(tf::TransformListener&, std::vector<tf::StampedTransform>&, std::string);
bool findClosestHeadToFace(std::vector<tf::StampedTransform>&, std::string);
bool lookForSpecificBodyTransform(tf::TransformListener&, std::string, tf::StampedTransform&);

std::string frame_id("camera_depth_optical_frame");

int main(int argc, char** argv)
{
	ros::init(argc, argv, "twist_mediator");

	ros::NodeHandle nh;
	std::string user_to_track;

    while(!ros::param::get("user_to_track", user_to_track))
    {
    	ros::Duration(1).sleep();
    }

    tf::TransformListener listener;

    while(nh.ok())
    {
    	std::string body_to_track_frame;

		while(true)							//Search continuously for a skeleton head very close to the designated user face.
		{
			std::vector<tf::StampedTransform> transforms;

			lookForEveryHeadTransform(listener, transforms, user_to_track);

			if(findClosestHeadToFace(transforms, body_to_track_frame))
			{
				break;
			}

			ros::Rate(30).sleep();
		}

		while(true)
		{
			tf::StampedTransform transform;

			if(lookForSpecificBodyTransform(listener, body_to_track_frame, transform))
			{
				double distance = std::sqrt(std::pow(transform.getOrigin().getX(), 2) + std::pow(transform.getOrigin().getY(), 2) + std::pow(transform.getOrigin().getZ(), 2));

				//TODO Ho la distanza, in base ad essa restituisco la percentuale di velocità del robot.
			}
			else
			{
				//TODO Esco al primo frame perso? O inserisco un conto? Da definire in base al comportamento del tracker.
			}

			ros::Rate(30).sleep();
		}
    }

    ros::shutdown();

    return EXIT_SUCCESS;
}

void lookForEveryHeadTransform(tf::TransformListener& listener, std::vector<tf::StampedTransform>& transforms, std::string user_to_track)
{
	tf::StampedTransform transform;

	for(int i = 1; i <= 15; i++)
	{
		std::ostringstream oss;
		oss << "head_" << i;

		try
		{
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

bool findClosestHeadToFace(std::vector<tf::StampedTransform>& transforms, std::string& body_to_track_frame)
{
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

	if(min < DISTANCE_THRESHOLD)	//Right skeleton found.
	{
		std::ostringstream oss;
		oss << "torso_" << index;
		body_to_track_frame = oss.str();
		return true;
	}
	else
	{
		return false;
	}
}

bool lookForSpecificBodyTransform(tf::TransformListener& listener, std::string body_to_track_frame, tf::StampedTransform& transform)
{
	try
	{
		listener.lookupTransform(frame_id, body_to_track_frame, ros::Time::now(), transform);
		return true;
	}
	catch(tf::TransformException &ex)
	{
		return false;
	}
}
