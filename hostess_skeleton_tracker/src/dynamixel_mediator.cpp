#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <limits>
#include <geometry_msgs/Twist.h>
#include "pan_controller.hpp"

//Maximum distance from skeleton head and face recognition points in space
#define DISTANCE_THRESHOLD 0.1
#define MINIMUM_ASSOCIATIONS_FOR_TRACKING 1
#define MAX_MEAN 5

void lookForEveryHeadTransform(tf::TransformListener&, std::vector<tf::StampedTransform>&, std::string);
bool findClosestHeadToFace(std::vector<tf::StampedTransform>&, std::string&);
bool lookForSpecificBodyTransform(tf::TransformListener&, std::string, std::string, tf::StampedTransform&);
int changeFrameAndReturnIndex(std::string&);
void twistCallback(geometry_msgs::Twist);

std::map<std::string, std::pair<ros::Time, int> > skeletons;
std::map<std::string, ros::Time> last_stamp;

double ratio;

ros::Publisher pub;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dynamixel_mediator");

	ros::NodeHandle nh;
	std::string user_to_track;

	std::string frame_id("camera_depth_frame");
	nh.getParam("camera_frame_id", frame_id);

	PanController pan_controller(nh);

	while(!pan_controller.isHome())
	{
		ros::Duration(0.5).sleep();
	}

	ROS_INFO("Waiting for user identity.");

    while(!ros::param::get("user_to_track", user_to_track) && nh.ok())
    {
    	ros::Duration(1).sleep();
    }

    std::string topic_to_subscribe("/kobra/tracker_cmd_vel");
    nh.getParam("topic_to_subscribe", topic_to_subscribe);
    std::string topic_to_advertise("/kobra/cmd_vel");
	nh.getParam("topic_to_advertise", topic_to_advertise);

    ros::Subscriber twistSubscriber = nh.subscribe(topic_to_subscribe, 1, twistCallback);
    pub = nh.advertise<geometry_msgs::Twist>(topic_to_advertise, 1);

    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;

    int skeleton_to_track = 0;
    std::deque<double> speed_to_rotate(MAX_MEAN, 0);

    tf::Transform panTransform;
    panTransform.setOrigin(tf::Vector3(0, 0, 0.05));
    tf::Quaternion panOrientation;

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
				skeleton_to_track = changeFrameAndReturnIndex(skeleton_to_track_frame);
				ros::param::set("skeleton_to_track", skeleton_to_track);
				break;
			}

			pan_controller.standStill();

			panOrientation.setRPY(0, 0, pan_controller.getRotation());
			panTransform.setRotation(panOrientation);
			broadcaster.sendTransform(tf::StampedTransform(panTransform, ros::Time::now(), "virgil_top_link", "pan_link"));

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

			if(lookForSpecificBodyTransform(listener, frame_id, skeleton_to_track_frame, transform))
			{
				//TODO Ho la distanza, in base ad essa restituisco la percentuale di velocità del robot.
				double distance = std::sqrt(std::pow(transform.getOrigin().getX(), 2) + std::pow(transform.getOrigin().getY(), 2));

				if(transform.getOrigin().getY() > 0.1)
				{
					//Giro a sinistra
					speed_to_rotate.pop_front();
					speed_to_rotate.push_back(2 * acos(transform.getOrigin().getX() / distance));

					double speed = 0;

					for(int i = 0; i < MAX_MEAN; i++)
					{
						speed += speed_to_rotate[i] / MAX_MEAN;
					}

					pan_controller.turnLeft(speed);
				}
				else if(transform.getOrigin().getY() < -0.1)
				{
					//Giro a destra
					speed_to_rotate.pop_front();
					speed_to_rotate.push_back(2 * acos(transform.getOrigin().getX() / distance));

					double speed = 0;

					for(int i = 0; i < MAX_MEAN; i++)
					{
						speed += speed_to_rotate[i] / MAX_MEAN;
					}

					pan_controller.turnRight(speed);
				}
				else
				{
					speed_to_rotate.pop_front();
					speed_to_rotate.push_back(0);
					pan_controller.standStill();
				}



				if(distance >= 0 && distance <= 1.5)
				{
					ratio = 1;
				}
				else if(distance > 1.5 && distance <= 2.5)
				{
					ratio = 1 - (distance - 1.5);
				}
				else if(distance > 2.5)
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
			}

			ros::spinOnce();

			panOrientation.setRPY(0, 0, pan_controller.getRotation());
			panTransform.setRotation(panOrientation);
			broadcaster.sendTransform(tf::StampedTransform(panTransform, ros::Time::now(), "virgil_top_link", "pan_link"));

			ros::Rate(30).sleep();
		}
    }

    ros::shutdown();

    return EXIT_SUCCESS;
}

void lookForEveryHeadTransform(tf::TransformListener& listener, std::vector<tf::StampedTransform>& transforms, std::string user_to_track)
{
	for(int i = 1; i <= 6; i++)
	{
		std::ostringstream oss;
		oss << "head_" << i;

		try
		{
			tf::StampedTransform transform;

			//TODO sistemare i tempi, posso chiedere 2 tempi diversi per i 2 frames da comparare (ultima face e tf passate).
			listener.lookupTransform(user_to_track, oss.str(), ros::Time(0), transform);

			if(transform.stamp_ != last_stamp[oss.str()])
			{
				last_stamp[oss.str()] = transform.stamp_;
				transforms.push_back(transform);
			}
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
	std::string frame_to_track;

	for(int i = 0; i < transforms.size(); i++)
	{
		double current = std::sqrt(std::pow(transforms[i].getOrigin().getX(), 2) + std::pow(transforms[i].getOrigin().getY(), 2) + std::pow(transforms[i].getOrigin().getZ(), 2));

		if(current < min)
		{
			min = current;
			frame_to_track = transforms[i].child_frame_id_;
		}
	}

	ros::Time now = ros::Time::now();

	for(auto iterator = skeletons.begin(); iterator != skeletons.end();)
	{
		if((now - iterator->second.first).nsec >= 5e8)				//More than half a second passed from the previous finding, I delete the entry
		{
			iterator = skeletons.erase(iterator);
		}
		else
		{
			++iterator;
		}
	}

	if(min < DISTANCE_THRESHOLD)									//Right skeleton found.
	{
		skeletons[frame_to_track].first = now;
		skeletons[frame_to_track].second++;

		if(skeletons[frame_to_track].second >= MINIMUM_ASSOCIATIONS_FOR_TRACKING)
		{
			skeleton_to_track_frame = frame_to_track;
			skeletons.clear();
			return true;
		}
	}

	return false;
}

bool lookForSpecificBodyTransform(tf::TransformListener& listener, std::string frame_id, std::string body_to_track_frame, tf::StampedTransform& transform)
{
	try
	{
		listener.lookupTransform(frame_id, body_to_track_frame, ros::Time(0), transform);

		if(transform.stamp_ != last_stamp[body_to_track_frame])
		{
			last_stamp[body_to_track_frame] = transform.stamp_;

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

int changeFrameAndReturnIndex(std::string& frame)
{
	int index = atoi(frame.substr(frame.rfind("_") + 1, frame.length()).c_str());

	std::ostringstream oss;
	oss << "torso_" << index;
	frame = oss.str();

	return index;
}

void twistCallback(geometry_msgs::Twist oldTwist)
{
	geometry_msgs::Twist newTwist;

	newTwist.linear.x = oldTwist.linear.x * ratio;
	newTwist.angular.z = oldTwist.angular.z * ratio;

	pub.publish(newTwist);
	ROS_WARN("Sparo twist");
	return;
}
