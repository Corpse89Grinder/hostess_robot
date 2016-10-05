#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <limits>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/String.h>
#include "pan_controller.hpp"

//Maximum distance from skeleton head and face recognition points in space
#define DISTANCE_THRESHOLD 0.1
#define MINIMUM_ASSOCIATIONS_FOR_TRACKING 1
#define MAX_MEAN 5

#define PI 3.14159265358979323846

void lookForEveryHeadTransform(tf::TransformListener&, std::vector<tf::StampedTransform>&, std::string);
bool findClosestHeadToFace(std::vector<tf::StampedTransform>&, std::string&);
std::string lookForSpecificBodyTransform(tf::TransformListener&, std::string, std::string, tf::StampedTransform&);
int changeFrameAndReturnIndex(std::string&);
void twistCallback(geometry_msgs::Twist);
void goalCallback(actionlib_msgs::GoalStatusArray);
void resetLoop();

geometry_msgs::Twist newTwist;

std::map<std::string, std::pair<ros::Time, int> > skeletons;
std::map<std::string, ros::Time> last_stamp;
std::map<std::string, std::pair<ros::Time, int> > goals_status;
std::map<std::string, int> users_log;

std::vector<double> association_distances;

double ratio;
int skeleton_to_track = 0;

std::string current_goal_id = "";

ros::Publisher pub, cancel, logger;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dynamixel_mediator");

	ros::NodeHandle nh;
	std::string user_to_track;
	std::string frame_id;

	ros::param::set("skeleton_to_track", skeleton_to_track);

	std::string direction = "still";

	PanController pan_controller(nh);

	ROS_INFO("Waiting for reference frame.");

	while(!ros::param::get("camera_frame_id", frame_id) && nh.ok())
	{
		ros::Duration(1).sleep();
	}

	std::string topic_to_subscribe("/kobra/tracker_cmd_vel");
    nh.getParam("topic_to_subscribe", topic_to_subscribe);
    std::string topic_to_advertise("/kobra/cmd_vel");
    nh.getParam("topic_to_advertise", topic_to_advertise);

	ros::Subscriber twistSubscriber = nh.subscribe(topic_to_subscribe, 1, twistCallback);
    pub = nh.advertise<geometry_msgs::Twist>(topic_to_advertise, 1);

    ros::Subscriber goalSubscriber = nh.subscribe("/move_base/status", 1, goalCallback);

    cancel = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
    logger = nh.advertise<std_msgs::String>("logger", 10);

    tf::TransformListener listener;
    tf::TransformBroadcaster broadcaster;

    std::deque<double> speed_to_rotate_left(MAX_MEAN, 0);
	std::deque<double> speed_to_rotate_right(MAX_MEAN, 0);

	while(nh.ok())
	{
		ROS_INFO("Waiting for user identity.");

		while(!ros::param::get("user_to_track", user_to_track) && nh.ok())
		{
			ros::Duration(1).sleep();
		}

		user_to_track = user_to_track.substr(1, user_to_track.size());

		ROS_INFO("User identity received.");

		ROS_INFO("Waiting for goal.");

		while(current_goal_id == "")
		{
			ros::spinOnce();
			ros::Duration(1).sleep();
		}

		ROS_INFO("Goal received.");

		std_msgs::String msg;

		//Log identity, goal id and time
		msg.data = "start";
		logger.publish(msg);

		msg.data = user_to_track;
		logger.publish(msg);

		msg.data = current_goal_id;
		logger.publish(msg);

		std::ostringstream sstream;

		sstream << goals_status[current_goal_id].first.sec << "." << goals_status[current_goal_id].first.nsec;
		msg.data = sstream.str();
		logger.publish(msg);

		association_distances.clear();

		while(goals_status[current_goal_id].second != 3 && nh.ok())
		{
			ROS_INFO("Looking for %s's face.", user_to_track.c_str());

			std::string skeleton_to_track_frame;

			ros::Time reset = ros::Time::now();
			ros::Time abort = ros::Time::now();

			bool restart = false;

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

				ratio = std::max(0.0, ratio - 0.005);

				ros::spinOnce();

				if((ros::Time::now() - reset).sec >= 30 && !pan_controller.isHome())
				{
					ROS_INFO("Resetting camera orientation.");
					pan_controller.goHome();
				}

				if((ros::Time::now() - abort).sec >= 120)
				{
					ROS_INFO("Timout exceeded, restarting.");
					restart = true;

					break;
				}

				ros::Rate(30).sleep();
			}

			if(restart)
			{
				resetLoop();

				break;
			}

			ROS_INFO("User %s and skeleton %s associated. Start tracking.", user_to_track.c_str(), skeleton_to_track_frame.c_str());

			while(ros::param::get("skeleton_to_track", skeleton_to_track) && nh.ok())
			{
				if(skeleton_to_track == 0)
				{
					ROS_INFO("User %s and skeleton %s association lost. Stop tracking.", user_to_track.c_str(), skeleton_to_track_frame.c_str());
					pan_controller.standStill();

					break;
				}

				tf::StampedTransform transform;

				std::string returnString = lookForSpecificBodyTransform(listener, frame_id, skeleton_to_track_frame, transform);

				if(returnString == "found")
				{
					//Ho la distanza, in base ad essa restituisco la percentuale di velocità del robot.
					double distance = std::sqrt(std::pow(transform.getOrigin().getX(), 2) + std::pow(transform.getOrigin().getY(), 2));

					double alphaRAD = asin(transform.getOrigin().getY() / distance);

					pan_controller.turn(alphaRAD, newTwist.angular.z);

					if(skeleton_to_track != -1)
					{
						if(distance >= 0 && distance <= 1.5)
						{
							ratio = 1;
						}
						else if(distance > 1.5)
						{
							ratio = std::max(1 - (distance - 1.5), 0.0);
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
					else
					{
						ratio = std::max(0.0, ratio - 0.005);
					}
				}
				else if(returnString == "not found")
				{
					ratio = std::max(0.0, ratio - 0.005);
					pan_controller.standStill();
				}
				else if(returnString == "skip")
				{
					pan_controller.continueTurning();
				}

				ros::spinOnce();

				ros::Rate(30).sleep();
			}
		}

		if(goals_status[current_goal_id].second == 3)
		{
			ROS_INFO("Goal reached correctly, restarting.");

			msg.data = "Association distances:";
			logger.publish(msg);

			for(int i = 0; i < association_distances.size(); i++)
			{
				sstream.clear();
				sstream.str(std::string());
				sstream << association_distances[i];
				msg.data = sstream.str();
				logger.publish(msg);
			}

			resetLoop();

			msg.data = "succeeded";
			logger.publish(msg);
		}

		pan_controller.goHome();

		//Mando il robot alla posizione iniziale
	}

	ROS_INFO("Shutting down.");

    ros::shutdown();

    exit(EXIT_SUCCESS);
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

			association_distances.push_back(min);
			return true;
		}
	}

	return false;
}

std::string lookForSpecificBodyTransform(tf::TransformListener& listener, std::string frame_id, std::string body_to_track_frame, tf::StampedTransform& transform)
{
	try
	{
		if(skeleton_to_track == -1)
		{
			listener.lookupTransform(frame_id, "torso_k", ros::Time(0), transform);
		}
		else
		{
			listener.lookupTransform(frame_id, body_to_track_frame, ros::Time(0), transform);
		}

		if(transform.stamp_ != last_stamp[body_to_track_frame])
		{
			last_stamp[body_to_track_frame] = transform.stamp_;

			return "found";
		}
		else
		{
			return "skip";
		}
	}
	catch(tf::TransformException &ex)
	{
		return "not found";
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
	newTwist.linear.x = oldTwist.linear.x * ratio;
	newTwist.angular.z = oldTwist.angular.z * ratio;

	pub.publish(newTwist);

	return;
}

void goalCallback(actionlib_msgs::GoalStatusArray goals)
{
	for(int i = 0; i < goals.status_list.size(); i++)
	{
		if(goals_status.count(goals.status_list[i].goal_id.id) == 0)
		{
			//new goal received
			current_goal_id = goals.status_list[i].goal_id.id;

			goals_status[current_goal_id].first = goals.status_list[i].goal_id.stamp;
			goals_status[current_goal_id].second = goals.status_list[i].status;

			ROS_INFO("Current goal: %s", current_goal_id.c_str());
		}
		else
		{
			goals_status[goals.status_list[i].goal_id.id].second = goals.status_list[i].status;
		}
	}
}

void resetLoop()
{
	ROS_INFO("Reverting loop.");

	ros::param::del("user_to_track");

	ros::param::set("skeleton_to_track", -2);

	actionlib_msgs::GoalID msg;

	msg.id = current_goal_id;
	msg.stamp = goals_status[current_goal_id].first;

	cancel.publish(msg);

	current_goal_id = "";

	ros::Duration(5).sleep();
}
