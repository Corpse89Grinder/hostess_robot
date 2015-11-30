#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <geometry_msgs/Twist.h>

std::string frame_id("camera_depth_frame");
std::string child_frame_id("torso");

int main(int argc, char** argv)
{
	ros::init(argc, argv, "follow_me");

	ros::NodeHandle nh;

    tf::TransformListener listener;

    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/kobra/command_vel", 1);

	ros::Time last_stamp = ros::Time::now();

    while(nh.ok())
    {
    	geometry_msgs::Twist twist;

		twist.linear.x = twist.angular.z = 0;

		try
		{
			tf::StampedTransform transform;

			listener.lookupTransform(frame_id, child_frame_id, ros::Time(0), transform);

			if(transform.stamp_ != last_stamp)
			{
				last_stamp = transform.stamp_;

				double distance = std::sqrt(std::pow(transform.getOrigin().getX(), 2) + std::pow(transform.getOrigin().getY(), 2) + std::pow(transform.getOrigin().getZ(), 2));

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

				if(transform.getOrigin().getY() > 0.1)
				{
					//Giro a sinistra
					twist.angular.z = 0.5;
				}
				else if(transform.getOrigin().getY() < -0.1)
				{
					//Giro a destra
					twist.angular.z = -0.5;
				}
			}
		}
		catch(tf::TransformException &ex)
		{
			
		}

	   	pub.publish(twist);

	   	ros::Rate(30).sleep();
	}

	ros::shutdown();

	return EXIT_SUCCESS;
}
