#include <ros/ros.h>
#include <geometry_msgs/Transform.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "try");
	ros::NodeHandle nh;

	geometry_msgs::Transform a, b;

	a.translation.x = 1;
	a.translation.y = a.translation.z = 0;
	a.rotation.x = a.rotation.y = 0;
	a.rotation.z = 0.5;
	a.rotation.w = 1;

	b.translation.x = 1;
	b.translation.y = b.translation.z = 0;
	b.rotation.x = b.rotation.y = b.rotation.z = 0;
	b.rotation.w = 1;

	tf::TransformBroadcaster br;

	while(nh.ok())
	{
		tf::Transform a_transform, b_transform, c_transform;
		a_transform.setOrigin(tf::Vector3(a.translation.x, a.translation.y, a.translation.z));
		a_transform.setRotation(tf::Quaternion(a.rotation.x, a.rotation.y, a.rotation.z, a.rotation.w));

		br.sendTransform(tf::StampedTransform(a_transform, ros::Time::now(), "map", "a"));

		b_transform.setOrigin(tf::Vector3(b.translation.x, b.translation.y, b.translation.z));
		b_transform.setRotation(tf::Quaternion(b.rotation.x, b.rotation.y, b.rotation.z, b.rotation.w));

		br.sendTransform(tf::StampedTransform(b_transform, ros::Time::now(), "a", "b"));

		c_transform = a_transform * b_transform;

		br.sendTransform(tf::StampedTransform(c_transform, ros::Time::now(), "map", "c"));

		ros::Rate(1).sleep();
	}
}
