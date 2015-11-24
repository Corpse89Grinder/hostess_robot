#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "prova");
	ros::NodeHandle nh;

	ros::Rate rate(1.0);

	tf::TransformListener listener;
	tf::TransformBroadcaster broadcaster;

	while(nh.ok())
	{
		rate.sleep();

		tf::StampedTransform transform;

		try
		{
			listener.lookupTransform("Giuseppe", "Antonio", ros::Time(0), transform);

			transform.child_frame_id_ = "prova";

			broadcaster.sendTransform(transform);
		}
		catch(tf::TransformException exc)
		{
			continue;
		}
	}

	return EXIT_SUCCESS;
}
