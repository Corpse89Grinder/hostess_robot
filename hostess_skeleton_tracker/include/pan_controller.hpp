#ifndef PAN_CONTROLLER_HPP_
#define PAN_CONTROLLER_HPP_

#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <cyton_wrapper/dynamixelIO_wrapper_base.h>
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>

class PanController
{
	private:
		ros::NodeHandle private_nh_;

		boost::shared_ptr<pluginlib::ClassLoader<dynamixelIO_wrapper_base::DynamixelIO_Base> > dxio_loader;

		boost::shared_ptr<dynamixelIO_wrapper_base::DynamixelIO_Base> dxio;

		boost::mutex mutex;

		int mDeviceIndex, mBaudnum, mUpdateRate;
		std::string mYamlPath;

		double extremeLeft, extremeRight;

		double turningSpeed, lambda, targetPosition, presentPosition;

		void turnRight(double);
		void turnLeft(double);
		void broadcastRotation();

	public:
		PanController(ros::NodeHandle&);
		~PanController();
		bool isHome();
		void goHome();
		void standStill();
		void turn(double, double);
		void continueTurning();
};

#endif /* PAN_CONTROLLER_HPP_ */
