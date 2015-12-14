#include "pan_controller.hpp"

#define SPEED 1.0

PanController::PanController(ros::NodeHandle& nh): private_nh_("~")
{
	private_nh_.param("DeviceIndex", mDeviceIndex, 1);
	private_nh_.param("Baudnum", mBaudnum, 34);
	private_nh_.param("UpdateRate", mUpdateRate, 20);
	std::string tempYamlPath("/home/rob/virgil_ws/src/hostess_robot/hostess_skeleton_tracker/launch/include/pan_joint.yaml");
	private_nh_.param("YamlPath", mYamlPath, tempYamlPath);

	//verifico se il plugin può essere caricato

	try
	{
		dxio_loader.reset(new pluginlib::ClassLoader<dynamixelIO_wrapper_base::DynamixelIO_Base>("cyton_wrapper","dynamixelIO_wrapper_base::DynamixelIO_Base"));
	}
	catch(pluginlib::PluginlibException& e)
	{
		ROS_ERROR("The plugin_loader failed to load for some reason. Error: %s", e.what());
	}

	//creo un'istanza del plugin, così dxio potrà accedere ai metodi della classe DynamixelIO_Base nel namespace dynamixelIO_wrapper_base

	try
	{
		dxio = dxio_loader->createInstance("dynamixelIO_wrapper_plugin::DynamixelIO");
		ROS_INFO("Loading PlugIn IO...");
		dxio->initialize(mDeviceIndex, mBaudnum, mUpdateRate, mYamlPath, &nh);

		dxio->start();
	}
	catch(pluginlib::PluginlibException& e)
	{
		ROS_ERROR("The plugin_loader failed to load for some reason. Error: %s", e.what());
	}

	extremeRight = (dxio->getMinAngle(0) - 2048) * 0.0015339804;	//Estrema destra del motore
	extremeLeft = (dxio->getMaxAngle(0) - 2048) * 0.0015339804;	//Estrema sinistra del motore
}

PanController::~PanController()
{
	dxio->stop();
}

void PanController::goHome()
{
	std::vector<std::vector<double> > v;
	std::vector<double> pv;

	pv.clear();
	pv.push_back(0.0);
	pv.push_back(SPEED);
	v.push_back(pv);

	dxio->setMultiPosVel(v);
}

void PanController::standStill()
{
	std::vector<std::vector<double> > v;
	std::vector<double> pv;

	pv.clear();
	pv.push_back(0.0);
	pv.push_back(0.0);
	v.push_back(pv);

	dxio->setMultiPosVel(v);
}

void PanController::turnLeft()
{
	std::vector<std::vector<double> > v;
	std::vector<double> pv;

	pv.clear();
	pv.push_back(extremeLeft);
	pv.push_back(SPEED);
	v.push_back(pv);

	dxio->setMultiPosVel(v);
}

void PanController::turnRight()
{
	std::vector<std::vector<double> > v;
	std::vector<double> pv;

	pv.clear();
	pv.push_back(extremeRight);
	pv.push_back(SPEED);
	v.push_back(pv);

	dxio->setMultiPosVel(v);
}
