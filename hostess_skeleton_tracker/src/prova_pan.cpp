#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <cyton_wrapper/dynamixelIO_wrapper_base.h>
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv)
{
	ros::init(argc,argv,"prova_pan");

	ros::NodeHandle nh;

	pluginlib::ClassLoader<dynamixelIO_wrapper_base::DynamixelIO_Base> dxio_loader("cyton_wrapper","dynamixelIO_wrapper_base::DynamixelIO_Base");

	boost::shared_ptr<dynamixelIO_wrapper_base::DynamixelIO_Base> dxio;
      
	try
	{
		dxio = dxio_loader.createInstance("dynamixelIO_wrapper_plugin::DynamixelIO");

		ROS_INFO("Loading PlugIn IO...");

		dxio->initialize(4, 34, 20, "/home/rob/virgil_ws/src/hostess_robot/hostess_skeleton_tracker/launch/include/pan_joint.yaml", &nh);

		dxio->start();
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
	}
   
	int16_t pan_inc = 15, pan_home_pos = 2048; //incrementa di  4.4 gradi
	int16_t pan_pos;
	bool exit_ = false;

	system("stty raw");//setting the terminal in raw mode
	
	char i = 'l';
	std::vector<std::vector<double> > v;
	std::vector<double> pv;
	std::vector<std::vector<int16_t> > v_int;
	std::vector<int16_t> pv_int;

	//Set both motor to zero

	pv.clear();
	pv.push_back(0.0);
	pv.push_back(0.6);
	v.push_back(pv);

	dxio->setMultiPosVel(v);

	v.clear();
	pan_pos = pan_home_pos;

	uint16_t minA_p;
	dxio->getCWAngleLimit(0, minA_p);

	uint16_t maxA_p;
	dxio->getCCWAngleLimit(0, maxA_p);

	while(!exit_ && ros::ok())
	{
		i = getchar();

		switch(i)
		{
			case 'q':
			{
				system("stty cooked");
				exit_ = true;

				break;
			}

			case 'd':
			{
				//pan a destra
				pv_int.clear();

				if((pan_pos - pan_inc) >= minA_p && (pan_pos - pan_inc) <= maxA_p)
				{
					pan_pos = pan_pos - pan_inc;
				}
				else if((pan_pos - pan_inc) <= minA_p)
				{
					pan_pos = minA_p;
				}
				else
				{
					pan_pos = maxA_p;
				}

				//std::cout<<"pan_pos: "<<pan_pos<<"\n";
				pv_int.push_back(pan_pos);
				pv_int.push_back(55);
				v_int.push_back(pv_int);

				dxio->setMultiPosVel(v_int);
				v_int.clear();

				break;
			}

			case 'a':
			{
				//pan a sinistra
				pv_int.clear();
				pan_pos = pan_pos + pan_inc;

				if((pan_pos + pan_inc) >= minA_p && (pan_pos + pan_inc) <= maxA_p)
				{
					pan_pos = pan_pos + pan_inc;
				}
				else if((pan_pos + pan_inc) <= minA_p)
				{
					pan_pos = minA_p;
				}
				else
				{
					pan_pos = maxA_p;
				}

				//	std::cout<<"pan_pos: "<<pan_pos<<"\n";
				pv_int.push_back(pan_pos);
				pv_int.push_back(55);
				v_int.push_back(pv_int);

				dxio->setMultiPosVel(v_int);
				v_int.clear();

				break;
			}

			case 's':
			{
				//Set both motor to zero
				pv.clear();
				pv.push_back(0.0);
				pv.push_back(0.6);
				v.push_back(pv);

				dxio->setMultiPosVel(v);
				v.clear();

				pan_pos = pan_home_pos;

				break;
			}

			default:
			{
				break;
			}
		}
	}

	dxio->stop();

	return EXIT_SUCCESS;
}


