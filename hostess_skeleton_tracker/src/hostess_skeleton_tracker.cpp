#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int64.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>
#include <string>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <map>
#include <opencv2/video/tracking.hpp>

#define MAX_USERS 6
#define DISTANCE_THRESHOLD 0.2

std::string genericUserCalibrationFileName;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

std::map<int, std::pair<ros::Time, ros::Duration> > users_timeouts;

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator&, XnUserID, void*);
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator&, XnUserID, void*);
void XN_CALLBACK_TYPE User_OutOfScene(xn::UserGenerator&, XnUserID, void*);
void XN_CALLBACK_TYPE User_BackIntoScene(xn::UserGenerator&, XnUserID, void*);

XnUInt32 calibrationData;

void publishTransform(tf::TransformBroadcaster&, XnUserID const&, XnSkeletonJoint const&, std::string const&, std::string const&);
void publishHeadTransforms(tf::TransformBroadcaster&, const std::string&);
bool publishTorsoTransform(tf::TransformBroadcaster&, const std::string&, int);
void publishAllTorsos(tf::TransformBroadcaster&, const std::string&);
bool checkCenterOfMass(XnUserID const&);
void stopTrackingAll(int);
void startTrackingAll();
void predictAndPublish(tf::TransformBroadcaster&);
void kalmanPrediction();
void kalmanUpdate(tf::TransformListener&, int);
int checkBestMatch(tf::TransformBroadcaster&, tf::TransformListener&);

//--------------Kalman Filter-----------------

const int stateSize = 6;
const int measSize = 3;
const int contrSize = 0;
cv::KalmanFilter kf(stateSize, measSize, contrSize);

cv::Mat state(stateSize, 1, CV_32F);
cv::Mat meas(measSize, 1, CV_32F);

double ticks = 0;
int notFoundCount = 0;
bool detected = false;
bool newMeas = false;

ros::Time lastStamp;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hostess_skeleton_tracker");
    ros::NodeHandle nh;

    std::string configFilename = ros::package::getPath("hostess_skeleton_tracker") + "/init/openni_tracker.xml";
    genericUserCalibrationFileName = ros::package::getPath("hostess_skeleton_tracker") + "/init/GenericUserCalibration.bin";

    XnStatus nRetVal;

    while(nh.ok())
    {
    	nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());

    	if(nRetVal != XN_STATUS_OK)
    	{
    		ROS_INFO("InitFromXml failed: %s Retrying in 3 seconds...", xnGetStatusString(nRetVal));
    		ros::Duration(3).sleep();
    	}
    	else
    	{
    		break;
    	}
    }

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);

    if(nRetVal != XN_STATUS_OK)
	{
		ROS_ERROR("Find depth generator failed: %s", xnGetStatusString(nRetVal));
	}

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);

	if (nRetVal != XN_STATUS_OK)
	{
		nRetVal = g_UserGenerator.Create(g_Context);

	    if (nRetVal != XN_STATUS_OK)
	    {
		    ROS_ERROR("NITE is likely missing: Please install NITE >= 1.5.2.21. Check the readme for download information. Error Info: User generator failed: %s", xnGetStatusString(nRetVal));
            return nRetVal;
	    }
	}

	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		ROS_INFO("Supplied user generator doesn't support skeleton");
		return EXIT_FAILURE;
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_UPPER);

    XnCallbackHandle hUserCallbacks;
    g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
    g_UserGenerator.RegisterToUserExit(User_OutOfScene, NULL, hUserCallbacks);
    g_UserGenerator.RegisterToUserReEnter(User_BackIntoScene, NULL, hUserCallbacks);

	nRetVal = g_Context.StartGeneratingAll();

	if(nRetVal != XN_STATUS_OK)
	{
		ROS_ERROR("StartGenerating failed: %s", xnGetStatusString(nRetVal));
	}

    std::string frame_id("camera_depth_frame");
    nh.getParam("camera_frame_id", frame_id);

    int skeleton_to_track = 0;
    nh.setParam("skeleton_to_track", skeleton_to_track);

    cv::setIdentity(kf.transitionMatrix);
    cv::setIdentity(kf.errorCovPre);

	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
	kf.measurementMatrix.at<float>(0) = 1.0f;
	kf.measurementMatrix.at<float>(7) = 1.0f;
	kf.measurementMatrix.at<float>(14) = 1.0f;

	cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));

	static tf::TransformListener listener;
	static tf::TransformBroadcaster br;

	ros::Duration timeout;

	while(nh.ok())
	{
		while(nh.getParam("skeleton_to_track", skeleton_to_track) && nh.ok())
		{
			if(skeleton_to_track != 0)	//0 means no skeleton to track, otherwise it represents the user id of the tracker
			{
				break;
			}

			g_Context.WaitAndUpdateAll();
			publishHeadTransforms(br, frame_id);

			ros::Rate(30).sleep();
		}

		while(nh.ok() && skeleton_to_track != 0)
		{
			stopTrackingAll(skeleton_to_track);

			while(nh.ok())
			{
				if(detected)
				{
					predictAndPublish(br);
				}

				g_Context.WaitAndUpdateAll();

				if(!publishTorsoTransform(br, frame_id, skeleton_to_track))
				{
					break;
				}
				else
				{
					kalmanUpdate(listener, skeleton_to_track);
					ros::Rate(30).sleep();
				}
			}

			startTrackingAll();

			ros::Time before = ros::Time::now();

			timeout.sec = timeout.nsec = 0;

			while(nh.ok())
			{
				g_Context.WaitAndUpdateAll();

				publishAllTorsos(br, frame_id);

				int index = checkBestMatch(br, listener);

				if(index != 0)
				{
					skeleton_to_track = index;
					break;
				}
				else
				{
					timeout += ros::Time::now() - before;
					before = ros::Time::now();

					if(timeout.sec > 2)
					{
						skeleton_to_track = 0;
						break;
					}
				}

				ros::Rate(30).sleep();
			}
		}

		ros::param::set("skeleton_to_track", skeleton_to_track);
	}

	g_Context.Shutdown();
	return EXIT_SUCCESS;
}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("New User %d.", nId);

	if(calibrationData == NULL)
	{
		g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(nId, genericUserCalibrationFileName.c_str());
		g_UserGenerator.GetSkeletonCap().SaveCalibrationData(nId, calibrationData);
	}
	else
	{
		g_UserGenerator.GetSkeletonCap().LoadCalibrationData(nId, calibrationData);
	}

	int skeleton_to_track;
	ros::param::get("skeleton_to_track", skeleton_to_track);

	if(skeleton_to_track == 0)
	{
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
		ROS_INFO("Start tracking user: %d.", nId);
	}
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("Lost user %d. Stop tracking.", nId);

	if(g_UserGenerator.GetSkeletonCap().IsTracking(nId))
	{
		g_UserGenerator.GetSkeletonCap().StopTracking(nId);
	}
}

void XN_CALLBACK_TYPE User_BackIntoScene(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("User %d back into scene. Restart tracking.", nId);

	g_UserGenerator.GetSkeletonCap().StartTracking(nId);
}

void XN_CALLBACK_TYPE User_OutOfScene(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("User %d out of scene. Stop tracking.", nId);

	if(g_UserGenerator.GetSkeletonCap().IsTracking(nId))
	{
		g_UserGenerator.GetSkeletonCap().StopTracking(nId);
	}
}

void publishTransform(tf::TransformBroadcaster& br, XnUserID const& user, XnSkeletonJoint const& joint, std::string const& frame_id, std::string const& child_frame_id)
{
    XnSkeletonJointPosition joint_position;
    XnSkeletonJointOrientation joint_orientation;

    if(g_UserGenerator.GetSkeletonCap().IsTracking(user))
    {
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);
    }

    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);

    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));

    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

void publishHeadTransforms(tf::TransformBroadcaster& br, const std::string& frame_id)
{
	XnUInt16 users_count = MAX_USERS;
    XnUserID users[MAX_USERS];

    g_UserGenerator.GetUsers(users, users_count);

    for(int i = 0; i < users_count; ++i)
    {
        XnUserID user = users[i];

        if(!checkCenterOfMass(user))
        {
            continue;
        }

        publishTransform(br, user, XN_SKEL_HEAD, frame_id, "head");
    }
}

bool publishTorsoTransform(tf::TransformBroadcaster& br, const std::string& frame_id, int skeleton_to_track)
{
    if(checkCenterOfMass(skeleton_to_track) && g_UserGenerator.GetSkeletonCap().IsTracking(skeleton_to_track))
	{
		publishTransform(br, skeleton_to_track, XN_SKEL_TORSO, frame_id, "torso");
		return true;
	}
	else
	{
		return false;
	}
}

void publishAllTorsos(tf::TransformBroadcaster& br, const std::string& frame_id)
{
	ROS_INFO("Publishing all torsos");

	XnUInt16 users_count = MAX_USERS;
	XnUserID users[MAX_USERS];

	g_UserGenerator.GetUsers(users, users_count);

	for(int i = 0; i < users_count; ++i)
	{
		XnUserID user = users[i];

		if(!checkCenterOfMass(user))
		{
			continue;
		}

		publishTransform(br, user, XN_SKEL_TORSO, frame_id, "torso");
	}

	ROS_INFO("Published all torsos");
}

void predictAndPublish(tf::TransformBroadcaster& br)
{
	kalmanPrediction();

	double x = state.at<float>(0);
	double y = state.at<float>(1);
	double z = state.at<float>(2);

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(x, y, z));
	transform.setRotation(tf::Quaternion(0, 0, 0, 1));

	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "torso_k"));
}

void kalmanPrediction()
{
	ROS_INFO("Kalman prediction started");

	double precTick = ticks;

	ticks = (double)cv::getTickCount();

	double dT = (ticks - precTick) / cv::getTickFrequency();

	kf.transitionMatrix.at<float>(2) = dT;
	kf.transitionMatrix.at<float>(9) = dT;
	kf.transitionMatrix.at<float>(16) = dT;

	state = kf.predict();

	ROS_INFO("Kalman prediction completed");
}

void kalmanUpdate(tf::TransformListener& listener, int skeleton_to_track)
{
	ROS_INFO("Kalman update started");

	tf::StampedTransform transform;

	std::ostringstream oss;
	oss << "torso_" << skeleton_to_track;

	try
	{
		listener.lookupTransform("map", oss.str(), ros::Time(0), transform);

		if(lastStamp == transform.stamp_)
		{
			return;
		}
		else
		{
			lastStamp = transform.stamp_;

			notFoundCount = 0;

			if(!detected)
			{
				detected = true;

				state.at<float>(0) = transform.getOrigin().getX();
				state.at<float>(1) = transform.getOrigin().getY();
				state.at<float>(2) = transform.getOrigin().getZ();
				state.at<float>(3) = 0;
				state.at<float>(4) = 0;
				state.at<float>(5) = 0;

				kf.statePost = state;
			}
			else
			{
				meas.at<float>(0) = transform.getOrigin().getX();
				meas.at<float>(1) = transform.getOrigin().getY();
				meas.at<float>(2) = transform.getOrigin().getZ();

				kf.correct(meas);
			}
		}
	}
	catch(tf::TransformException &ex)
	{

	}

	ROS_INFO("Kalman update completed");
}

int checkBestMatch(tf::TransformBroadcaster& br, tf::TransformListener& listener)
{
	predictAndPublish(br);

	double min = std::numeric_limits<double>::max();
	std::string frame_to_track;

	XnUInt16 users_count = MAX_USERS;
	XnUserID users[MAX_USERS];

	g_UserGenerator.GetUsers(users, users_count);

	for(int i = 0; i < users_count; ++i)
	{
		tf::StampedTransform transform;

		std::ostringstream oss;
		oss << "torso_" << i;

		try
		{
			listener.lookupTransform("map", oss.str(), ros::Time::now(), transform);

			double current = std::sqrt(std::pow(transform.getOrigin().getX(), 2) + std::pow(transform.getOrigin().getY(), 2) + std::pow(transform.getOrigin().getZ(), 2));

			if(current < min)
			{
				min = current;
				frame_to_track = transform.child_frame_id_;
			}
		}
		catch(tf::TransformException &ex)
		{

		}
	}

	if(min < DISTANCE_THRESHOLD)
	{
		//RIASSOCIAZIONE UTENTE

		return atoi(frame_to_track.substr(frame_to_track.find("_") + 1, frame_to_track.size()).c_str());
	}
	else
	{
		return 0;
	}
}

bool checkCenterOfMass(XnUserID const& user)
{
	XnPoint3D center_of_mass;
	XnStatus status = g_UserGenerator.GetCoM(user, center_of_mass);

	if(status != XN_STATUS_OK || (center_of_mass.X == 0 && center_of_mass.Y == 0 && center_of_mass.Z == 0))
    {
		if(users_timeouts[user].first.isZero())
		{
			users_timeouts[user].first = ros::Time::now();

			return true;
		}
		else
		{
			ros::Time now = ros::Time::now();

			users_timeouts[user].second += now - users_timeouts[user].first;
			users_timeouts[user].first = now;

			if(users_timeouts[user].second.sec >= 1)
			{
				return false;
			}
			else
			{
				return true;
			}
		}
    }
	else
	{
		if(!users_timeouts[user].first.isZero())
		{
			users_timeouts[user].first = ros::Time(0);
		}

		if(!users_timeouts[user].second.isZero())
		{
			users_timeouts[user].second = ros::Duration(0);
		}

		return true;
	}
}

void stopTrackingAll(int current_user)
{
	XnUInt16 users_count = MAX_USERS;
	XnUserID users[MAX_USERS];

	g_UserGenerator.GetUsers(users, users_count);

	for(int i = 0; i < users_count; ++i)
	{
		XnUserID user = users[i];

		if(user != current_user && g_UserGenerator.GetSkeletonCap().IsTracking(user))
		{
			g_UserGenerator.GetSkeletonCap().StopTracking(user);
		}
	}
}

void startTrackingAll()
{
	XnUInt16 users_count = MAX_USERS;
	XnUserID users[MAX_USERS];

	g_UserGenerator.GetUsers(users, users_count);

	for(int i = 0; i < users_count; ++i)
	{
		XnUserID user = users[i];

		g_UserGenerator.GetSkeletonCap().StartTracking(user);
	}
}
