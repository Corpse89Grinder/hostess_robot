#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int64.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <string>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <map>

#define MAX_USERS 15

std::string genericUserCalibrationFileName;

xn::Context        g_Context;
xn::DepthGenerator g_DepthGenerator;
xn::UserGenerator  g_UserGenerator;

XnBool g_bNeedPose   = FALSE;
XnChar g_strPose[20] = "";

#define CHECK_RC(nRetVal, what)											\
	if (nRetVal != XN_STATUS_OK)										\
	{																	\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));	\
		return nRetVal;													\
	}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator&, XnUserID, void*);
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator&, XnUserID, void*);
void publishTransform(XnUserID const&, XnSkeletonJoint const&, std::string const&, std::string const&);
void publishHeadTransforms(const std::string&);
void publishTorsoTransform(const std::string&, int&);
bool checkCenterOfMass(XnUserID const&);

std::map<int, std::pair<ros::Time, ros::Duration> > users_timeouts;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openni_tracker");
    ros::NodeHandle nh;

    std::string configFilename = ros::package::getPath("hostess_skeleton_tracker") + "/init/openni_tracker.xml";
    genericUserCalibrationFileName = ros::package::getPath("hostess_skeleton_tracker") + "/init/GenericUserCalibration.bin";
    XnStatus nRetVal = g_Context.InitFromXmlFile(configFilename.c_str());
    CHECK_RC(nRetVal, "InitFromXml");

    nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
    CHECK_RC(nRetVal, "Find depth generator");

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
		return 1;
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_UPPER);

    XnCallbackHandle hUserCallbacks;
	g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

    std::string frame_id("camera_depth_frame");
    nh.getParam("camera_frame_id", frame_id);

    int skeleton_to_track = 0;
    nh.setParam("skeleton_to_track", skeleton_to_track);

	while(nh.ok())
	{
		while(nh.getParam("skeleton_to_track", skeleton_to_track) && nh.ok())
		{
			if(skeleton_to_track != 0)	//0 means no skeleton to track, otherwise it represents the user id of the tracker
			{
				break;
			}

			g_Context.WaitAndUpdateAll();
			publishHeadTransforms(frame_id);

			ros::Rate(30).sleep();
		}

		while(nh.ok())
		{
			g_Context.WaitAndUpdateAll();
			publishTorsoTransform(frame_id, skeleton_to_track);

			ros::Rate(30).sleep();

			if(skeleton_to_track == 0)
			{
				ros::param::set("skeleton_to_track", skeleton_to_track);
				break;
			}
		}
	}

	g_Context.Shutdown();
	return EXIT_SUCCESS;
}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("New User %d", nId);

	g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(nId, genericUserCalibrationFileName.c_str());
	g_UserGenerator.GetSkeletonCap().StartTracking(nId);

	ROS_INFO("Start tracking user %d", nId);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("Lost user %d", nId);

	g_UserGenerator.GetSkeletonCap().StopTracking(nId);

	ROS_INFO("Stop tracking user %d", nId);
}

void publishTransform(XnUserID const& user, XnSkeletonJoint const& joint, std::string const& frame_id, std::string const& child_frame_id)
{
    static tf::TransformBroadcaster br;

    XnSkeletonJointPosition joint_position;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, joint, joint_position);

    double x = -joint_position.position.X / 1000.0;
    double y = joint_position.position.Y / 1000.0;
    double z = joint_position.position.Z / 1000.0;

    XnSkeletonJointOrientation joint_orientation;
    g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, joint, joint_orientation);

    XnFloat* m = joint_orientation.orientation.elements;
    KDL::Rotation rotation(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);

    double qx, qy, qz, qw;
    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));

    tf::Quaternion frame_rotation;
    frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

void publishHeadTransforms(const std::string& frame_id)
{
    ros::Time now = ros::Time::now();

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

        publishTransform(user, XN_SKEL_HEAD, frame_id, "head");
    }
}

void publishTorsoTransform(const std::string& frame_id, int& skeleton_to_track)
{
    ros::Time now = ros::Time::now();

	if(checkCenterOfMass(skeleton_to_track))
	{
		publishTransform(skeleton_to_track, XN_SKEL_TORSO, frame_id, "torso");
	}
	else
	{
		skeleton_to_track = 0;
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
			users_timeouts [user].first = now;

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
