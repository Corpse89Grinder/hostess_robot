#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <string>

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

#define MAX_USERS 1

std::string userCalibrationFileName;

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
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability&, XnUserID, void*);
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability&, XnUserID, XnBool, void*);
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability&, XnChar const*, XnUserID, void*);
void publishTransform(XnUserID const&, XnSkeletonJoint const&, std::string const&, std::string const&);
void publishTransforms(const std::string&);
void publishCenterOfMass(const std::string&);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openni_tracker");
    ros::NodeHandle nh;

    std::string configFilename = ros::package::getPath("hostess_skeleton_tracker") + "/init/openni_tracker.xml";
    userCalibrationFileName = ros::package::getPath("hostess_skeleton_tracker") + "/init/GenericUserCalibration.bin";
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

	XnCallbackHandle hCalibrationCallbacks;
	g_UserGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks);

	if (g_UserGenerator.GetSkeletonCap().NeedPoseForCalibration())
	{
		g_bNeedPose = TRUE;

		if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))
		{
			ROS_INFO("Pose required, but not supported");
			return 1;
		}

		XnCallbackHandle hPoseCallbacks;
		g_UserGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks);

		g_UserGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose);
	}

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");

	ros::Rate r(30);

    ros::NodeHandle pnh("~");
    std::string frame_id("camera_depth_frame");
    pnh.getParam("camera_frame_id", frame_id);

	while(nh.ok())
	{
		g_Context.WaitAndUpdateAll();

		//publishCenterOfMass(frame_id);
		publishTransforms(frame_id);

		r.sleep();
	}

	g_Context.Shutdown();
	return 0;
}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("New User %d", nId);

	if (g_bNeedPose)
	{
		g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
	}
	else
	{
		g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
	}
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("Lost user %d", nId);
	//g_UserGenerator.GetSkeletonCap().StopTracking(nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie)
{
	ROS_INFO("Calibration started for user %d", nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie)
{
	if (bSuccess)
	{
		ROS_INFO("Calibration complete, start tracking user %d", nId);
		g_UserGenerator.GetSkeletonCap().StartTracking(nId);
	}
	else
	{
		ROS_INFO("Calibration failed for user %d", nId);

		if (g_bNeedPose)
		{
			g_UserGenerator.GetPoseDetectionCap().StartPoseDetection(g_strPose, nId);
		}
		else
		{
			g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
		}
	}
}

void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, XnChar const* strPose, XnUserID nId, void* pCookie)
{
    ROS_INFO("Pose %s detected for user %d", strPose, nId);
    g_UserGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
    g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
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

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
}

void publishCenterOfMass(const std::string& frame_id)
{
	static tf::TransformBroadcaster br;
	ros::Time now = ros::Time::now();

	XnUInt16 users_count = MAX_USERS;
    XnUserID users[MAX_USERS];

    g_UserGenerator.GetUsers(users, users_count);

    for(int i = 0; i < users_count; ++i)
    {
        XnUserID user = users[i];

        XnPoint3D center_of_mass;
        XnStatus status = g_UserGenerator.GetCoM(user, center_of_mass);

        if(status == FALSE)
        {
        	continue;
        }

        std::ostringstream child_frame_id;
        child_frame_id << "center_of_mass_" << user;

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(center_of_mass.X, center_of_mass.Y, center_of_mass.Z));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));

        tf::Transform change_frame;
		change_frame.setOrigin(tf::Vector3(0, 0, 0));

		tf::Quaternion frame_rotation;
		frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
		change_frame.setRotation(frame_rotation);

		transform = change_frame * transform;

        br.sendTransform(tf::StampedTransform(transform, now, frame_id, child_frame_id.str()));
    }

    return;
}

void publishTransforms(const std::string& frame_id)
{
    ros::Time now = ros::Time::now();

    XnUInt16 users_count = MAX_USERS;
    XnUserID users[MAX_USERS];

    g_UserGenerator.GetUsers(users, users_count);

    for(int i = 0; i < users_count; ++i)
    {
        XnUserID user = users[i];

        if (!g_UserGenerator.GetSkeletonCap().IsTracking(user))
        {
            continue;
        }

        publishTransform(user, XN_SKEL_HEAD,           frame_id, "head");
//      publishTransform(user, XN_SKEL_NECK,           frame_id, "neck");
        publishTransform(user, XN_SKEL_TORSO,          frame_id, "torso");
//		publishTransform(user, XN_SKEL_LEFT_SHOULDER,  frame_id, "left_shoulder");
//		publishTransform(user, XN_SKEL_LEFT_ELBOW,     frame_id, "left_elbow");
//		publishTransform(user, XN_SKEL_LEFT_HAND,      frame_id, "left_hand");
//
//		publishTransform(user, XN_SKEL_RIGHT_SHOULDER, frame_id, "right_shoulder");
//		publishTransform(user, XN_SKEL_RIGHT_ELBOW,    frame_id, "right_elbow");
//		publishTransform(user, XN_SKEL_RIGHT_HAND,     frame_id, "right_hand");
//
//		publishTransform(user, XN_SKEL_LEFT_HIP,       frame_id, "left_hip");
//		publishTransform(user, XN_SKEL_LEFT_KNEE,      frame_id, "left_knee");
//		publishTransform(user, XN_SKEL_LEFT_FOOT,      frame_id, "left_foot");
//
//		publishTransform(user, XN_SKEL_RIGHT_HIP,      frame_id, "right_hip");
//		publishTransform(user, XN_SKEL_RIGHT_KNEE,     frame_id, "right_knee");
//		publishTransform(user, XN_SKEL_RIGHT_FOOT,     frame_id, "right_foot");
  }
}
