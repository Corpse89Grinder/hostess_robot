#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>
#include <sstream>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>
#include <map>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/video/tracking.hpp>
#include <vector>
#include <std_msgs/String.h>
#include <string>

#define MAX_USERS 15							//Grandezza massima del vettore che contiene gli utenti, definito così da OpenNI
#define DISTANCE_THRESHOLD 1					//Raggio massimo in cui cerco un nuovo scheletro nel caso di riassociazione rapida con Kalman
#define LIKENESS_THRESHOLD 0.75					//Soglia minima di correlazione per riassociazione rapida
#define KALMAN_TIMEOUT 5						//Tempo massimo per riassociazione rapida con Kalman e correlazione

std::string frame_id;							//Il frame TF relativo alla camera, viene letto mediante parametro
std::string parent_frame_id("map");				//Il frame TF base dello spazio in cui opera il robot
int skeleton_to_track = 0;						//skeleton_to_track rappresenta l'ID dello scheletro dell'utente che sto trackando, viene utilizzato come rosparam per sincronizzarsi con il nodo mediatore di velocità e gestore della camera

//-------------------------Variabili e callback OpenNI e NiTE------------------------------
xn::Context				g_Context;
xn::DepthGenerator		g_DepthGenerator;
xn::UserGenerator		g_UserGenerator;
xn::SceneAnalyzer		g_SceneAnalyzer;
xn::ImageGenerator		g_ImageGenerator;

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator&, XnUserID, void*);
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator&, XnUserID, void*);
void XN_CALLBACK_TYPE User_OutOfScene(xn::UserGenerator&, XnUserID, void*);
void XN_CALLBACK_TYPE User_BackIntoScene(xn::UserGenerator&, XnUserID, void*);

std::string genericUserCalibrationFileName;		//Stringa per il file della calibrazione generica dello scheletro, serve per saltare la posa di avvio del tracking
XnUInt32 calibrationData;						//Calibrazione dello scheletro, caricata una sola volta in runtime in memoria, poi riutilizzata per gli altri utenti
//-----------------------------------------------------------------------------------------

std::map<int, double> distances;								//Hash table utilizzata per tenere traccia delle distanze dei vari scheletri dalla stima del filtro di Kalman durante la fase di riassociazione rapida

std::map<int, image_transport::Publisher> image_publishers;		//Hash table contenente i publisher delle immagini delle sagome degli utenti nel campo di vista della camera

cv::Mat HSVImage;												//Immagine OpenCV della camera RGB, continuamente aggiornata e convertita nel profilo HSV per effettuare la correlazione dell'istogramma

//------------------------------Funzioni per il calcolo e la pubblicazione delle coordinate con TF-----------------------------
bool calcUserTransforms(XnUserID const&, tf::Transform&, tf::Transform&, tf::Transform&, tf::Transform&);
void publishUserTransforms(int, tf::Transform, tf::Transform, tf::Transform, tf::Transform, ros::Time);
void publishAllTransforms();

void updateHSVImage(const sensor_msgs::ImageConstPtr&);
void calcUserHistogram(int, cv::Mat&, bool, image_transport::ImageTransport&);
double histogramComparison(cv::Mat);

bool checkCenterOfMass(XnUserID const&);

cv::Mat userHistogram;

tf::StampedTransform parentTransform;
bool updateParent();

//--------------Inizializzazione e funzioni filtro di Kalman-----------------
const int stateSize = 6;
const int measSize = 3;
const int contrSize = 0;

cv::KalmanFilter kf1(stateSize, measSize, contrSize);
cv::KalmanFilter kf2(stateSize, measSize, contrSize);

cv::Mat state(stateSize, 1, CV_32F);
cv::Mat meas(measSize, 1, CV_32F);

cv::Point oldPosition;

double ticks = 0;
double dT;

int notFoundCount = 0;

void kalmanInitialization();
void predictAndPublish();
void kalmanPrediction();
void kalmanUpdate(tf::Transform);
//--------------------------------------------------------------------------

bool detected = false;

std::vector<std::pair<double, double> > reassociations;

ros::Publisher logger;

int main(int argc, char **argv)
{
	//-------------------------------------Inizializzazione ROS--------------------------------------------
	ros::init(argc, argv, "hostess_skeleton_tracker");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, updateHSVImage);

	std::string configFilename = ros::package::getPath("hostess_skeleton_tracker") + "/init/openni_tracker.xml";
	genericUserCalibrationFileName = ros::package::getPath("hostess_skeleton_tracker") + "/init/GenericUserCalibration.bin";

	logger = nh.advertise<std_msgs::String>("logger", 10);
	//-----------------------------------------------------------------------------------------------------

	//--------------------------------Inizializzazione OpenNI e NiTE---------------------------------------
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

	frame_id = "camera_depth_frame";

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);

	if(nRetVal != XN_STATUS_OK)
	{
		nRetVal = g_ImageGenerator.Create(g_Context);
	}

	if(nRetVal != XN_STATUS_OK)
	{
		ROS_ERROR("Find image generator failed: %s", xnGetStatusString(nRetVal));
	}

	if(g_DepthGenerator.IsCapabilitySupported(XN_CAPABILITY_ALTERNATIVE_VIEW_POINT))
	{
		g_DepthGenerator.GetAlternativeViewPointCap().SetViewPoint(g_ImageGenerator);
		frame_id = "camera_rgb_frame";
	}

	nh.setParam("camera_frame_id", frame_id);

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_SCENE, g_SceneAnalyzer);

	if(nRetVal != XN_STATUS_OK)
	{
		nRetVal = g_SceneAnalyzer.Create(g_Context);
	}

	if(nRetVal != XN_STATUS_OK)
	{
		ROS_ERROR("Find scene analyzer failed: %s", xnGetStatusString(nRetVal));
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

	//-----------------------------------------------------------------------------------------------------

	nh.getParam("camera_frame_id", frame_id);					//Frame TF definitivo della camera di riferimento (RGB o Depth)
	nh.getParam("parent_frame_id", parent_frame_id);			//Frame TF definitivo del punto di origine (map)

	//-----------------------------------Listener e Broadcaster TF-----------------------------------------
	static tf::TransformListener listener;
	static tf::TransformBroadcaster br;
	//-----------------------------------------------------------------------------------------------------

	kalmanInitialization();					//Funzione di inizializzazione del filtro di Kalman

	ros::Time lastDetected;					//Timestamp che contiene l'ultimo istante in cui ho visto lo scheletro principale prima di perderlo

	while(nh.getParam("skeleton_to_track", skeleton_to_track) && nh.ok())			//Ciclo principale
	{
		ros::spinOnce();					//Aggiorno ROS e chiamo le callback
		updateParent();						//Aggiorno la posizione della camera relativa all'origine, serve per poter calcolare le coordinate degli scheletri in maniera globale

		ros::Time now = ros::Time::now();	//Timestamp di questo istante

		//---------------------------Aggiorno il dT (delta T) dall'ultima iterazione del ciclo, serve per il filtro di Kalman----------------------
		double precTick = ticks;
		ticks = (double)cv::getTickCount();
		dT = (ticks - precTick) / cv::getTickFrequency();
		//-----------------------------------------------------------------------------------------------------------------------------------------

		g_Context.WaitAndUpdateAll();		//Funzione di aggiornamento di OpenNI e NiTE, ferma il ciclo e chiama le callback necessarie

		if(skeleton_to_track == 0)
		{
			publishAllTransforms();			//Sono ancora in fase di associazione con riconoscimento facciale, pubblico le coordinate di testa e torso di tutti gli utenti visibili
		}
		else if(skeleton_to_track > 0)
		{
			//Fase di tracking dell'utente corretto

			tf::Transform torso_local, torso_global, head_local, head_global;			//TF testa e torso, rispetto alla camera (local) e rispetto alla mappa (global)

			if(calcUserTransforms(skeleton_to_track, torso_local, torso_global, head_local, head_global))				//Controllo di validità del tracking
			{
				publishUserTransforms(skeleton_to_track, torso_local, torso_global, head_local, head_global, now);		//Se il tracking è valido pubblico le coordinate

				if(!detected)
				{
					calcUserHistogram(skeleton_to_track, userHistogram, false, it);		//Calcolo istogramma utente, senza accumularlo
				}
				else
				{
					calcUserHistogram(skeleton_to_track, userHistogram, true, it);		//Calcolo istogramma utente, accumulando ai precedenti valori
				}

				kalmanUpdate(torso_global);												//Aggiornamento filtro di Kalman

				lastDetected = now;														//Aggiorno il timestamp relativo all'ultimo istante di tracking
			}
			else																		//Tracking non valido, perdo l'utente e passo alla riassociazione con Kalman
			{
				if(detected)
				{
					//Perso il tracking imposto lo skeleton_to_track a -1, per indicare al dynamyxel_mediator che deve cercare torso_k
					skeleton_to_track = -1;
					ros::param::set("skeleton_to_track", skeleton_to_track);
				}
				else
				{
					skeleton_to_track = 0;
					ros::param::set("skeleton_to_track", skeleton_to_track);
				}
			}

			predictAndPublish();				//Predico e pubblico Kalman
		}
		else if(skeleton_to_track == -1)
		{
			//Fase di riassociazione rapida con Kalman e correlazione tra istogrammi

			XnUInt16 users_count = MAX_USERS;
			XnUserID users[MAX_USERS];

			g_UserGenerator.GetUsers(users, users_count);

			int closer = 0;
			double maxCorrelation = 0;
			tf::Transform torso_new;

			//Il ciclo seguente controlla se uno tra gli utenti visibili rispetta le condizioni di riassociazione rapida

			for(int i = 0; i < users_count; ++i)
			{
				XnUserID user = users[i];

				tf::Transform torso_local, torso_global, head_local, head_global;

				if(calcUserTransforms(user, torso_local, torso_global, head_local, head_global))
				{
					publishUserTransforms(user, torso_local, torso_global, head_local, head_global, now);

					//Calcolo la distanza tra l'utente user e la stima del filtro di Kalman
					double distance = std::sqrt(std::pow(torso_global.getOrigin().getX() - state.at<float>(0), 2) + std::pow(torso_global.getOrigin().getY() - state.at<float>(1), 2));

					if(distance < distances[user])
					{
						distances[user] = distance;

						ROS_INFO("Distance user %d: %f", user, distance);

						//Controllo se l'utente dista meno di un metro dalla stima (usando solo coordinate X e Y, Z è trascurata)
						if(distances[user] <= DISTANCE_THRESHOLD)
						{
							cv::Mat histogram;

							calcUserHistogram(user, histogram, false, it);		//Calcolo istogramma utente user, per confrontarlo con quello dell'utente corretto

							double currentCorrelation = histogramComparison(histogram);

							ROS_INFO("Correlation user %d: %f", user, currentCorrelation);

							if(currentCorrelation > maxCorrelation)
							{
								maxCorrelation = currentCorrelation;

								closer = user;									//Closer rappresenta l'utente più probabile per la riassociazione rapida

								torso_new = torso_global;
							}
						}
					}
				}
			}

			//Controllo se il valore di correlazione dell'utente più vicino supera la soglia minima imposta
			if(maxCorrelation >= LIKENESS_THRESHOLD)
			{
				//Reimposto il numero dello scheletro corretto
				skeleton_to_track = closer;
				ros::param::set("skeleton_to_track", skeleton_to_track);

				ROS_INFO("Re-associating user to skeleton %d, distance: %f, correlation: %f.", skeleton_to_track, distances[closer], maxCorrelation);

				if(log_data)
				{
					reassociations.push_back(std::pair<double, double>(distances[closer], maxCorrelation));

					for(int i = 1; i <= MAX_USERS; ++i)
					{
						distances[i] = std::numeric_limits<double>::max();
					}
				}

				detected = false;

				kalmanUpdate(torso_new);

				predictAndPublish();

				ros::Rate(30).sleep();

				continue;
			}

			//Controllo se è scaduto il tempo per la riassociazione rapida e azzero nuovamente i parametri
			if((now - lastDetected).sec >= KALMAN_TIMEOUT)
			{
				detected = false;
				skeleton_to_track = 0;
				ros::param::set("skeleton_to_track", skeleton_to_track);

				for(int i = 1; i <= MAX_USERS; ++i)
				{
					distances[i] = std::numeric_limits<double>::max();
				}

				ROS_INFO("Could not re-associate using kalman filter, returning to facial recognition re-association");
			}
		}
		else if(skeleton_to_track == -2)
		{
			//Il valore -2 è utilizzato per il reset dell'algoritmo ad accompagnamento concluso (sia positivamente che negativamente)
			detected = false;
			ros::param::set("skeleton_to_track", 0);

			if(log_data)									//Log dei dati per la riassociazione
			{
				std::ostringstream sstream;
				std_msgs::String msg;

				msg.data = "Reassociation distances:";

				logger.publish(msg);

				for(int i = 0; i < reassociations.size(); i++)
				{
					sstream << reassociations[i].first;
					msg.data = sstream.str();
					sstream.clear();
					sstream.str(std::string());
					logger.publish(msg);
				}

				msg.data = "Reassociation correlations:";

				logger.publish(msg);

				for(int i = 0; i < reassociations.size(); i++)
				{
					sstream << reassociations[i].second;
					msg.data = sstream.str();
					sstream.clear();
					sstream.str(std::string());
					logger.publish(msg);
				}

				reassociations.clear();
			}

			for(int i = 1; i <= MAX_USERS; ++i)
			{
				distances[i] = std::numeric_limits<double>::max();
			}
		}

		if(detected)
		{
			//Predico e pubblico le coordinate con il filtro di Kalman
			predictAndPublish();
		}

		//Metto in pausa ros, affinché esegua i passi del ciclo ad una frequenza di 30Hz
		ros::Rate(30).sleep();
	}

	g_Context.Shutdown();
	return EXIT_SUCCESS;
}

//----------------------------------------------Callbacks OpenNI-NiTE----------------------------------------------------------
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("New User %d. Start tracking.", nId);

	if(calibrationData == NULL)
	{
		g_UserGenerator.GetSkeletonCap().LoadCalibrationDataFromFile(nId, genericUserCalibrationFileName.c_str());
		g_UserGenerator.GetSkeletonCap().SaveCalibrationData(nId, calibrationData);
	}
	else
	{
		g_UserGenerator.GetSkeletonCap().LoadCalibrationData(nId, calibrationData);
	}

	g_UserGenerator.GetSkeletonCap().StartTracking(nId);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("Lost user %d. Stop tracking.", nId);

	g_UserGenerator.GetSkeletonCap().StopTracking(nId);
}

void XN_CALLBACK_TYPE User_BackIntoScene(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("User %d back into scene. Restart tracking.", nId);

	g_UserGenerator.GetSkeletonCap().StartTracking(nId);
}

void XN_CALLBACK_TYPE User_OutOfScene(xn::UserGenerator& generator, XnUserID nId, void* pCookie)
{
	ROS_INFO("User %d out of scene. Stop tracking.", nId);

	g_UserGenerator.GetSkeletonCap().StopTracking(nId);
}
//-----------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------Funzioni del filtro di Kalman---------------------------------------------------

void kalmanInitialization()
{
	cv::setIdentity(kf1.transitionMatrix);
	kf1.transitionMatrix.at<float>(3) = 1.0f;
	kf1.transitionMatrix.at<float>(10) = 1.0f;
	kf1.transitionMatrix.at<float>(17) = 1.0f;

	kf1.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
	kf1.measurementMatrix.at<float>(0) = 1.0f;
	kf1.measurementMatrix.at<float>(7) = 1.0f;
	kf1.measurementMatrix.at<float>(14) = 1.0f;

	cv::setIdentity(kf1.processNoiseCov, cv::Scalar(1e-2));
	kf1.processNoiseCov.at<float>(21) = 1e-1;
	kf1.processNoiseCov.at<float>(28) = 1e-1;
	kf1.processNoiseCov.at<float>(35) = 1e-1;

	cv::setIdentity(kf1.measurementNoiseCov, cv::Scalar(1e-1));
	cv::setIdentity(kf1.errorCovPost, cv::Scalar(1e-1));

	cv::setIdentity(kf2.transitionMatrix);
	kf2.transitionMatrix.at<float>(3) = 1.0f;
	kf2.transitionMatrix.at<float>(10) = 1.0f;
	kf2.transitionMatrix.at<float>(17) = 1.0f;

	kf2.measurementMatrix = cv::Mat::zeros(measSize, stateSize, CV_32F);
	kf2.measurementMatrix.at<float>(0) = 1.0f;
	kf2.measurementMatrix.at<float>(7) = 1.0f;
	kf2.measurementMatrix.at<float>(14) = 1.0f;

	cv::setIdentity(kf2.processNoiseCov, cv::Scalar(1e-2));
	kf2.processNoiseCov.at<float>(21) = 1e-1;
	kf2.processNoiseCov.at<float>(28) = 1e-1;
	kf2.processNoiseCov.at<float>(35) = 1e-1;

	cv::setIdentity(kf2.measurementNoiseCov, cv::Scalar(1e-1));
	cv::setIdentity(kf2.errorCovPost, cv::Scalar(1e-1));
}

void predictAndPublish()
{
	kalmanPrediction();

	tf::Transform transform;
	transform.setOrigin(tf::Vector3(state.at<float>(0), state.at<float>(1), state.at<float>(2)));
	transform.setRotation(tf::Quaternion(0, 0, 0, 1));

	static tf::TransformBroadcaster br;
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), parent_frame_id, "torso_k"));
}

void kalmanPrediction()
{
	kf2.transitionMatrix.at<float>(3) = dT;
	kf2.transitionMatrix.at<float>(10) = dT;
	kf2.transitionMatrix.at<float>(17) = dT;

	state = kf2.predict();
}

void kalmanUpdate(tf::Transform transform)
{
	if(!detected)
	{
		detected = true;

		kf1.statePre.at<float>(0) = transform.getOrigin().getX();
		kf1.statePre.at<float>(1) = transform.getOrigin().getY();
		kf1.statePre.at<float>(2) = transform.getOrigin().getZ();
		kf1.statePre.at<float>(3) = 0;
		kf1.statePre.at<float>(4) = 0;
		kf1.statePre.at<float>(5) = 0;

		kf2.statePre.at<float>(0) = transform.getOrigin().getX();
		kf2.statePre.at<float>(1) = transform.getOrigin().getY();
		kf2.statePre.at<float>(2) = transform.getOrigin().getZ();
		kf2.statePre.at<float>(3) = 0;
		kf2.statePre.at<float>(4) = 0;
		kf2.statePre.at<float>(5) = 0;

		kf1.predict();
		state = kf2.predict();

		oldPosition = cv::Point(state.at<float>(0), state.at<float>(1));
	}
	else
	{
		meas.at<float>(0) = transform.getOrigin().getX();
		meas.at<float>(1) = transform.getOrigin().getY();
		meas.at<float>(2) = transform.getOrigin().getZ();

		kf1.correct(meas);

		kf1.transitionMatrix.at<float>(3) = dT;
		kf1.transitionMatrix.at<float>(10) = dT;
		kf1.transitionMatrix.at<float>(17) = dT;

		state = kf1.predict();

		cv::Mat meas2(measSize, 1, CV_32F);

		meas2.at<float>(0) = state.at<float>(0);
		meas2.at<float>(1) = state.at<float>(1);
		meas2.at<float>(2) = state.at<float>(2);

		kf2.correct(meas2);
	}
}

//--------------------------------------------Funzioni relative alle coordinate utente e alla libreria TF-----------------------------------------------

void publishUserTransforms(int user, tf::Transform torso_local, tf::Transform torso_global, tf::Transform head_local, tf::Transform head_global, ros::Time now)
{
    static tf::TransformBroadcaster broadcaster;

    std::ostringstream oss;

	oss << "torso_" << user;
	broadcaster.sendTransform(tf::StampedTransform(torso_local, now, frame_id, oss.str()));

	oss.str("");
	oss.clear();
	oss << "global_torso_" << user;
	broadcaster.sendTransform(tf::StampedTransform(torso_global, now, parent_frame_id, oss.str()));

	oss.str("");
	oss.clear();
	oss << "head_" << user;
	broadcaster.sendTransform(tf::StampedTransform(head_local, now, frame_id, oss.str()));

	oss.str("");
	oss.clear();
	oss << "global_head_" << user;
	broadcaster.sendTransform(tf::StampedTransform(head_global, now, parent_frame_id, oss.str()));
}

void publishAllTransforms()
{
	XnUInt16 users_count = MAX_USERS;
	XnUserID users[MAX_USERS];

	g_UserGenerator.GetUsers(users, users_count);

	ros::Time now = ros::Time::now();

	for(int i = 0; i < users_count; ++i)
	{
		XnUserID user = users[i];

		tf::Transform torso_local, torso_global, head_local, head_global;

		if(calcUserTransforms(user, torso_local, torso_global, head_local, head_global))
		{
			publishUserTransforms(user, torso_local, torso_global, head_local, head_global, now);
		}
    }
}

bool checkCenterOfMass(XnUserID const& user)									//Controllo la validità del centroide, aiuta a capire se ho perso il tracking
{
	XnPoint3D center_of_mass;
	XnStatus status = g_UserGenerator.GetCoM(user, center_of_mass);

	if(status != XN_STATUS_OK || center_of_mass.X == 0 || center_of_mass.Y == 0 || center_of_mass.Z == 0)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool calcUserTransforms(XnUserID const& user, tf::Transform& torso_local, tf::Transform& torso_global, tf::Transform& head_local, tf::Transform& head_global)
{																				//Calcolo e pubblico le coordinate di testa e torso dell'utente user
	XnSkeletonJointPosition torso_position, head_position;
	XnSkeletonJointOrientation torso_orientation, head_orientation;

	if(g_UserGenerator.GetSkeletonCap().IsTracking(user) && checkCenterOfMass(user))
	{
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_TORSO, torso_position);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_TORSO, torso_orientation);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(user, XN_SKEL_HEAD, head_position);
		g_UserGenerator.GetSkeletonCap().GetSkeletonJointOrientation(user, XN_SKEL_HEAD, head_orientation);

		if(torso_position.fConfidence < 1 && head_position.fConfidence < 1)
		{
			return false;
		}
	}
	else
	{
		return false;
	}

	double torso_x = -torso_position.position.X / 1000.0;
	double torso_y = torso_position.position.Y / 1000.0;
	double torso_z = torso_position.position.Z / 1000.0;
	double head_x = -head_position.position.X / 1000.0;
	double head_y = head_position.position.Y / 1000.0;
	double head_z = head_position.position.Z / 1000.0;

	XnFloat* torso_m = torso_orientation.orientation.elements;
	KDL::Rotation torso_rotation(torso_m[0], torso_m[1], torso_m[2], torso_m[3], torso_m[4], torso_m[5], torso_m[6], torso_m[7], torso_m[8]);
	double torso_qx, torso_qy, torso_qz, torso_qw;
	torso_rotation.GetQuaternion(torso_qx, torso_qy, torso_qz, torso_qw);
	torso_local.setOrigin(tf::Vector3(torso_x, torso_y, torso_z));
	torso_local.setRotation(tf::Quaternion(torso_qx, -torso_qy, -torso_qz, torso_qw));

	XnFloat* head_m = head_orientation.orientation.elements;
	KDL::Rotation head_rotation(head_m[0], head_m[1], head_m[2], head_m[3], head_m[4], head_m[5], head_m[6], head_m[7], head_m[8]);
	double head_qx, head_qy, head_qz, head_qw;
	head_rotation.GetQuaternion(head_qx, head_qy, head_qz, head_qw);
	head_local.setOrigin(tf::Vector3(head_x, head_y, head_z));
	head_local.setRotation(tf::Quaternion(head_qx, -head_qy, -head_qz, head_qw));

	tf::Transform change_frame;
	change_frame.setOrigin(tf::Vector3(0, 0, 0));
	tf::Quaternion frame_rotation;
	frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
	change_frame.setRotation(frame_rotation);

	torso_local = change_frame * torso_local;
	head_local = change_frame * head_local;

	torso_global = parentTransform * torso_local;
	head_global = parentTransform * head_local;

	return true;
}

bool updateParent()								//Aggiorna ad ogni iterazione del ciclo principale le coordinate della camera rispetto al punto origine map
{
	static tf::TransformListener listener;

	try
	{
		listener.lookupTransform(parent_frame_id, frame_id, ros::Time(0), parentTransform);
		return true;
	}
	catch(tf::TransformException& ex)
	{
		return false;
	}
}
//------------------------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------------Funzioni di calcolo e comparazione dell'istogramma dei colori-------------------------------------------------
void updateHSVImage(const sensor_msgs::ImageConstPtr& msg)								//Aggiorno ad ogni iterazione l'immagine HSV a partire da quella RGB
{
	cv::Mat image;

	try
	{
		image = cv_bridge::toCvShare(msg, "bgr8")->image;

		if(image.empty())
		{
			return;
		}

		cv::cvtColor(image, image, cv::COLOR_BGR2HSV);

		HSVImage = image.clone();
	}
	catch(cv_bridge::Exception& e)
	{
		//Conversion failed
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());

		return;
	}
}

void calcUserHistogram(int user, cv::Mat& histogram, bool accumulate, image_transport::ImageTransport& it)		//Calcolo l'istogramma dell'utente user e
{																												//pubblico l'immagine della sagoma dell'utente
	xn::SceneMetaData smd;																						//in un topic apposito
	xn::DepthMetaData dmd;

	g_DepthGenerator.GetMetaData(dmd);
	g_SceneAnalyzer.GetMetaData(smd);

	const XnLabel* pLabels = smd.Data();

	cv::Mat maskImage = cv::Mat(dmd.FullYRes(), dmd.FullXRes(), CV_8UC1);

	for(XnUInt y = 0; y < dmd.YRes(); ++y)
	{
		for(XnUInt x = 0; x < dmd.XRes(); ++x, ++pLabels)
		{
			maskImage.at<uchar>(y, x) = (*pLabels == user) ? 255 : 0;
		}
	}

	const int histSize[] = {50, 60};
	const int channels[] = {0, 1};

	float h_range[] = {0, 180};
	float s_range[] = {0, 256};
	const float *histRange[] = {h_range, s_range};

	cv::calcHist(&HSVImage, 1, channels, maskImage, histogram, 2, histSize, histRange, true, accumulate);
	cv::normalize(histogram, histogram, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

	cv::Mat maskedImage;
	HSVImage.copyTo(maskedImage, maskImage);
	cv::cvtColor(maskedImage, maskedImage, cv::COLOR_HSV2BGR);
	cv::bitwise_not(maskImage, maskImage);
	cv::Mat white(maskedImage.rows, maskedImage.cols, maskedImage.type(), cv::Scalar(255, 255, 255));
	white.copyTo(maskedImage, maskImage);

	if(image_publishers.count(user) == 0)
	{
		std::stringstream topic_string;
		topic_string << "histograms/user_" << user;

		image_publishers[user] = it.advertise(topic_string.str(), 1);
	}

	sensor_msgs::ImagePtr msg;
	msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", maskedImage).toImageMsg();

	image_publishers[user].publish(msg);
}

double histogramComparison(cv::Mat histogram)					//Comparazione tra istogrammi
{
	return cv::compareHist(userHistogram, histogram, CV_COMP_CORREL);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------
