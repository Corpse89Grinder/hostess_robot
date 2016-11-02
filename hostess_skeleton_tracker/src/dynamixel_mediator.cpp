#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sstream>
#include <limits>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <std_msgs/String.h>
#include "pan_controller.hpp"

#define DISTANCE_THRESHOLD 0.1											//Distanza massima in metri per l'associazione volto-scheletro
#define MINIMUM_ASSOCIATIONS_FOR_TRACKING 2								//Numero di possibili associazioni minime prima di proseguire nell'associazione (serve ad evitare associazioni a persone non corrette)
#define MAX_MEAN 10														//Finestra del filtro a media mobile
#define CAMERA_RESET 30													//Timeout per il reset dell'orientamento della camera
#define SESSION_ABORT 120												//Timeout per l'annullamento della sessione
#define PI 3.14159265358979323846										//Pi greco per le conversioni angolari

//---------------------------------Funzioni per l'associazione iniziale e per la fase di accompagnamento-------------------------------------
void lookForEveryHeadTransform(tf::TransformListener&, std::vector<tf::StampedTransform>&, std::string);
bool findClosestHeadToFace(std::vector<tf::StampedTransform>&, std::string&);
std::string lookForSpecificBodyTransform(tf::TransformListener&, std::string, std::string, tf::StampedTransform&);
int changeFrameAndReturnIndex(std::string&);
//-------------------------------------------------------------------------------------------------------------------------------------------

//----------------------------------Callback di ROS-----------------------------------------------
void twistCallback(geometry_msgs::Twist);
void goalCallback(actionlib_msgs::GoalStatusArray);
//------------------------------------------------------------------------------------------------

void resetLoop();														//Funzione per la reinizializzazione dell'algoritmo

geometry_msgs::Twist newTwist;											//Il messaggio twist in cui andrò a scrivere i valori di velocità mediati

std::map<std::string, std::pair<ros::Time, int> > skeletons;			//Hash map che contiene gli scheletri nel campo di vista e i tempi relativi alla loro prima visualizzazione (serve ad evitare associazioni errate)
std::map<std::string, ros::Time> last_stamp;							//Hash map che contiene il nome della giuntura che l'algoritmo cerca nel campo di vista (es. torso_3) e il momento in cui è stata visualizzata per l'ultima volta (tenerne traccia è l'unico modo per non considerarla più di una volta)
std::map<std::string, std::pair<ros::Time, int> > goals_status;			//Hash map che contiene i goal e i loro stati

std::vector<double> association_distances;								//Tengo traccia delle distanze tra volti e teste nel momento dell'associazione iniziale (per il logger)

int skeleton_to_track = 0;												//Parametro intero che contiene l'ID dello scheletro corretto da considerare nel tracking

std::string current_goal_id = "";										//Stringa che conterrà l'ID del goal selezionato al momento del login

ros::Publisher pub, cancel, logger;										//I publisher di ROS utilizzati

std::deque<double> speed(MAX_MEAN, 0);									//Filtro a media mobile per la velocità

bool log_data = false;													//Variabile booleana che sceglie se effettuare il log o no dei dati. Per modificarla utilizzare l'argomento specifico nel file di lancio (qui è meglio lasciarla su false)

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dynamixel_mediator");

	ros::NodeHandle nh;

	std::string user_to_track;											//Stringa che conterrà l'ID dell'utente corretto, come da database, ricevuta al momento del login
	std::string frame_id;												//Frame TF della camera

	ros::param::set("skeleton_to_track", skeleton_to_track);			//Inizializzo a 0 l'ID dello scheletro da trackare

	ros::param::get("log_data", log_data);								//Recupero la condizione di log dal file .launch

	PanController pan_controller(nh);									//Istanza del controllore del pan-tilt

	ROS_INFO("Waiting for reference frame.");

	while(!ros::param::get("camera_frame_id", frame_id) && nh.ok())		//Leggo dai parametri ROS il frame della camera corretto
	{
		ros::Duration(1).sleep();
	}

	std::string topic_to_subscribe("/kobra/tracker_cmd_vel");			//Topic da cui leggo i valori originali di velocità per il robot
    nh.getParam("topic_to_subscribe", topic_to_subscribe);				//Aggiorno il topic nel caso sia diverso da quello scritto nella stringa
    std::string topic_to_advertise("/kobra/cmd_vel");					//Topic in cui andrò a spedire i valori mediati di velocità per il robot
    nh.getParam("topic_to_advertise", topic_to_advertise);				//Aggiorno il topic nel caso sia diverso da quello scritto nella stringa

	ros::Subscriber twistSubscriber = nh.subscribe(topic_to_subscribe, 1, twistCallback);			//Subscriber per ricevere i valori di velocità dal modulo di navigazione
    pub = nh.advertise<geometry_msgs::Twist>(topic_to_advertise, 1);								//Publisher per spedire i valori di velocità mediati

    ros::Subscriber goalSubscriber = nh.subscribe("/move_base/status", 1, goalCallback);			//Subscriber per leggere il goal corretto e il suo stato

    cancel = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);							//Publisher per cancellare la action del move_base in caso di annullamento della sessione

    if(log_data)
    {
    	logger = nh.advertise<std_msgs::String>("logger", 10);			//Publisher per il logger, solo in caso di scelta di log positiva
    }

    tf::TransformListener listener;										//Listener TF

    double ratio = 0;													//Variabile che indica la percentuale con cui andrò a mediare la velocità del robot (numero reale da 0 a 1)

    std_msgs::String msg;												//Messaggio utilizzato dal logger

	while(nh.ok())														//Ciclo padre, attende identità e goal e poi avvia l'accompagnamento
	{
		ROS_INFO("Waiting for user identity.");

		while(!ros::param::get("user_to_track", user_to_track) && nh.ok())			//Attendo fino a che non ricevo l'identità dell'utente da accompagnare
		{
			ros::Duration(1).sleep();
		}

		user_to_track = user_to_track.substr(1, user_to_track.size());				//Depuro la stringa dall'underscore _ iniziale

		ROS_INFO("User identity received.");

		ROS_INFO("Waiting for goal.");

		while(current_goal_id == "")												//Attendo l'arrivo del goal corretto
		{
			ros::spinOnce();
			ros::Duration(1).sleep();
		}

		ROS_INFO("Goal received.");

		if(log_data)																//Effettuo il log di ID utente e goal
		{
			//Log identity, goal id and time
			msg.data = "start";
			logger.publish(msg);

			msg.data = user_to_track;
			logger.publish(msg);

			msg.data = current_goal_id;
			logger.publish(msg);

			std::ostringstream sstream;

			sstream << goals_status[current_goal_id].first.sec << "." << goals_status[current_goal_id].first.nsec;
			msg.data = sstream.str();
			logger.publish(msg);

			association_distances.clear();
		}

		while(goals_status[current_goal_id].second != 3 && nh.ok())					//Ciclo dell'accompagnamento, che contiene le fasi di associazione, di tracking e riassociazione rapida
		{
			ROS_INFO("Looking for %s's face.", user_to_track.c_str());

			std::string skeleton_to_track_frame;

			ros::Time reset = ros::Time::now();										//Timeout per il reset dell'orientamento della camera
			ros::Time abort = ros::Time::now();										//Timeout per annullare la sessione di accompagnamento

			bool restart = false;

			while(nh.ok())															//Ciclo che effettua l'associazione volto-scheletro
			{
				std::vector<tf::StampedTransform> transforms;						//Vettore di TF che conterrà tutte le teste degli scheletri nel campo di vista

				lookForEveryHeadTransform(listener, transforms, user_to_track);		//Cerco le teste degli scheletri

				if(findClosestHeadToFace(transforms, skeleton_to_track_frame))		//Controllo se ho una corrispondenza volto corretto-testa di uno scheletro
				{
					skeleton_to_track = changeFrameAndReturnIndex(skeleton_to_track_frame);			//Recupero l'ID dello scheletro corretto sotto forma di intero (fino ad ora era su stringa)
					ros::param::set("skeleton_to_track", skeleton_to_track);						//Comunico allo skeleton tracker l'ID dello scheletro di cui tenere traccia
					break;
				}

				ratio = std::max(0.0, ratio - 0.005);								//Riduco gradualmente la percentuale di velocità da applicare alle ruote
				speed.pop_front();													//Elimino il campione più vecchio dalla media mobile
				speed.push_back(ratio);												//Aggiungo il nuovo campione alla media mobile

				ros::spinOnce();													//Lascio agire le callback per impartire il moto al robot (in caso di moto residuo da una precedente associazione poi persa)

				if((ros::Time::now() - reset).sec >= CAMERA_RESET && !pan_controller.isHome())		//Tempo di reset dell'orientamento della camera scaduto
				{
					pan_controller.goHome();										//Mando la camera nella posizione centrale
				}

				if((ros::Time::now() - abort).sec >= SESSION_ABORT)					//Non ho l'utente corretto da più di 2 minuti, annullo la sessione e mando il robot nella posizione iniziale. E' qui che posso lanciare eventuali allarmi o notificare al log l'accaduto.
				{
					ROS_INFO("Timout exceeded, restarting.");
					restart = true;

					break;
				}

				ros::Rate(30).sleep();												//Attendo che termini la finestra temporale del fotogramma
			}

			if(restart)																//Effettuo il reset dell'algoritmo in caso di timeout
			{
				resetLoop();

				break;
			}

			//A questo punto, se sono qui è perché ho associato un volto e uno scheletro di cui conosco l'ID, procedo nell'accompagnare l'utente
			ROS_INFO("User %s and skeleton %s associated. Start tracking.", user_to_track.c_str(), skeleton_to_track_frame.c_str());

			while(ros::param::get("skeleton_to_track", skeleton_to_track) && goals_status[current_goal_id].second != 3 && nh.ok())			//Ciclo di accompagnamento
			{
				if(skeleton_to_track == 0)							//Controllo subito un'eventuale perdita totale del tracking da parte dello skeleton tracker
				{
					ROS_INFO("User %s and skeleton %s association lost. Stop tracking.", user_to_track.c_str(), skeleton_to_track_frame.c_str());
					pan_controller.standStill();					//Blocco la camera nella sua posizione attuale

					break;											//Abbandono il ciclo di accompagnamento per ripartire dall'associazione volto-scheletro
				}

				tf::StampedTransform transform;

				std::string returnString = lookForSpecificBodyTransform(listener, frame_id, skeleton_to_track_frame, transform);			//Cerco il torso dell'utente corretto o della stima del filtro di Kalman

				if(returnString == "found")							//Ho trovato un nuovo set di coordinate del torso dell'utente
				{
					double distance = std::sqrt(std::pow(transform.getOrigin().getX(), 2) + std::pow(transform.getOrigin().getY(), 2));		//Calcolo la distanza tra la camera e l'utente corretto (planare e non spaziale)

					double alphaRAD = asin(transform.getOrigin().getY() / distance);			//Angolo sotteso tra la direzione di vista della camera e la posizione dell'utente

					if(skeleton_to_track != -1)								//Sto trackando l'utente e non la stima del filtro di kalman
					{
						if(distance >= 0 && distance <= 1.5)				//Distanza dell'utente inferiore al metro e mezzo
						{
							ratio = 1;										//Velocità massima
						}
						else if(distance > 1.5)								//Distanza maggiore del metro e mezzo
						{
							ratio = std::max(1 - (distance - 1.5), 0.0);	//Velocità mediata tra 1 e 0 (0 per distanze maggiori o uguali ai 2 metri e mezzo)
						}

						//Controllo per sicurezza il valore di percentuale velocità prima di muovere il robot
						if(ratio < 0)
						{
							ratio = 0;
						}
						else if(ratio > 1)
						{
							ratio = 1;
						}
					}
					else											//Sto trackando la stima del filtro di kalman perché ho perso l'utente
					{
						ratio = std::max(0.0, ratio - 0.005);		//Riduco gradualmente la velocità del robot
					}

					speed.pop_front();								//Aggiorno il filtro a media mobile
					speed.push_back(ratio);

					ros::spinOnce();								//Lascio agire le callback

					pan_controller.turn(alphaRAD, newTwist.angular.z);			//Comunico al pan controller l'angolo tra il vettore di vista della camera e l'utente, unitamente alla rotazione effettuata dal robot per controcorreggerla
				}
				else if(returnString == "not found")				//Non ho trovato le coordinate
				{
					ratio = std::max(0.0, ratio - 0.005);			//Riduco gradualmente la velocità
					speed.pop_front();								//Aggiorno il filtro a media mobile
					speed.push_back(ratio);
					pan_controller.standStill();					//Fermo la camera

					ros::spinOnce();								//Lascio agire le callback
				}
				else if(returnString == "skip")						//Ho trovato le coordinate ma sono le stesse trovate in precedenza, salto il calcolo
				{
					pan_controller.continueTurning();				//Proseguo con la precedente rotazione della camera

					ros::spinOnce();								//Lascio agire le callback
				}

				ros::Rate(30).sleep();								//Attendo fino al prossimo fotogramma
			}
		}

		if(goals_status[current_goal_id].second == 3)				//Controllo se ho raggiunto il goal
		{
			ROS_INFO("Goal reached correctly, restarting.");

			if(log_data)											//Mando al logger i dati raccolti
			{
				msg.data = "Association distances:";
				logger.publish(msg);

				std::ostringstream sstream;

				for(int i = 0; i < association_distances.size(); i++)
				{
					sstream.clear();
					sstream.str(std::string());
					sstream << association_distances[i];
					msg.data = sstream.str();
					logger.publish(msg);
				}
			}

			ratio = 0;												//Azzero la percentuale di velocità del robot

			resetLoop();											//Ri effettuo l'inizializzazione

			if(log_data)											//Invio al logger la conferma per il salvataggio dei dati su file
			{
				msg.data = "succeeded";
				logger.publish(msg);
			}

			//Dovrei mandare il robot alla posizione iniziale, ma non essendo definita ho saltato questa parte, basta una action a delle coordinate fisse
		}

		pan_controller.goHome();									//Resetto la posizione della camera
	}

	ROS_INFO("Shutting down.");

    ros::shutdown();

    exit(EXIT_SUCCESS);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------Funzioni per l'associazione iniziale e per la fase di accompagnamento-------------------------------------
void lookForEveryHeadTransform(tf::TransformListener& listener, std::vector<tf::StampedTransform>& transforms, std::string user_to_track)
{
	//Per ogni scheletro possibilmente nella vista dello skeleton tracker, cerco le coordinate della testa corssipondente, relativamente alle coordinate del volto dell'utente

	for(int i = 1; i <= 15; i++)
	{
		std::ostringstream oss;
		oss << "head_" << i;

		try
		{
			tf::StampedTransform transform;

			listener.lookupTransform(user_to_track, oss.str(), ros::Time(0), transform);

			if(transform.stamp_ != last_stamp[oss.str()])
			{
				last_stamp[oss.str()] = transform.stamp_;
				transforms.push_back(transform);
			}
		}
		catch(tf::TransformException &ex)
		{
			continue;
		}
	}
}

bool findClosestHeadToFace(std::vector<tf::StampedTransform>& transforms, std::string& skeleton_to_track_frame)
{
	//Per ogni testa trovata calcolo la distanza dalle coordinate del volto e scelgo quella più vicina e distante meno di 10cm dal volto stesso

	double min = std::numeric_limits<double>::max();
	std::string frame_to_track;

	for(int i = 0; i < transforms.size(); i++)
	{
		double current = std::sqrt(std::pow(transforms[i].getOrigin().getX(), 2) + std::pow(transforms[i].getOrigin().getY(), 2) + std::pow(transforms[i].getOrigin().getZ(), 2));

		if(current < min)
		{
			min = current;
			frame_to_track = transforms[i].child_frame_id_;
		}
	}

	ros::Time now = ros::Time::now();

	for(auto iterator = skeletons.begin(); iterator != skeletons.end();)
	{
		if((now - iterator->second.first).sec >= 1)					//Più di 1 secondo è passato dalla precedente rilevazione, probabile falso positivo del modulo di face recognition, non ne tengo più conto
		{
			iterator = skeletons.erase(iterator);
		}
		else
		{
			++iterator;
		}
	}

	if(min < DISTANCE_THRESHOLD)									//Ho trovato lo scheletro corretto
	{
		skeletons[frame_to_track].first = now;
		skeletons[frame_to_track].second++;

		if(skeletons[frame_to_track].second >= MINIMUM_ASSOCIATIONS_FOR_TRACKING)
		{
			skeleton_to_track_frame = frame_to_track;
			skeletons.clear();

			if(log_data)
			{
				association_distances.push_back(min);
			}

			return true;
		}
	}

	return false;
}

std::string lookForSpecificBodyTransform(tf::TransformListener& listener, std::string frame_id, std::string body_to_track_frame, tf::StampedTransform& transform)
{
	//Cerco il torso dell'utente o quello stimato dal filtro di Kalman

	try
	{
		if(skeleton_to_track == -1)			//Kalman
		{
			listener.lookupTransform(frame_id, "torso_k", ros::Time(0), transform);
		}
		else								//Utente
		{
			listener.lookupTransform(frame_id, body_to_track_frame, ros::Time(0), transform);
		}

		if(transform.stamp_ != last_stamp[body_to_track_frame])			//Controllo se ho una nuova rilevazione o ho trovato nuovamente l'ultima
		{
			last_stamp[body_to_track_frame] = transform.stamp_;

			return "found";
		}
		else															//Rilevazione uguale all'ultima, sto andando più veloce dello skeleton tracker, salto un fotogramma
		{
			return "skip";
		}
	}
	catch(tf::TransformException &ex)									//Torso non trovato
	{
		return "not found";
	}
}

int changeFrameAndReturnIndex(std::string& frame)						//Ritorno con l'ID dello scheletro in forma numerica e con il relativo frame testuale
{
	int index = atoi(frame.substr(frame.rfind("_") + 1, frame.length()).c_str());

	std::ostringstream oss;
	oss << "torso_" << index;
	frame = oss.str();

	return index;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------

//---------------------------------------------Callback di ROS------------------------------------------------------------------------------------------
void twistCallback(geometry_msgs::Twist oldTwist)
{
	//Medio la velocità del robot

	double ratio = 0;

	for(int i = 0; i < speed.size(); i++)
	{
		ratio += speed[i] / MAX_MEAN;
	}

	newTwist.linear.x = oldTwist.linear.x * ratio;
	newTwist.angular.z = oldTwist.angular.z * ratio;

	pub.publish(newTwist);

	return;
}

void goalCallback(actionlib_msgs::GoalStatusArray goals)
{
	//Ricevo il goal verso cui muoverò il robot accompagnando l'utente

	for(int i = 0; i < goals.status_list.size(); i++)
	{
		if(goals_status.count(goals.status_list[i].goal_id.id) == 0)
		{
			//new goal received
			current_goal_id = goals.status_list[i].goal_id.id;

			goals_status[current_goal_id].first = goals.status_list[i].goal_id.stamp;
			goals_status[current_goal_id].second = goals.status_list[i].status;

			ROS_INFO("Current goal: %s", current_goal_id.c_str());
		}
		else
		{
			goals_status[goals.status_list[i].goal_id.id].second = goals.status_list[i].status;
		}
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------

void resetLoop()
{
	//Ciclo di reset dell'algoritmo per poter procedere ad una nuova sessione

	ROS_INFO("Resetting loop.");

	for(int i = 0; i < speed.size(); i++)
	{
		speed.pop_front();
		speed.push_back(0);
	}

	ros::param::del("user_to_track");

	ros::param::set("skeleton_to_track", -2);

	actionlib_msgs::GoalID msg;

	msg.id = current_goal_id;
	msg.stamp = goals_status[current_goal_id].first;

	cancel.publish(msg);

	current_goal_id = "";

	ros::Duration(5).sleep();
}
