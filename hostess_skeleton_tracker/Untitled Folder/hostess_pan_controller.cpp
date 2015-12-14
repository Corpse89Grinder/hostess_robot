#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <cyton_wrapper/dynamixelIO_wrapper_base.h>
#include <geometry_msgs/Vector3.h>


class PanTiltTrajectory{

private:
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber mPixelSub;
    ros::Subscriber mSizeStream_Sub;
    
    boost::shared_ptr <pluginlib::ClassLoader<dynamixelIO_wrapper_base::DynamixelIO_Base> >
        dxio_loader;//("cyton_wrapper","dynamixelIO_wrapper_base::DynamixelIO_Base");
    
    boost::shared_ptr<dynamixelIO_wrapper_base::DynamixelIO_Base> dxio;
    
    int mDeviceIndex,mBaudnum, mUpdateRate;
    std::string mYamlPath;
    double PanPosition;
    double TiltPosition;
    double initPix[2];
    double alpha_x;
    double alpha_y;
    double imgWidth;
    double imgHeight;
    

public:
    PanTiltTrajectory():private_nh_("~"),PanPosition(0.0), TiltPosition(0.0), alpha_x(1.0), alpha_y(1.0){
        
        private_nh_.param("DeviceIndex",mDeviceIndex,1);
        private_nh_.param("Baudnum",mBaudnum,34);
        private_nh_.param("UpdateRate",mUpdateRate,20);     //in Hertz
        std::string tempYamlPath("/home/rob/virgil_ws/src/cyton_wrapper/config/PT_joint.yaml");
        private_nh_.param("YamlPath",mYamlPath,tempYamlPath);
        // subscriber che legge le coordinate x e y cliccate sullo stream video
        mPixelSub = nh_.subscribe<geometry_msgs::Vector3>("/pan_tilt_trajectory",100,&PanTiltTrajectory::pixel_trajectory_CB,this);
        // subscriber che legge la dimensione del tag img dello stream video
        mSizeStream_Sub = nh_.subscribe<geometry_msgs::Vector3>("/size_stream",1,&PanTiltTrajectory::size_stream_CB,this);
        
            //verifico se il plugin può essere caricato
        //new pluginlib::ClassLoader<T>(std::string package,std::string 	base_class,)
        
                try {
            dxio_loader.reset(new pluginlib::ClassLoader<dynamixelIO_wrapper_base::DynamixelIO_Base>("cyton_wrapper","dynamixelIO_wrapper_base::DynamixelIO_Base"));
        } catch (pluginlib::PluginlibException& e) {
            ROS_ERROR("The plugin_loader failed to load for some reason. Error: %s", e.what());
        }
        
        //creo un'istanza del plugin, così dxio potrà accedere ai metodi della classe DynamixelIO_Base nel namespace dynamixelIO_wrapper_base
        
        try {
            dxio = dxio_loader->createInstance("dynamixelIO_wrapper_plugin::DynamixelIO");
            ROS_INFO("Loading PlugIn IO...");
            dxio->initialize(mDeviceIndex,mBaudnum,mUpdateRate,mYamlPath,&nh_);
            
            dxio->start();
        } catch (pluginlib::PluginlibException& e) {
            ROS_ERROR("The plugin_loader failed to load for some reason. Error: %s", e.what());

        }
       
    }
    
    void size_stream_CB(const geometry_msgs::Vector3::ConstPtr& wh);
    void pixel_trajectory_CB(const geometry_msgs::Vector3::ConstPtr& uv);
    


};

void PanTiltTrajectory::size_stream_CB(const geometry_msgs::Vector3::ConstPtr& wh){

    imgWidth = wh->x;
    imgHeight = wh->y;
    std::cout<<"imgWidth \n"<<imgWidth<<"\n";
    std::cout<<"imgHeight \n"<<imgHeight<<"\n";
    return;
    
}



void PanTiltTrajectory::pixel_trajectory_CB(const geometry_msgs::Vector3::ConstPtr& uv){

    //dxio -> getMinAngle(ID), dxio->getMaxAngle(ID) leggono angolo min e max da arm_joint.yaml
    //  PAN ID = 0
    //  TILT ID = 1
    
    /*double minanglePan = (dxio->getMinAngle(0) - 512) * 0.005113269;
    double maxanglePan = (dxio->getMaxAngle(0) - 512) * 0.005113269;
    double minangleTilt = (dxio->getMinAngle(1) - 512) * 0.005113269;
    double maxangleTilt = (dxio->getMaxAngle(1) - 512) * 0.005113269;*/
    
    //Modifica
    double minanglePan = (dxio->getMinAngle(0) - 2048) * 0.0015339804;
    double maxanglePan = (dxio->getMaxAngle(0) - 2048) * 0.0015339804;
    double minangleTilt = (dxio->getMinAngle(1) - 2048) * 0.0015339804;
    double maxangleTilt = (dxio->getMaxAngle(1) - 2048) * 0.0015339804;
     ROS_INFO("minanglePan: %f, maxanglePan: %f, minangleTilt: %f, maxangleTilt: %f", minanglePan, maxanglePan, minangleTilt, maxangleTilt);

    // added later	
//    double imgWidth = 643.0; //added 12/10/15
//    double imgHeight = 580.0;//added 12/10/15 
 
 
   //
    
    //double imgWidth = 613;
    //double imgHeight = 313;
    double in_min_pan = 0;
    double in_min_tilt = 0;
    double in_max_pan = imgWidth;
    double in_max_tilt = imgHeight;
    //Modifica
    double vel_pan = 1.0;
    double vel_tilt = 0.5;
    //
    
    std::vector< std::vector<double> > trajectPanTilt;
    std::vector<double> pv;     //pv posizione - velocità
    
    //controllo se z == 1 per capire quando scatta l'evento mousedown e quindi che valori attuali hanno sia pan che tilt e quali coordinate u e v ho cliccato sullo stream, questo perchè tutti gli spostamenti delta saranno riferiti a quei punti e a quegli angoli
    if(uv->z <= 1.01 && uv->z >= 0.99){
        
        int count_PAN = 0;
        int count_TILT = 0;
        //dxio->getPresentPosition(ID,double PresentPosRad))
        //controllo per verificare di leggere veramente i valori attuali degli angoli e setto le varibili PanPositione e TiltPosition
        while(!dxio->getPresentPosition(0,PanPosition))count_PAN++;
        while(!dxio->getPresentPosition(1,TiltPosition))count_TILT++;
        
        //salvo u e v di partenza
        initPix[0] = uv->x;
        initPix[1] = uv->y;
        ROS_INFO("PanPos: %d,Iterat: %d, TiltPos: %d, Iterat: %d", PanPosition,count_PAN,TiltPosition,count_TILT);
        ROS_INFO("initPix[0]: %f, initPix[1]: %f \n", initPix[0],initPix[1]);
        return;
            }
    //setto posizione centrale
    if(uv->x == -1 && uv->y == -1 && uv->z == -1){
        pv.resize(2);
        pv[0] = 0.0;
        pv[1] = (3.0);
        trajectPanTilt.push_back(pv);
        ROS_INFO("Pan %f, %f",pv[0],pv[1]);
        pv.clear();
        pv.resize(2);
        pv[0] = 0.0;
        pv[1] = (3.0);
        trajectPanTilt.push_back(pv);
        ROS_INFO("Tilt %f, %f",pv[0],pv[1]);
        dxio->setMultiPosVel(trajectPanTilt);
        return;
    
    }
    
    //una volta settati i parametri di partenza possiamo calcolare i deltax e deltay da questi parametri iniziali
    //entriamo qui quando scatta l'evento mousemove e infatti è z == 0
    
          //PAN
    pv.resize(2);
    ROS_INFO("initPix[0]: %f, initPix[1]: %f", initPix[0],initPix[1]);
    // delta lungo x rispetto al punto u cliccato all'inizio
    double deltaXpx = uv->x - initPix[0];
    ROS_INFO("DeltaXpx %f", deltaXpx);
   
    //Modifica
    alpha_x = ((maxanglePan-minanglePan)/imgWidth); //added 12/10/15
    //
    //alpha_x = (2.0 * atan(0.5 * (432.0/497.679177)))/imgWidth;
    
    pv[0]= PanPosition - deltaXpx * alpha_x; //added 12/10/15 cambiado de signo
    
    //pv[0] = /*static_cast<int16_t>*/PanPosition  + deltaXpx * alpha_x;
    if(pv[0] < minanglePan){
        pv[0] = /*static_cast<int16_t>*/(minanglePan);
        ROS_INFO("pv[0] saturated min PAN");
    }
    
    if(pv[0] > maxanglePan){
        pv[0] =/* static_cast<int16_t>*/(maxanglePan);
        ROS_INFO("pv[0] saturated max PAN");
    }
    
    
    //pv[0] = static_cast<int16_t>((((fabs(deltaXpx) - in_min_pan)*(maxanglePan - minanglePan))/(in_max_pan - in_min_pan)) + minanglePan);
    
    //pv[1] = /*static_cast<int16_t>*/(22.2);    //ax18 deltav = 0.111rpm 0-1023
    //Modifica
    pv[1] = vel_pan;    //ax18 deltav = 0.111rpm 0-1023 velocita motor Pan
    //
    /*if(deltaXpx < 0){
        pv[0] = PanPosition - pv[0];
    }
    else{
        pv[0] = PanPosition + pv[0];
    }*/
    //ROS_INFO("pv[0] %d", pv[0]);
    
    trajectPanTilt.push_back(pv);
    ROS_INFO("Pan %f, %f",pv[0],pv[1]);
    pv.clear();
    
    //TILT
    pv.resize(2);
    double deltaYpx = uv->y - initPix[1];
    
    //Modifica
    alpha_y = ((maxangleTilt-minangleTilt)/imgHeight); //added 12/10/15
    //alpha_y = (2.0 * atan(0.5 * (240.0/502.383070)))/imgHeight;
    
    //pv[0] = /*static_cast<int16_t>*/TiltPosition + deltaYpx * alpha_y;
    pv[0] = TiltPosition - deltaYpx * alpha_y; //added 12/10/15 cambiado de signo
    if(pv[0] < minangleTilt){
        pv[0] = /*static_cast<int16_t>*/(minangleTilt);
        ROS_INFO("pv[0] saturated min TILT");
    }
    
    if(pv[0] > maxangleTilt){
        pv[0] = /*static_cast<int16_t>*/(maxangleTilt);
        ROS_INFO("pv[0] saturated max TILT");
    }
    //pv[1] = /*static_cast<int16_t>*/(22.2);    //ax18 deltav = 0.111rpm 0-1023
    //Modifica
    pv[1] = vel_tilt;  //ax18 deltav = 0.111rpm 0-1023 velocita motor Tilt
    //
    trajectPanTilt.push_back(pv);
    ROS_INFO("Tilt %f, %f",pv[0],pv[1]);
    dxio->setMultiPosVel(trajectPanTilt);


}

int main(int argc, char** argv){
 
    ros::init(argc,argv,"PanTiltTrajectoryExecutor");
    PanTiltTrajectory ptt;
    ros::spin();

}


