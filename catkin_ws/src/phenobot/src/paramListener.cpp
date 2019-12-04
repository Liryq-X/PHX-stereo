#include "paramListener.h"
#define GPIO_PORT gpio388

Param_listener::Param_listener(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{ // constructor
            ROS_INFO("in class constructor of Param_listener");

            //initialize camera 
            //set initial values

            listen_param(); 
            initParamMap();

            createDayFolder();

            cout<<"open system\n";

            system = Arena::OpenSystem();

}

Param_listener::~Param_listener()
{

    cout<<"disconnect cameras\n";

    //if((system->GetDevices()).size()>0)
        //pCameras->PHXM_disconnect();

    cout<<"close system\n";

    Arena::CloseSystem(system);
}


void Param_listener::initParamMap(){
    cout<<"initParamMap\n";
    // read from a file

    //******** Read json settings *******//
    std::ifstream config_doc("/home/nvidia/catkin_ws/src/phenobot/config.json");
    Json::Value config;
    config_doc >> config;

  for( int index = 0; index < config["cameras"].size(); index++ ) 
    {
    params_rev["exposureAuto"] = config["cameras"][index]["ExposureAuto"].asString();
    params_rev["exposureTime"] = config["cameras"][index]["ExposureTime"].asString();
    params_rev["triggerMode"] = config["cameras"][index]["TriggerMode"].asString();
    params_rev["pixelFormat"] = config["cameras"][index]["PixelFormat"].asString();
    params_rev["gainAuto"] = config["cameras"][index]["GainAuto"].asString();
    params_rev["balanceWhiteAuto"] = config["cameras"][index]["BalanceWhiteAuto"].asString();
    params_rev["gain"] = config["cameras"][index]["Gain"].asString();
    params_rev["balanceRatio"] = config["cameras"][index]["BalanceRatio"].asString();
  }
}

void Param_listener::exposureTimeCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard exposureTime: [%s]", msg->data.c_str());

  //params_rev.insert ( std::pair<string,string>("exposureTime", msg->data.c_str()) );

  if(params_rev["exposureTime"].compare(msg->data.c_str()) !=0){
    
    cout<<"before setting\n";

    pCameras->PHXM_set_exposureTime(stof(msg->data.c_str())); // 

    cout<<"after setting\n";

    params_rev["exposureTime"]= msg->data.c_str();

  }
  
}

void Param_listener::exposureAutoCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard exposureAuto: [%s]", msg->data.c_str());

 // params_rev.insert ( std::pair<string,string>("exposureAuto", msg->data.c_str()) );

  if(params_rev["exposureAuto"].compare(msg->data.c_str()) !=0){
    
    pCameras->PHXM_set_exposureAuto(msg->data.c_str());

    params_rev["exposureAuto"]= msg->data.c_str();

  }

}

void Param_listener::triggerModeCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard triggerMode: [%s]", msg->data.c_str());
 // params_rev.insert ( std::pair<string,string>("triggerMode", msg->data.c_str()) );

  if(params_rev["triggerMode"].compare(msg->data.c_str()) !=0){
    
    pCameras->PHXM_set_triggerMode(msg->data.c_str());

    params_rev["triggerMode"]= msg->data.c_str();

  }

}

void Param_listener::triggerSourceCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard triggerSource: [%s]", msg->data.c_str());
 // params_rev.insert ( std::pair<string,string>("triggerMode", msg->data.c_str()) );

  if(params_rev["triggerSource"].compare(msg->data.c_str()) !=0){
    
    pCameras->PHXM_set_triggerSource(msg->data.c_str());

    params_rev["triggerSource"]= msg->data.c_str();

  }

}

void Param_listener::triggerSelectorCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard triggerSelector: [%s]", msg->data.c_str());
 // params_rev.insert ( std::pair<string,string>("triggerMode", msg->data.c_str()) );

  if(params_rev["triggerSelector"].compare(msg->data.c_str()) !=0){
    
    pCameras->PHXM_set_triggerSelector(msg->data.c_str());

    params_rev["triggerSelector"]= msg->data.c_str();

  }

}

 void Param_listener::pixelFormatCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard pixelFormat: [%s]", msg->data.c_str());
  //params_rev.insert ( std::pair<string,string>("pixelFormat", msg->data.c_str()) );

  if(params_rev["pixelFormat"].compare(msg->data.c_str()) !=0){
    
    pCameras->PHXM_set_pixelFormat(msg->data.c_str());

    params_rev["pixelFormat"]= msg->data.c_str();

  }
}

void Param_listener::gainAutoCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard gainAuto: [%s]", msg->data.c_str());
  //params_rev.insert ( std::pair<string,string>("gainAuto", msg->data.c_str()) );

  if(params_rev["gainAuto"].compare(msg->data.c_str()) !=0){
    
    pCameras->PHXM_set_gainAuto(msg->data.c_str());

    params_rev["gainAuto"]= msg->data.c_str();

  }

}

void Param_listener::balanceWhiteAutoCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard balanceWhiteAuto: [%s]", msg->data.c_str());
  //params_rev.insert ( std::pair<string,string>("balanceWhiteAuto", msg->data.c_str()) );

  if(params_rev["balanceWhiteAuto"].compare(msg->data.c_str()) !=0){
    
    pCameras->PHXM_set_balanceWhiteAuto(msg->data.c_str());

    params_rev["balanceWhiteAuto"]= msg->data.c_str();

  }

}

void Param_listener::gainCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard gain: [%s]", msg->data.c_str());
  //params_rev.insert ( std::pair<string,string>("gain", msg->data.c_str()) );

  if(params_rev["gain"].compare(msg->data.c_str()) !=0){
    
    pCameras->PHXM_set_gain(stof(msg->data.c_str()));

    params_rev["gain"]= msg->data.c_str();

  }

}

void Param_listener::balanceRatioCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard balance ratio: [%s]", msg->data.c_str());

  //params_rev.insert ( std::pair<string,string>("gain", msg->data.c_str()) );

  if(params_rev["balanceRatio"].compare(msg->data.c_str()) !=0){
    
    pCameras->PHXM_set_balanceRatio(stof(msg->data.c_str()));

    params_rev["balanceRatio"]= msg->data.c_str();

  }

}


void Param_listener::captureCallback(const std_msgs::String::ConstPtr& msg){

    ROS_INFO("I heard: [%s]", msg->data.c_str());
//////////////
    cout<<"start capturing, check node values\n";
    pCameras->getMultiNodeMesg();
    pCameras->initializeIds();

// set save id of each camera 
//	 for(ptrCamera pCamera : pCameras->cameras_)
//		pCamera->private_save_id = pCameras->common_save_id;


    pCameras->PHXM_set_triggerSource("Line0");
    pCameras->PHXM_set_triggerMode("On");
    pCameras->PHXM_set_triggerActivation("RisingEdge");
	pCameras->PHXM_set_maxPacketSize(9000);

    pCameras->PHXM_start_capturing();

}


void Param_listener::triggerCallback(const std_msgs::String::ConstPtr& msg){

  ROS_INFO("I heard: [%s]", msg->data.c_str());

    cout<<"set triggerCamera to be true\n";

    triggerCamera = true;

    std::vector<string> vec_ts(3);


    //double initial_count = cv::getTickCount();

    for(int i = 0; i < vec_ts.size(); i++)
    {
        int r_tg = pCameras->PHXM_trigger(vec_ts[i]);

        vec_ts[i] = pCameras->PHXM_getCurrentTimeStr();

        if(r_tg==0)
            std::cout<<"Trigger "<<i << std::endl;
        else
        {
            std::cout<<"Trigger failed"<< std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(150)); 

    }

   // std::cout<<"stop count "<<((double)cv::getTickCount() - initial_count)/getTickFrequency()<<endl;

}

void Param_listener::connectCallback(const std_msgs::String::ConstPtr& msg){

            //create folder for today

            ROS_INFO("I heard: [%s]", msg->data.c_str());

            system->UpdateDevices(100);

            setSystem(system);

            camList = system->GetDevices();

            unsigned int numCameras = camList.size();

            cout<<"numCameras: "<<numCameras<<endl;

		    if (numCameras == 0)
		        {
			        throw GenICam::GenericException("No camera connected", __FILE__, __LINE__);
		        }

            cout<<"Init and connect to cameras: "<<numCameras<<endl;

            std::vector<Arena::DeviceInfo> deviceInfos;

            for(int id = 0; id < numCameras; id++)
            {

                cout<<"create device\n";

                IDevice* pCam = system->CreateDevice(camList[id]); 

                //boost::shared_ptr<PHX_camera> pCamera(new PHX_camera(pCam));

                pCamera.reset(new PHX_camera(pCam));

                cout<<"connect to the camera\n";

                pCamera->cam_connect();

                string camID = pCamera->cameraID;
                
                cout<<"camID: "<<camID<<endl;

                pCameras->PHX_new(pCamera, paras_cams[camID], id);
   

            }


            std::cout<<"Cameras connected: " << pCameras->nCams_ <<std::endl;


        //for preview function
        image_transport::ImageTransport it(nhi_); /////////

        pub1 = it.advertise("camera1/image", 1);
        pub2 = it.advertise("camera2/image", 1);

}

void Param_listener::hardwareTriggerCallback(const std_msgs::String::ConstPtr& msg){

  ROS_INFO("I heard: [%s]", msg->data.c_str());

//trigger by hardwre
    cout<<"set triggerCamera to be true\n";

    triggerCamera = true;

    std::vector<string> vec_ts(100);

    for(int i = 0; i < 1; i++)
    {
        int r_tg = pCameras->PHXM_trigger(vec_ts[i]);


        if(r_tg==0)
            std::cout<<"Trigger"<<std::endl;
        else
            std::cout<<"Trigger failed"<< std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); //50
    }


}

void Param_listener::disconnectCallback(const std_msgs::String::ConstPtr& msg){  

    ROS_INFO("I heard: [%s]", msg->data.c_str());

    pCameras->PHXM_disconnect();

}

void Param_listener::startPreviewCallback(const std_msgs::String::ConstPtr& msg){

    ROS_INFO("I heard: [%s]", msg->data.c_str());

    if(pCameras->cameras_.size()>0){

        cout<<"camera size: "<<pCameras->cameras_.size()<<endl;

        pCameras->PHX_if_preview = true;

        cout<<"set if_preview to be true\n";

        for(ptrCamera pCamera_ : pCameras->cameras_)
        { 
            pCamera_->if_preview = true;
        } 

        cout<<"add to thread\n";

        //preview_thread_added
        _threadWorker = std::thread(&Param_listener::preview, this);

    }

}

void Param_listener::stopPreviewCallback(const std_msgs::String::ConstPtr& msg){

        ROS_INFO("I heard: [%s]", msg->data.c_str());

        pCameras->PHX_if_preview = false;

        for(ptrCamera pCamera_ : pCameras->cameras_)
        { 
            pCamera_->if_preview = false;
        } 

        _threadWorker.join(); 
 
}

void Param_listener::preview(){
    
//trigger
        //triggerCamera = true;

        //int trigger_times = 100000;

        //std::vector<string> vec_ts(trigger_times);

        //int i = 0;
//get publishe image
//publihs msg  
/*  
        while(pCamera->if_preview && i<trigger_times){

            pCameras->PHXM_trigger(vec_ts[i]);

            i++;

            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            if((pCamera->publishImage).empty())
                continue;

            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pCamera->publishImage).toImageMsg();
        
            pub.publish(msg);
        }

*/

        cout<<"publish image\n";

		cout<<"triggerCamera: "<<triggerCamera<<endl;

        int cam_no = 1;

        while(pCameras->PHX_if_preview){
		
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            cam_no = 1;

            sensor_msgs::ImagePtr msg;

			if(!triggerCamera){

				gpioSetValue(GPIO_PORT, off); 

				std::this_thread::sleep_for(std::chrono::milliseconds(3));

				gpioSetValue(GPIO_PORT, on);


			}


            for(ptrCamera pCamera_ : pCameras->cameras_)
            { 

				if(!triggerCamera)
					pCamera_->showImage();

                if((pCamera_->publishImage).empty())
                    continue;


                if(cam_no==1){
                    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pCamera_->publishImage).toImageMsg();
                    pub1.publish(msg);
                }
                else{
                    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", pCamera_->publishImage).toImageMsg();
                    pub2.publish(msg);
                }
                cam_no++;

            } 
        }

}

void Param_listener::listen_param(){

  cout<<"listen_param\n";

  exposureAuto_sub = nh_.subscribe("exposureAuto", 1000, &Param_listener::exposureAutoCallback, this);

  exposureTime_sub = nh_.subscribe("exposureTime", 1000, &Param_listener::exposureTimeCallback, this);

  triggerMode_sub = nh_.subscribe("triggerMode", 1000, &Param_listener::triggerModeCallback, this);

  triggerSource_sub = nh_.subscribe("triggerSource", 1000, &Param_listener::triggerSourceCallback, this);

  triggerSelector_sub = nh_.subscribe("triggerSelector", 1000, &Param_listener::triggerSelectorCallback, this);

  pixelFormat_sub = nh_.subscribe("pixelFormat", 1000, &Param_listener::pixelFormatCallback, this);

  gainAuto_sub = nh_.subscribe("gainAuto", 1000, &Param_listener::gainAutoCallback, this);

  balanceWhiteAuto_sub = nh_.subscribe("balanceWhiteAuto", 1000, &Param_listener::balanceWhiteAutoCallback, this);

  gain_sub = nh_.subscribe("gain", 1000, &Param_listener::gainCallback, this);

  balance_ratio_sub = nh_.subscribe("balanceRatio", 1000, &Param_listener::balanceRatioCallback, this);

  trigger_sub = nh_.subscribe("trigger", 1000, &Param_listener::triggerCallback, this);

  connect_sub = nh_.subscribe("connect", 1000, &Param_listener::connectCallback, this);

  capture_sub = nh_.subscribe("capture", 1000, &Param_listener::captureCallback, this);

  hardware_trigger_sub = nh_.subscribe("hardware_trigger", 1000, &Param_listener::hardwareTriggerCallback, this);

  disconnect_sub = nh_.subscribe("disconnect", 1000, &Param_listener::disconnectCallback, this);

  start_preview_sub = nhi_.subscribe("startPreview", 1000, &Param_listener::startPreviewCallback, this);

  stop_preview_sub = nhi_.subscribe("stopPreview", 1000, &Param_listener::stopPreviewCallback, this);

}

void Param_listener::createDayFolder(){


    CurrentDayFolder = "/home/nvidia/catkin_ws/Images/" + getCurrentDateStr();

    const char* path = CurrentDayFolder.c_str();

    boost::filesystem::path dir(path);

    if(boost::filesystem::create_directory(dir)){

        cout<<"Directory created: "<< CurrentDayFolder<<endl;
    }

}


/*
string Param_listener::getCurrentDateStr(){

        ftime(&stTimeb);
        ptm = localtime(&stTimeb.time);
        sprintf(szTime, "%04d-%02d-%02d-%02d-%02d-%02d-%03d",
                ptm->tm_year + 1900, ptm->tm_mon+1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm);

        string out_str(szTime);

        return out_str;

}*/


string Param_listener::getCurrentDateStr(){

    std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    std::string s(30, '\0');
    std::strftime(&s[0], s.size(), "%Y-%m-%d", std::localtime(&now));
    return s;
}

