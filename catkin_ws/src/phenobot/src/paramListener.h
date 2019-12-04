#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <string>
#include <map>
#include <iostream>
#include "lucidPHX.h"
#include <json/json.h>
//#include <jsoncpp/json/json.h>
#include <json/value.h>

#include <chrono>  // chrono::system_clock
#include <ctime>   // localtime
#include <sstream> // stringstream
#include <iomanip> // put_time


//ros publish image
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>

using namespace std;

class Param_listener: public PHXMulti
{

public:
    Param_listener(ros::NodeHandle* nodehandle); 

    ~Param_listener();

    std::map<string, string> params_rev;

    std::map<string, std::map<string, string>> paras_cams;

    boost::shared_ptr<PHXMulti> pCameras{new PHXMulti() }; 

    boost::shared_ptr<PHX_camera> pCamera; 

    //boost::shared_ptr<PHX_camera> pCamera{new PHX_camera()}; 

    std::vector<boost::shared_ptr<PHX_camera>> cameras;

    std::vector<Arena::DeviceInfo> camList;

    ISystem*  system;

    bool triggerCamera = false;

    std::thread _threadWorker;

    bool globalPreview = false;

    struct  tm      *ptm;
    struct  timeb   stTimeb;
    static  char    szTime[30];

private:

    ros::NodeHandle nh_;
    ros::NodeHandle nhi_;

    ros::Subscriber exposureAuto_sub;
    ros::Subscriber exposureTime_sub;
    ros::Subscriber triggerMode_sub;
    ros::Subscriber triggerSource_sub;
    ros::Subscriber triggerSelector_sub;
    ros::Subscriber pixelFormat_sub;
    ros::Subscriber gainAuto_sub;
    ros::Subscriber balanceWhiteAuto_sub;
    ros::Subscriber gain_sub;
    ros::Subscriber balance_ratio_sub;
    ros::Subscriber trigger_sub;
    ros::Subscriber connect_sub;
    ros::Subscriber capture_sub;
    ros::Subscriber hardware_trigger_sub;
    ros::Subscriber disconnect_sub;
    ros::Subscriber start_preview_sub;
    ros::Subscriber stop_preview_sub;

    image_transport::Publisher pub1;
    image_transport::Publisher pub2;

//callback functions

    void exposureTimeCallback(const std_msgs::String::ConstPtr& msg);
    void exposureAutoCallback(const std_msgs::String::ConstPtr& msg);
    void triggerModeCallback(const std_msgs::String::ConstPtr& msg);
    void triggerSourceCallback(const std_msgs::String::ConstPtr& msg);
    void triggerSelectorCallback(const std_msgs::String::ConstPtr& msg);
    void pixelFormatCallback(const std_msgs::String::ConstPtr& msg);
    void gainAutoCallback(const std_msgs::String::ConstPtr& msg);
    void balanceWhiteAutoCallback(const std_msgs::String::ConstPtr& msg);
    void gainCallback(const std_msgs::String::ConstPtr& msg);
    void balanceRatioCallback(const std_msgs::String::ConstPtr& msg);
    void triggerCallback(const std_msgs::String::ConstPtr& msg);
    void connectCallback(const std_msgs::String::ConstPtr& msg);
    void captureCallback(const std_msgs::String::ConstPtr& msg);
    void hardwareTriggerCallback(const std_msgs::String::ConstPtr& msg);
    void disconnectCallback(const std_msgs::String::ConstPtr& msg);
    void startPreviewCallback(const std_msgs::String::ConstPtr& msg);
    void stopPreviewCallback(const std_msgs::String::ConstPtr& msg);

    void initParamMap(); 
    void listen_param();

    void preview();
    void createDayFolder();

    string CurrentDayFolder;

    string getCurrentDateStr();

};
