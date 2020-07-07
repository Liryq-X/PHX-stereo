#pragma once

#include "singleCamera.h"

//#include <Spinnaker.h>
//#include <SpinGenApi/SpinnakerGenApi.h>
#include "jetsonGPIO.h"
#include "GenTL.h"
#ifdef __linux__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#endif

#include "GenICam.h"

#ifdef __linux__
#pragma GCC diagnostic pop
#endif
// clang-format off
#include "ArenaApi.h"
#include "SaveApi.h"
#include "opencv2/opencv.hpp"
// clang-format on

#include <unistd.h>
#include <math.h>


#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <map>
#include <thread>
#include <mutex>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <boost/circular_buffer.hpp>
#include <boost/algorithm/string.hpp>

//for getlocaltime
#include <stdio.h>
#include <sys/timeb.h>
#include <time.h>


//using namespace Spinnaker;
//using namespace Spinnaker::GenApi;
//using namespace Spinnaker::GenICam;

using namespace cv;
using namespace std;
using namespace Arena;
using namespace std;
using namespace GenApi;


typedef std::chrono::time_point<std::chrono::system_clock> Stime;
namespace fs = boost::filesystem;
struct PHX_Params
{
    string triggerMode = "On";      // Use trigger to take images, if off, continuous taking
    string triggerSource = "Line0";

    string pixelFormat = "BayerRG8";
    
    string exposureAuto = "Off";
    string balanceWhiteAuto = "Off";
    string gainAuto = "Off";

    float exposureTime = 10000.0;      // Unit us
    float gain = 5.0;
    
    // default settings:
    string acquisitionMode = "Continuous";  // Do Not Use "single frame". That means after start acquisition and take 1 picture, it will stop
    int acquisitionBurstFrameCount = 1;
    int triggerDelay = 14;  // in us
    
    
    bool acquisitionFrameRateEnable = true; // Auto set acquisition Frame Rate to 170hz, max 60 hz transfer
    float acquisitionFrameRate = 30;    // fps
};

struct ImagePac
{
    int image_id;

    string time_stamp;
    
    cv::Mat image;

};


class PHX_camera: public singleCamera
{
public:
    PHX_camera(){};
	PHX_camera(IDevice* P_cam);
	~PHX_camera();

	int cam_connect();              // Connect to the camera

    int set_params(map<string, string> params); // Read camera parameters

	int cam_init();                 // Set parameters to camera, (re)start worker thread. 

	int set_shutter(float shutter); 
	int set_max_packet_size(int mp); 
    int set_exposureAuto(string s); //Continous Off
    int set_triggerMode(string s);
    int set_triggerActivation(string s);
    int set_triggerSource(string s);
    int set_triggerSelector(string s);
    int set_pixelFormat(string s);
    int set_gainAuto(string s);
    int set_balanceWhiteAuto(string s);
    int set_gain(float s);
    int set_balance_ratio(float b);


    int cam_grab(cv::Mat& cv_img);       // After filling trigger, grab the image to a cv Mat, called by upper level/other class
	
    bool cam_trigger_ready();               // Check if the camera trigger is ready
	void cam_trigger(string& trigger_time);  // Trigger the camera, depend on the triggering type, file software trigger or not. Enable the listenning thread to receive one frame.
	int cam_disconnect();

    int cam_get_img(cv::Mat& mat_out);

    bool exceptionThrown = false;

    // Camera parameters
	PHX_Params      camParas;

    // Camera control/connection related
    IDevice*        pCam;
    INodeMap*       pNodeMap;       // Parameters map
    INodeMap*       pDevNodeMap;    // Containing the serial number, for telling left or right

    // Working mode. Activate: Triggered by this module, save timestamps. Passive: Triggered by other modules, no need to save timestamps.
    bool    is_active_mode_ = true;


//for get local time
    string CurrentOutputFolder = "Images/";

    string getCurrentDateStr();
    string  getCurrentTimeStr();

    struct  tm      *ptm;
    struct  timeb   stTimeb;
    static  char    szTime[30];

    bool if_preview = false;

    bool debug_mode = false;

    ofstream outFile;

    cv::Mat         publishImage;

    void getNodeMesg(); //for test

    //for test

    ISystem* systemPHX;
    void setSystem(ISystem* sys);

    void initializeId();

    void    showImage();

	int get_private_save_id();

	void set_private_save_id(int new_id);

private:
    // Worker thread related
    // Worker thread entry function
    void    start_streaming();
    // Worker thread main function
    void    streaming();

    // Image saving thread related
    void    start_saving();
    void    saving();

    IImage*        pImage;           // The image type supported by Spinnaker, used by the worker thread
    unsigned char* inputBufferPtr;

    // Sharing related
    cv::Mat         cvImage;         // The cv storage of the image, for sharing
    cv::Mat         bayer_image;
   // std::deque<std::pair<int, cv::Mat>> img_buffer_;     // Buffer of the images to save, paired with ms timestamp

    std::deque<ImagePac> img_buffer_;

    // Node functions
    int set_node_val(INodeMap* p_node_map, string node_name, string value);
    int set_node_val(INodeMap* p_node_map, string node_name, int value);
    int set_node_val(INodeMap* p_node_map, string node_name, float value);
    template<typename T>
        int get_node_val(INodeMap* p_node_map, string node_name, T& value);

    int private_save_id = 0;
};


typedef boost::shared_ptr<PHX_camera> ptrCamera;
typedef boost::circular_buffer<string> tsBuffer;


#include "jetsonGPIO.h"

class PHXMulti
{
public:
    PHXMulti();
    ~PHXMulti();

    // Connect a new camera to organize together, with its parameters
    void PHX_new(ptrCamera pcamera, std::map<string, string>& paras_cams, int id);

    
    int PHXM_disconnect();

    // Syncronization
    void PHXM_sync(Stime ts);

    // Set parameters
    int PHXM_set_param(std::map<string, std::map<string, string>>& paras_cam, std::string save_path = "");

    // Set exposure (shutter)
    int PHXM_set_exposureAuto(string s);
    int PHXM_set_exposureTime(int t_us);
    int PHXM_set_maxPacketSize(int mp);
    int PHXM_set_triggerMode(string s);
    int PHXM_set_triggerActivation(string s);
    int PHXM_set_triggerSource(string s);
    int PHXM_set_triggerSelector(string s);
    int PHXM_set_pixelFormat(string s);
    int PHXM_set_gainAuto(string s);
    int PHXM_set_balanceWhiteAuto(string s);
    int PHXM_set_gain(float s);
    int PHXM_set_balanceRatio(float b);
  

    // Start capturing
    int PHXM_start_capturing();

    // Trigger camera
    int PHXM_trigger(string& ts);

    // Auto trigger
   // int PHXM_auto_trigger(int frame, float freq = 12.0f);
    int PHXM_stop_trigger();

    // Get preview images
    int PHXM_get_previews(std::vector<cv::Mat>& vec_imgs);

    // All the cameras
    std::vector<boost::shared_ptr<PHX_camera>> cameras_;
    // Number of cameras
    int nCams_ = 0;
    // Current exposure(shutter), if < 0, not read
    int exposure_us_ = -1;
    // "Soft", "GPIO" or "Client", "Soft" needs to software trigger at least one camera. "GPIO": trigger all cameras through GPIO. "Client": cameras are all triggered by other modules/computers 
    string  moduleMode_ = "GPIO";  

    int common_save_id = 0;

    bool    cont_triggering_ = false;   

    ISystem* system;

    void setSystem(ISystem* sys);
    void getMultiNodeMesg(); //for test

    void initializeIds();

    string PHXM_getCurrentTimeStr();

    bool PHX_if_preview = false; //both cameras preview
private:
    tsBuffer ts_buffer_;

    // The worker thread for triggering cameras without blocking
    std::thread thread_trigger_;   
    
};
