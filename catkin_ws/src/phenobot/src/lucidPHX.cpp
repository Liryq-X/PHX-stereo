#include <sstream>
#include <chrono>
#include "lucidPHX.h"
#include <opencv2/imgproc.hpp>
//#include <opencv2/imgcodecs.hpp>
#define DEBUG_INFO 1
#define TIMEOUT 2000
// ************************* Lucid Pheonix3.2MP single camera control ***************************/

#define GPIO_PORT gpio388
//#define GPIO_PORT gpio397

PHX_camera::PHX_camera(IDevice* p_cam):
    pCam(p_cam){
    
}

PHX_camera::~PHX_camera()
{
    if(_syncSave.csv_writter.is_open())
        _syncSave.csv_writter.close();

    cout<<"destructer\n";
    cam_disconnect();
}

int PHX_camera::cam_connect()
{
    int result = 0;
    try
    {


       // INodeMap & nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
        // Retrieve TL device nodemap and print device information
        //pDevNodeMap = &(pCam->GetTLDeviceNodeMap());
        pDevNodeMap = pCam->GetTLDeviceNodeMap();

        CStringPtr pDevIdNode = pDevNodeMap->GetNode("DeviceID");
        
        cameraID = pDevIdNode->GetValue();

        // Initialize camera with sdk
        pCam->InitializeEvents();
 
        // Retrieve GenICam nodemap
        pNodeMap = pCam->GetNodeMap();
        }
        catch (GenICam::GenericException& ge)
        {
                std::cout << "\nGenICam exception thrown: " << ge.what() << "\n";
                exceptionThrown = true;
        }
        catch (std::exception& ex)
        {
                std::cout << "\nStandard exception thrown: " << ex.what() << "\n";
                exceptionThrown = true;
        }
        catch (...)
        {
                std::cout << "\nUnexpected exception thrown\n";
                exceptionThrown = true;
        }
    isConnected = true;
    return result;
}
int PHX_camera::set_params(map<string, string> params)
{
    if(params.find("TriggerMode") != params.end())
        camParas.triggerMode = params["TriggerMode"];
    if(params.find("TriggerSource") != params.end())
        camParas.triggerSource = params["TriggerSource"];

    if(params.find("PixelFormat") != params.end())
        camParas.pixelFormat = params["PixelFormat"];

    if(params.find("ExposureAuto") != params.end())
        camParas.exposureAuto = params["ExposureAuto"];
    if(params.find("GainAuto") != params.end())
        camParas.gainAuto = params["GainAuto"]; 
    if(params.find("BalanceWhiteAuto") != params.end())
        camParas.balanceWhiteAuto = params["BalanceWhiteAuto"];

    if(params.find("ExposureTime") != params.end())
        camParas.exposureTime = stof(params["ExposureTime"]);

    if(params.find("Gain") != params.end())
        camParas.gain = stof(params["Gain"]);

    // If in streaming. If not streaming, no do start it.
    if(_streaming)
    {
        // First stop streaming, and EndAcquisition, then restart
        try{
            _streaming = false;
	        _threadWorker.join();
            cam_init();
        }
        catch (GenICam::GenericException &e)
        {
            cout << "Error: " << e.what() << endl;
            return -1;
        }
    }

    return 0;
}

int PHX_camera::set_shutter(float shutter)
{
    try
    {
        if(set_node_val(pNodeMap, "ExposureTime", shutter)!=0)
        {
            std::cout<<"Set ExposureTime failed"<<std::endl;
            return -2;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}

int PHX_camera::set_max_packet_size(int mp)
{
    try
    {
        if(set_node_val(pNodeMap, "DeviceStreamChannelPacketSize", mp)!=0)
        {
            std::cout<<"Set max packet size failed"<<std::endl;
            return -2;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}




int PHX_camera::set_exposureAuto(string s){

    try
    {
        if(set_node_val(pNodeMap, "ExposureAuto", s)!=0)
        {
            std::cout<<"Set ExposureAuto failed"<<std::endl;
            return -2;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}

int PHX_camera::set_triggerMode(string s){
    try
    {
        if(set_node_val(pNodeMap, "TriggerMode", s)!=0)
        {
            std::cout<<"Set TriggerMode failed"<<std::endl;
            return -2;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}
int PHX_camera::set_triggerActivation(string s){
    try
    {
        if(set_node_val(pNodeMap, "TriggerActivation", s)!=0)
        {
            std::cout<<"Set TriggerActivation failed"<<std::endl;
            return -2;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}
int PHX_camera::set_triggerSource(string s){
    try
    {

        if(set_node_val(pNodeMap, "TriggerSource", s)!=0)
        {
            std::cout<<"Set TriggerSource failed"<<std::endl;
            return -2;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}

int PHX_camera::set_triggerSelector(string s){
    try
    {
        if(set_node_val(pNodeMap, "TriggerSelector", s)!=0)
        {
            std::cout<<"Set TriggerSelector failed"<<std::endl;
            return -2;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}


int PHX_camera::set_pixelFormat(string s){
    try
    {
        if(set_node_val(pNodeMap, "PixelFormat", s)!=0)
        {
            std::cout<<"Set PixelFormat failed"<<std::endl;
            return -2;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}

int PHX_camera::set_gainAuto(string s){
    try
    {
        if(set_node_val(pNodeMap, "GainAuto", s)!=0)
        {
            std::cout<<"Set GainAuto failed"<<std::endl;
            return -2;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}

int PHX_camera::set_balanceWhiteAuto(string s){
    try
    {
        if(set_node_val(pNodeMap, "BalanceWhiteAuto", s)!=0)
        {
            std::cout<<"Set BalanceWhiteAuto failed"<<std::endl;
            return -2;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}

int PHX_camera::set_gain(float s){
    try
    {
        if(set_node_val(pNodeMap, "Gain", s)!=0)
        {
            std::cout<<"Set Gain failed"<<std::endl;
            return -2;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}

int PHX_camera::set_balance_ratio(float b){
    try
    {

		if(Arena::GetNodeValue<GenICam::gcstring>(pCam->GetNodeMap(), "BalanceWhiteAuto")!="Off")
		{
			cout<<"balance white is continous \n";
            std::cout<<"Set BalanceRatio failed"<<std::endl;
            return -2;
		}

        if(set_node_val(pNodeMap, "BalanceRatio", b)!=0)
        {
            std::cout<<"Set BalanceRatio failed"<<std::endl;
            return -2;
        }
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }
    return 0;
}


// Set parameters to camera, (re)start worker thread. 
int PHX_camera::cam_init()
{
    int result = 0;

    try
    {

        start_streaming();

        start_saving();

    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }

    return result;
}

bool PHX_camera::cam_trigger_ready()
{
    return (!isTriggered) && _streaming;
}

void PHX_camera::cam_trigger(string& trigger_time)
{

    if(debug_mode){

        cout<< "[" << cameraID << "] "<<"get node values\n";

        getNodeMesg();
    }

    try{
        if(debug_mode)
            cout<< "[" << cameraID << "] "<<"inside cam_trigger\n";

        ptsTrigger = &trigger_time;
    

        isTriggered = true;

		//private_save_id = common_save_id;

}
        catch (GenICam::GenericException &e)
        {
            cout<<"caught exception, stop stream\n";
            pCam->StopStream();
            cout << "[" << cameraID << "] " << "Error: " << e.what() << endl;

        }

}

int PHX_camera::get_private_save_id()
{
	return private_save_id;
}

void PHX_camera::set_private_save_id(int new_id)
{
	private_save_id = new_id;

	//if(debug_mode)
		cout << "[" << cameraID << "] set private id as "<<new_id<<"........................\n";
}

void PHX_camera::getNodeMesg(){
        cout<<"TriggerSelector: "<<Arena::GetNodeValue<GenICam::gcstring>(pCam->GetNodeMap(), "TriggerSelector")<<endl;
        cout<<"TriggerSource: "<<Arena::GetNodeValue<GenICam::gcstring>(pCam->GetNodeMap(), "TriggerSource")<<endl;
        cout<<"TriggerMode: "<<Arena::GetNodeValue<GenICam::gcstring>(pCam->GetNodeMap(), "TriggerMode")<<endl;
        cout<<"LineSelector: "<<Arena::GetNodeValue<GenICam::gcstring>(pCam->GetNodeMap(), "LineSelector")<<endl;
        cout<<"LineMode: "<<Arena::GetNodeValue<GenICam::gcstring>(pCam->GetNodeMap(), "LineMode")<<endl;
        cout<<"AcquisitionMode: "<<Arena::GetNodeValue<GenICam::gcstring>(pCam->GetNodeMap(), "AcquisitionMode")<<endl;
        cout<<"TriggerActivation: "<<Arena::GetNodeValue<GenICam::gcstring>(pCam->GetNodeMap(), "TriggerActivation")<<endl;
}

int PHX_camera::cam_disconnect()
{
    std::clog<<"["<<cameraID<<"] Camera disconnecting "<<std::endl;
    
    if(_streaming){
        try{
            _streaming = false;
	        _threadWorker.join();
        }
        catch (GenICam::GenericException &e)
        {
            cout << "Error: " << e.what() << endl;
            return -1;
        }
    }
    if(is_saving_){
        try{
            is_saving_ = false;
	        thread_saver_.join();
        }
        catch (GenICam::GenericException &e)
        {
            cout << "Error: " << e.what() << endl;
            return -1;
        }
    }

    isConnected = false;

    //set node to acquisition start
    //cout<< "[" << cameraID << "] "<<"in disconnect, set trigger selector\n";
    //Arena::SetNodeValue<GenICam::gcstring>(pCam->GetNodeMap(), "TriggerSelector", "AcquisitionStart");


    cout<< "[" << cameraID << "] "<<"in disconnect, DeinitializeEvents\n";
    if(pCam!=0)
        pCam->DeinitializeEvents(); //Deinit

    std::clog<<"["<<cameraID<<"] Camera disconnected "<<std::endl;
	return 0;
}

int PHX_camera::cam_get_img(cv::Mat& mat_out)
{
    try{
        _updateMutex.lock();
        mat_out = cvImage.clone();
        _updateMutex.unlock();
    }
    catch(std::exception& e)
    {
        std::cout<<"Get image error: "<<e.what()<<std::endl;
        return -1;
    }
    return 0;
}

// Streaming

void PHX_camera::start_streaming()
{

    // Start worker process to receive images
    
	_threadWorker = std::thread(&PHX_camera::streaming, this);
    cout << "[" << cameraID << "] " << "Started acquiring images..." << endl;
}

void PHX_camera::initializeId()
{
   //initialize save id
    cout<<"initialize the private_save_id\n";

    CurrentOutputFolder = "/home/nvidia/catkin_ws/Images/" + getCurrentDateStr()+"/" + cameraID;

    ifstream input(CurrentOutputFolder + "/timeStamp.csv");

    if(input.is_open()){

        cout<<"open the input file\n";

        int line_num = 0;

        string last_line;

        for(string line; getline(input, line); line_num++)
            last_line = line;

        //cout<<"last_line: "<<last_line<<endl;

        if(line_num > 0){

            vector<string> vec;

            boost::split(vec, last_line, boost::is_any_of(","));
            for(auto&i:vec)
                cout<<i<<endl;

            if(vec.size()>0)
                private_save_id = stoi(vec[0]) + 1;

            cout<<"private_save_id: "<< private_save_id <<endl;
        }

    }
    
    input.close();

    std::cout<<"["<<cameraID<<"] "<<" private_save_id "<< private_save_id <<std::endl;
}


void PHX_camera::streaming()
{

    //create folder
      std::cout<<"["<<cameraID<<"] "<<"create folder"<<std::endl;

      CurrentOutputFolder = "/home/nvidia/catkin_ws/Images/" + getCurrentDateStr()+"/" + cameraID;

      const char* path = CurrentOutputFolder.c_str();

      boost::filesystem::path dir(path);

      if(boost::filesystem::create_directory(dir)){

         cout<<"Directory created: "<< CurrentOutputFolder<<endl;
      }

 
    cout << "[" << cameraID << "] " <<"streaming\n";

    // Start camera streaming, if using continues mode
    pCam->StartStream(); 
    // image id counter reset
    //imgId = 0;

    _streaming = true;
    // Worker thread main loop
    while((_streaming && isConnected))
    {
        if(!isTriggered){
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            continue;
        }

        try
        {
            // Retrieve next received image and ensure image completion
            //imgId ++; 
            
            pImage = pCam->GetImage(TIMEOUT); //GetNextImage(); //TIMEOUT
            
            // Check the triggerMode and is module active
            // If passive mode, or trigger Off, save all image with received time as timestamp
            // If active mode, and trigger On, save the triggered image, and trigger timestamp

            string triggered_time;
            if(camParas.triggerMode == "Off") {
               // triggered_time = std::chrono::system_clock::now();
            }
            else{
                triggered_time = *ptsTrigger;
                isTriggered = false;
            }

            if (pImage->IsIncomplete())
            {
                if(!if_preview)
                    cout << "[" << cameraID << "] " << "Image incomplete ..." << endl << endl;
				//private_save_id++;
                //_syncSave.save_id++; //miss one image, but it's id keep consistent with the other
            }
            else
            {
                if(debug_mode)
                    cout<< "[" << cameraID << "] " <<"convert image \n";

                IImage* convertedImage = pImage;//pImage->Convert(PixelFormat_BayerRG8, HQ_LINEAR);
                // Save images to buffer                
                unsigned int XPadding = convertedImage->GetPaddingX();
                unsigned int YPadding = convertedImage->GetPaddingY();
                unsigned int rowsize = convertedImage->GetWidth();
                unsigned int colsize = convertedImage->GetHeight();

                // image data contains padding. When allocating Mat container size, you need to account for the X,Y image data padding.
                // Copy data to shared buffer
                _updateMutex.lock();
                inputBufferPtr = const_cast<uint8_t*>(convertedImage->GetData());

                cvImage = cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC1, (void*)inputBufferPtr).clone(); //, convertedImage->GetStride()
                _updateMutex.unlock();

                // Save the image, with filename&timestamp
                if(debug_mode)
                    cout<< "[" << cameraID << "] " <<"save image\n";

                if(1){ //autoSave
                    // Put the image into a saving buffer, leave the rest of the saving work to a saving thread
                    _updateMutex.lock();
/*
                    if(img_buffer_.size()<100)
                        img_buffer_.push_back(std::pair<int, cv::Mat>(
                                _syncSave.calc_stime_ms_diff(triggered_time),
                                cvImage.clone()));*/


                    ImagePac ip;
                    ip.image_id = private_save_id;
                    ip.time_stamp = triggered_time;
                    ip.image = cvImage.clone();

                    if(img_buffer_.size()<100)
                        img_buffer_.push_back(ip);

                    _updateMutex.unlock();




                }
            }
 
            // Release image
            pCam->RequeueBuffer(pImage);
            //pImage->Release();      

        }
        catch (GenICam::GenericException &e)
        {
            //cout<< "[" << cameraID << "] "<<"in streaming, set trigger selector\n";
            //Arena::SetNodeValue<GenICam::gcstring>(pCam->GetNodeMap(), "TriggerSelector", "AcquisitionStart");


            cout<<"[" << cameraID << "] " << "Stoped streaming"<<std::endl;
            pCam->StopStream(); //EndAcquisition();
            cout << "[" << cameraID << "] " << "Error: " << e.what() << endl;
        }
        
    }
    cout<<"[" << cameraID << "] " << "Stoped streaming loop"<<std::endl;
    pCam->StopStream(); //EndAcquisition();
    cout<<"[" << cameraID << "] " << "Stoped streaming"<<std::endl;
}


void PHX_camera::showImage(){ //for preview

/*
        gpioSetValue(GPIO_PORT, off); 

        std::this_thread::sleep_for(std::chrono::milliseconds(3));

        gpioSetValue(GPIO_PORT, on);
*/
        pImage = pCam->GetImage(TIMEOUT); 

        inputBufferPtr = const_cast<uint8_t*>(pImage->GetData());

        cvImage = cv::Mat(pImage->GetHeight(), pImage->GetWidth(), CV_8UC1, (void*)inputBufferPtr);

		cvtColor(cvImage, publishImage, CV_BayerRG2RGB);

        cv::resize(publishImage, publishImage, cv::Size(), 0.25, 0.25);

		pCam->RequeueBuffer(pImage);

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// Saving threads related

void PHX_camera::start_saving()
{ 
    // Start worker process to receive images
    
	thread_saver_ = std::thread(&PHX_camera::saving, this);
}

void PHX_camera::saving()
{

    cout << "[" << cameraID << "] " << "Started saving thread." << endl;
    is_saving_ = true;

    vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compression_params.push_back(85);

    while(is_saving_ && isConnected)
    {
        while(!img_buffer_.empty())
        {
            ImagePac ts_img = img_buffer_.front();


            // Create a unique filename
            std::stringstream ss_fn; // A string stream to generate filename, with format

            ss_fn<<std::setw(5)<<std::setfill('0')<<ts_img.image_id<<"_"<<cameraID<<".pgm";
            //fs::path img_path = fs::path(_syncSave.save_path) / fs::path(ss_fn.str());
		
            fs::path img_path = fs::path(CurrentOutputFolder) / fs::path(ss_fn.str());

            // Print image information
            if(!if_preview)
                cout << "[" << cameraID << "] "<< ". Image saved at " << img_path.c_str()<< endl;

			//only for capturing calibration image
/*
                    cvtColor(ts_img.image, publishImage, CV_BayerRG2RGB);

           		 	std::stringstream ss_fn_calib; // A string stream to generate filename, with format

					if(cameraID=="191400039")
            			ss_fn_calib<<"left"<<ts_img.image_id<<".jpg";
					else
            			ss_fn_calib<<"right"<<ts_img.image_id<<".jpg";

            		fs::path img_path_calib = fs::path(CurrentOutputFolder) / fs::path(ss_fn_calib.str());

                	cv::imwrite(img_path_calib.c_str(), publishImage);

*/
            // Save image
            _updateMutex.lock();      

                //convert image to be published

                if(ts_img.image_id % 3 ==0 ){ //publish an image for preview every 2s

                    cvtColor(ts_img.image, publishImage, CV_BayerRG2RGB);

           		 	std::stringstream ss_fn_preview; // A string stream to generate filename, with format

            		ss_fn_preview<<std::setw(6)<<std::setfill('0')<<ts_img.image_id<<"_"<<cameraID<<".jpg";

            		fs::path img_path_preview = fs::path(CurrentOutputFolder) / fs::path(ss_fn_preview.str());

                	cv::imwrite(img_path_preview.c_str(), publishImage);

                    cv::resize(publishImage, publishImage, cv::Size(), 0.25, 0.25);

                }


                if(debug_mode)
                    cout << "[" << cameraID << "] " <<"write images\n";
                cv::imwrite(img_path.c_str(), ts_img.image);


                img_buffer_.pop_front();
                _updateMutex.unlock();

                // Log file with timestamp. 
                if(_syncSave.csv_writter.is_open())
                    _syncSave.csv_writter<<
                        ts_img.image_id
                        <<","<<img_path.c_str()<<std::endl;

                outFile.open(CurrentOutputFolder+"/timeStamp.csv",ios::app);
                outFile <<ts_img.image_id <<","<< ts_img.time_stamp<<"\n";  //getCurrentTimeStr()
                outFile.close();

        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    img_buffer_.clear();
    cout << "[" << cameraID << "] " <<"Saving thread stoped"<<std::endl;
}

// Nodes

//trigger mode
int PHX_camera::set_node_val(INodeMap* p_node_map, string node_name, string value)
{
    int result = 0;

    try{

        Arena::SetNodeValue<GenICam::gcstring>(pCam->GetNodeMap(), node_name.c_str(), value.c_str());
    

        GenICam::gcstring after = Arena::GetNodeValue<GenICam::gcstring>(pCam->GetNodeMap(), node_name.c_str());

        cout<< "[" << cameraID << "] "<<"set "<<node_name << " as " << after<<endl;
    }

    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
    return result;
}
int PHX_camera::set_node_val(INodeMap* p_node_map, string node_name, int value)
{
    int result = 0;
    try{
        CValuePtr p_val = p_node_map->GetNode(node_name.c_str());
        if (!p_val || !IsWritable(p_val))
        {
            cout << "Unable set int Node "<<node_name<<". Aborting..." << endl << endl;
            return -1;
        }
        ((CIntegerPtr)p_val)->SetValue(value);
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
    return result;
}

//gain exposure done
int PHX_camera::set_node_val(INodeMap* p_node_map, string node_name, float value)
{
    int result = 0;

    try{
        GenApi::CFloatPtr pGainValue = pCam->GetNodeMap()->GetNode(node_name.c_str());

        if (!pGainValue)
        {
                throw GenICam::GenericException("ExposureTime node not found", __FILE__, __LINE__);
        }

        if (!GenApi::IsWritable(pGainValue))
        {
                throw GenICam::GenericException("ExposureTime node not writable", __FILE__, __LINE__);
        }


       if (value < pGainValue->GetMin())
        {
                value = pGainValue->GetMin();
        }

        if (value > pGainValue->GetMax())
        {
                value = pGainValue->GetMax();
        }

       std::cout  << "[" << cameraID << "] "<< "Set " << node_name << " as " <<value << " " << pGainValue->GetUnit() << "\n";

       	pGainValue->SetValue(value);
    } //try

    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
    return result;
}
template<typename T>
int PHX_camera::get_node_val(INodeMap* p_node_map, string node_name, T& value)
{
    int result = 0;
    try{
        CValuePtr p_val = p_node_map->GetNode(node_name.c_str());
        if (!p_val || !IsReadable(p_val))
        {
            cout << "Unable get Node "<<node_name<<". Aborting..." << endl << endl;
            return -1;
        }
        // String type
        if(is_same<T, string>::value){   
            value = ((CStringPtr)p_val)->GetValue();
        }
        // Int type
        else if(is_same<T, int>::value){
            value = ((CIntegerPtr)p_val)->GetValue();
        }
        // Float type
        else if(is_same<T, float>::value || is_same<T, double>::value)
        {
            value = ((CFloatPtr)p_val)->GetValue();
        }
        else{
            std::cout<<"Set value datatype not implemented"<<std::endl;
            return -1;
        }
        
    }
    catch (GenICam::GenericException &e)
    {
        cout << "Error: " << e.what() << endl;
        result = -1;
    }
    return result;
}

char PHX_camera::szTime[] = "abc";

string PHX_camera::getCurrentDateStr(){

        
        ftime(&stTimeb);
        ptm = localtime(&stTimeb.time);
        sprintf(szTime, "%04d-%02d-%02d",
                ptm->tm_year + 1900, ptm->tm_mon+1, ptm->tm_mday);

        string out_str(szTime);

        return out_str;
}


string PHX_camera::getCurrentTimeStr(){

        ftime(&stTimeb);
        ptm = localtime(&stTimeb.time);
        sprintf(szTime, "%04d-%02d-%02d-%02d-%02d-%02d-%03d",
                ptm->tm_year + 1900, ptm->tm_mon+1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min, ptm->tm_sec, stTimeb.millitm);

        string out_str(szTime);

        return out_str;

}

void PHX_camera::setSystem(ISystem* sys){
    
    systemPHX = sys;

}

// ************************* Lucid Pheonix3.2MP multiple camera control ***************************/

PHXMulti::PHXMulti()
{
    ts_buffer_.resize(15);
    gpioExport(GPIO_PORT);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    gpioSetDirection(GPIO_PORT, outputPin);
    //std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

PHXMulti::~PHXMulti()
{
    
    int res = PHXM_disconnect();
    if(res!=0)
        std::cout<<"Disconnect error: "<< -res << std::endl;
}

void PHXMulti::setSystem(ISystem* sys){
    
    system = sys;

}

// A new camera to organize together, with its parameters
void PHXMulti::PHX_new(ptrCamera pcamera, std::map<string, string>& paras_cam, int id)
{
    //cameras_.push_back(pcamera);
    if(cameras_.size() <=id)
        cameras_.resize(id+1);

    cameras_[id] = pcamera;
    pcamera->set_params(paras_cam);
    nCams_ += 1;
}

int PHXMulti::PHXM_disconnect()
{
    // Stop triggering thread
    cont_triggering_ = false;
    if(thread_trigger_.joinable())
        thread_trigger_.join();

    // Stop & release cameras
    for(ptrCamera pCamera : cameras_)
    {
        //pCamera->setSystem(system);  

        int res = pCamera->cam_disconnect();

        cout<<"before DestroyDevice\n";
        system->DestroyDevice(pCamera->pCam);
        cout<<"after \n";
        pCamera->pCam = 0;
        if(res!=0)
            return -(std::stoi(pCamera->cameraID));


    }
    cameras_.clear();
    nCams_ = 0;
    
    // Clear buffers
    ts_buffer_.clear();
    std::clog<<"All disconnected "<<std::endl;
    return 0;
}

// Syncronization
void PHXMulti::PHXM_sync(Stime ts)
{
    for(ptrCamera pCamera : cameras_)
    {
        std::cout<<"["<<pCamera->cameraID<<"] "<<"Sync"<<std::endl;
        pCamera->setup_sync(ts);
    }
}         

// Set parameters
int PHXMulti::PHXM_set_param(std::map<string, std::map<string, string>>& paras_cams, std::string save_path){
    for(ptrCamera pCamera : cameras_)
    {
        //std::cout<<"["<<pCamera->cameraID<<"] "<<"Paras"<<std::endl;
        int res = pCamera->set_params(paras_cams[pCamera->cameraID]);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
        //std::cout<<"["<<pCamera->cameraID<<"] "<<"Save"<<std::endl;
        pCamera->setup_save(true, pCamera->cameraID, save_path);

        // Working mode, "Active", "Passive"
        if(moduleMode_ == "Client")
            pCamera->is_active_mode_ = false;
        else
            pCamera->is_active_mode_ = true;
    }
    return 0;
}

// Set exposure (shutter)
int PHXMulti::PHXM_set_exposureAuto(string s){
    std::cout<<"Set exposureAuto to: "<< s << std::endl;
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->set_exposureAuto(s);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}

int PHXMulti::PHXM_set_exposureTime(int t_us){
    std::cout<<"Set exposureTime to: "<<t_us<<" us" << std::endl;
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->set_shutter(t_us);
        //pCamera->set_shutter(t_us);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}


int PHXMulti::PHXM_set_maxPacketSize(int mp){
    std::cout<<"PHXM, Set max packet size to: "<<mp<<" B" << std::endl;
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->set_max_packet_size(mp);

        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}


int PHXMulti::PHXM_set_triggerMode(string s){
    std::cout<<"Set triggerMode to: "<< s << std::endl;
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->set_triggerMode(s);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}

int PHXMulti::PHXM_set_triggerActivation(string s){
    std::cout<<"Set triggerActivation to: "<< s << std::endl;
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->set_triggerActivation(s);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}

int PHXMulti::PHXM_set_triggerSource(string s){

    for(ptrCamera pCamera : cameras_)
    {

        int res = pCamera->set_triggerSource(s);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}

int PHXMulti::PHXM_set_triggerSelector(string s){
    std::cout<<"Set triggerSelector to: "<< s << std::endl;
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->set_triggerSelector(s);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}

int PHXMulti::PHXM_set_pixelFormat(string s){
    std::cout<<"Set pixelFormat to: "<< s << std::endl;
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->set_pixelFormat(s);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}
int PHXMulti::PHXM_set_gainAuto(string s){
    std::cout<<"Set gainAuto to: "<< s << std::endl;
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->set_gainAuto(s);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}
int PHXMulti::PHXM_set_balanceWhiteAuto(string s){
    std::cout<<"Set balanceWhiteAuto to: "<< s << std::endl;
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->set_balanceWhiteAuto(s);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}

int PHXMulti::PHXM_set_gain(float s){
    std::cout<<"Set gain to: "<< s << std::endl;
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->set_gain(s);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}

int PHXMulti::PHXM_set_balanceRatio(float b){
    std::cout<<"Set balance ratio to: "<< b << std::endl;
    for(ptrCamera pCamera : cameras_)
    {
        int res = pCamera->set_balance_ratio(b);
        if(res != 0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}

// Start capturing
int PHXMulti::PHXM_start_capturing(){

    gpioSetValue(GPIO_PORT, on); //off
    for(ptrCamera pCamera : cameras_)
    {
        std::cout<<"["<<pCamera->cameraID<<"] "<<"Capture"<<std::endl;
        int res = pCamera->cam_init();
        if(res!=0)
            return -(std::stoi(pCamera->cameraID));
    }
    return 0;
}

void PHXMulti::getMultiNodeMesg(){
    for(ptrCamera pCamera : cameras_)
    {
        std::cout<<"["<<pCamera->cameraID<<"] "<<"get node message"<<std::endl;
        pCamera->getNodeMesg();
    }
}

// Trigger camera
int PHXMulti::PHXM_trigger(string& ts){  
    

    // wait for trigger ready

    int try_time_out = 0;
    bool trigger_ready = true;
    while(try_time_out<2000){
        trigger_ready = true;
        for(ptrCamera pCamera : cameras_)
        {
            trigger_ready = trigger_ready && pCamera->cam_trigger_ready();
        }
        if(trigger_ready)
            break;

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        try_time_out ++;
    }
    
    // Time out for triggering
    if(!trigger_ready)
        return -1;

    gpioSetValue(GPIO_PORT, off); //on

    std::this_thread::sleep_for(std::chrono::milliseconds(3));

    gpioSetValue(GPIO_PORT, on); //off


    for(ptrCamera pCamera : cameras_)
    {
        pCamera->cam_trigger(ts);

        //pCamera->ptsTrigger = &ts;
        //pCamera->isTriggered = true;
        //pCamera->private_save_id = common_save_id;
		pCamera->set_private_save_id(common_save_id);
    }

    //if(!PHX_if_preview)
        common_save_id++;

    return 0;
}

void PHXMulti::initializeIds(){  

    int max_id = 0;

    for(ptrCamera pCamera : cameras_)
    {
        pCamera->initializeId();

        
        if( pCamera->get_private_save_id() > max_id)
            max_id = pCamera->get_private_save_id();
    }

    common_save_id = max_id;

    cout<<"common_save_id: "<< common_save_id<<endl;

}

string PHXMulti::PHXM_getCurrentTimeStr(){

    for(ptrCamera pCamera : cameras_)
    {
        return pCamera->getCurrentTimeStr();

    }

    return "0";
}



int PHXMulti::PHXM_stop_trigger()
{
    std::cout<<"Stop triggering"<<std::endl;
    cont_triggering_ = false;
    if(cont_triggering_){
        if(thread_trigger_.joinable())
            thread_trigger_.join(); 
        else
            cout<<"Triggering thread not joinable"<<endl;  
    }
    return 0;
}


// Get previews
int PHXMulti::PHXM_get_previews(std::vector<cv::Mat>& vec_imgs)
{
    vec_imgs.resize(nCams_);
    
    for(int i = 0; i < nCams_; i++)
    {
        int r_tg = cameras_[i]->cam_get_img(vec_imgs[i]);
        if(r_tg!=0)
        {
            std::cout<<"Get images "<< cameras_[i]->cameraID <<" failed"<< std::endl;
            return -1;
        }
    }
    return 0;
}


