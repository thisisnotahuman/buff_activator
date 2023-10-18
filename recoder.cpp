 //
 // Created by castle on 2020/1/14.
 //

 #include "tdtcamera.h"
 #include "video_recoder.h"
#include<signal.h>
void ProgEnd(int signal)
{
    tdtlog::VideoRecoder::Release();
    exit(0);
}
 int main()
 {
 	tdtlog::VideoRecoder::Init("../../video");

 	tdtcamera::HikvisionCam camera("../../config/robot_param.yaml");
 //	tdtcamera::UVCCam camera("../../config/camera_param.yaml");
 //	tdtcamera::VideoDebug camera("../../config/camera_param.yaml");

 	tdtcamera::TImage frame;

 	int key = 0, last_key = 0;
 	while(true)
 	{
 		camera.GetImage(frame);
 		std::cout<< frame.cvimage_.size()<<std::endl;
         if(frame.cvimage_.empty()){
             continue;
         }
 		std::cout<<"pass"<<std::endl;
 //		if(frame.cvimage_.empty()) continue;
 		cv::imshow("video", frame.cvimage_);
 		key = cv::waitKey(10);
 		if(key == -1)
 		{
 			key = last_key;
 		}
 		if(key == 's')
 		{
 			if(last_key != key)
 			{
 				std::cout << "开始录像" << std::endl;
 			}
 			tdtlog::VideoRecoder::Recorder(frame.cvimage_);
 		}
 		else if(key == 'q')
 		{
 			std::cout << "退出" << std::endl;
 			break;
 		}
 		else
 		{
 			if(last_key != key)
 			{
 				tdtlog::VideoRecoder::Init("../../video");
 				std::cout << "停止录像" << std::endl;
 			}
 		}
 		last_key = key;
#ifdef RECORDER
        signal(SIGINT,ProgEnd);
        signal(SIGHUP,ProgEnd);
#endif
 	}
 	return 0;
 }