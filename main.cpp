/**
 * @Name: file name
 * @Description:
 * @Version: 1.0.0.1
 * @Author: your name
 * @Date: 2019-10-14 19:42:43
 * @LastEditors: your name
 * @LastEditTime: 2019-10-14 20:03:31
 */

#ifdef Calibrate

#undef VIDEO_DEBUG
#undef RECORDER

#endif
#include "Debug.h"
#include "ModelGenerator.h"
#include "SharedMemory.h"
#include "armor_detector.h"
#include "armor_resolver.h"
#include "buff_detector.h"
#include "buff_predictor.h"
#include "buff_resolver.h"
#include "log.h"
#include "number_detector.h"
#include "robot_decision.h"
#include "robot_predictor.h"
#include "tdtcamera.h"
#include "tdtcommon.h"
#include "timecounter.h"
#include "video_recoder.h"
#include <QApplication>
#include <TimeMatch.h>
#include <csignal>
#include <tdtcamera.h>
#include <thread>

#ifdef RECORDER
#include <signal.h>
#include <thread>
#endif
#include <opencv2/videoio.hpp>
/*
 * @note:这个函数为关机或者Ctrl+C后程序所做的操作(如果还有其他动作，就在exit前添加）
 * WARN：千万不能去掉exit，因为他本身就是SIGINT和SIGHUP的原本动作，如果去掉程序将无法关闭
 * 1.结束录制视频
 * 2.结束保存图片
 */
void ProgEnd(int signal) {
    //结束视频
#ifdef RECORDER
    tdtlog::VideoRecoder::Release();
#endif // RECORDER
    //保存图片

    exit(0);
}

using namespace tdttoolkit;
#ifdef WATCH_DOG
tdttoolkit::WatchDog watchdog = tdttoolkit::WatchDog();
#endif
[[noreturn]] void ProgOnImage() {
#ifdef VIDEO_DEBUG
    tdtcamera::Camera &camera = *(Debug::GetVirtualCamera());
#else
    tdtcamera::HikvisionCam HKcamera("../../config/robot_param.yaml");
    tdtcamera::Camera &     camera = HKcamera;
#endif
    tdtcamera::TImage frame;

    tdttoolkit::ReceiveMessage receiveMessage;
    tdtusart::timeSimulaneity  TimeSimulaneity(10, 8);

    tdtrobot::ArmorDetector  armorDetector;
    tdtrobot::ArmorResolver  armorResolver;
    tdtrobot::RobotPredictor robotPredictor;

    tdtbuff::BuffDetector              buffDetector;
    tdtbuff::BuffResolver              buffResolver(&camera);
    tdtbuff::BuffPredictor             buffPredictor;
    std::vector<tdttoolkit::BuffArmor> buff_armors;
    tdttoolkit::Buff                   buff_complete;
    tdtdecision::RobotDecision         robotDecision;

    tdttoolkit::BulletSpeedSolver bulletSpeedSolver;

    tdtusart::Send_Struct_t sendStruct;
    tdtusart::Recv_Struct_t recvStruct;
    robotPredictor.armorResolver = &armorResolver;
#ifdef VIDEO_DEBUG
    tdtusart::Usart &usart = *(Debug::GetVirtualUsart());
#else
    tdtusart::RealUsart     realUsart;
    tdtusart::Usart &       usart = realUsart;
#endif
    cv::Mat                         last_src;
    tdttoolkit::ResolvedArmor       last_resolved_armor;
    double                          last_tick = 0;
    tdt_communication::SharedMemory sharedMemory(0, sizeof(unsigned long long), tdt_communication::SharedMemory::NodeType::Server);

#ifdef RECORDER
    std::cout << "开始录像" << std::endl;
    tdtlog::VideoRecoder::Init("../../video");
#endif
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

#ifdef RADER_UPDATE
    unsigned int frame_number = 0;
#endif // RADER_UPDATE
    while (true) {

        TDT_INFO("耗时:%fms", (cv::getTickCount() - last_tick) * 1000 / cv::getTickFrequency());
        TDT_INFO("FPS:%f", cv::getTickFrequency() / (cv::getTickCount() - last_tick));
        robotPredictor.kalman_fps = cv::getTickFrequency() / (cv::getTickCount() - last_tick);
#ifdef VIDEO_DEBUG
        Debug::AddText("帧率", "fps:" + std::to_string(cv::getTickFrequency() / (cv::getTickCount() - last_tick)), cv::Point2d(1250, 30), cv::Scalar(255, 255, 255), 1, 3);
#endif
        last_tick = cv::getTickCount();
        TDT_INFO("---------------------------------------------------------------------------------------------------");

        // double t1 = cv::getTickCount();
        camera.GetImage(frame);

        // std::cout << "tGetImage = " << (cv::getTickCount() - t1) / cv::getTickFrequency() * 1000 << "ms\n";
#ifdef WATCH_DOG
        watchdog.Feed_Dog();
#endif
        if (frame.cvimage_.empty())
        {

            std::cout << "frame is empty!" << std::endl;
#ifdef VIDEO_DEBUG
            Debug::GetOrSetReceiveMessage(nullptr);
            Debug::UpdateView();
            Debug::LoopOver();
#endif
            continue;
        }
#ifdef Calibrate
        while (true) {
            camera.GetImage(frame);
            if (frame.cvimage_.empty()) {
                std::cout << "frame is empty!" << std::endl;
                continue;
            }
            int camcal = camera.CameraCalibrate(frame.cvimage_);
            if (camcal == 1)
                break;
            tdttoolkit::Time::UpdateFrameTime();
            tdtlog::Log::AllChannelLog("A new Frame.");
        }
#endif
#ifdef RECORDER

        tdtlog::VideoRecoder::img = frame.cvimage_.clone();
        std::thread th(tdtlog::VideoRecoder::Recorder_);
        if (th.joinable()) {
            th.join();
        }
//        tdtlog::VideoRecoder::Recorder(frame.cvimage_);
#endif
//        cv::imshow("1",frame.cvimage_);
//        switch (cv::waitKey(20))
//        {
//        case 27:
//            exit(0);
//
//        case 32:
//            camera.ReatartCamRunningProgram();
//            break;
//        }

        tdttoolkit::Time::UpdateFrameTime();
        Debug::AddText("时间戳", "Time:" + std::to_string(Time::GetFrameTime() / 1000000.0f), cv::Point2d(20, 30), cv::Scalar(255, 255, 255), 1, 3);
        tdtlog::Log::AllChannelLog("A new Frame.");
        double             t_sm = cv::getTickCount();
        unsigned long long defensive_camera_enemy;
        if (sharedMemory.cache() != -1) {
            sharedMemory >> defensive_camera_enemy;
            for (int i = 0; i < 2; i++) {
                if (defensive_camera_enemy & (1 << i))
                    std::cout << "[Defensive Camera]" << i << "st Camera find the enemy" << std::endl;
            }
        }
        std::cout << "Shared Memory 耗时" << (cv::getTickCount() - t_sm) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
        // ! 跑本地视频需要将下位机数据置零，上车时务必注释掉
        // receiveMessage.shoot_platform.yaw   = 0;
        // receiveMessage.shoot_platform.pitch = 0;
        // ! 跑本地视频需要将下位机数据置零，上车时务必注释掉

#ifndef VIDEO_DEBUG
        receiveMessage.mode = tdttoolkit::ArmorMode;
#endif
        Debug::GetOrSetReceiveMessage(&recvStruct);
        tdttoolkit::TransformRecv(recvStruct, receiveMessage);

        if (receiveMessage.mode == tdttoolkit::ArmorMode) {

            static auto p       = new TimeCounter("Armor Detector");
            double      t1      = cv::getTickCount();
            auto        armors  = armorDetector.Get(frame.cvimage_, receiveMessage);
            double      tDetect = (cv::getTickCount() - t1) / cv::getTickFrequency() * 1000;
            std::cout << "armorDetector.Get()耗时: " << tDetect << "ms" << std::endl;
            //
            double t2        = cv::getTickCount();
            int    usart_ret = usart.UsartRecv(recvStruct);
            std::cout << "Usart_Recv_RetCode: " << usart_ret << std::endl;
            Debug::GetOrSetReceiveMessage(&recvStruct);
            // simultaneity.calcTimeDiff(recvStruct.recieveTime, recieve_time);
            TimeSimulaneity.readData(recvStruct.data_, 120);
            tdttoolkit::TransformRecv(recvStruct, receiveMessage);
            std::cout << "GetImageTime=" << camera.GetImageTime() / 1000000 << std::endl;
            if (camera.GetImageTime() != -1) {
                int   t = TimeSimulaneity.timeMatch(camera.GetImageTime() / 1000000);
                float y, p;
                TimeSimulaneity.at(t, -2) >> y >> p;
                receiveMessage.shoot_platform = {float(p / 180 * CV_PI), float(y / 180 * CV_PI), bulletSpeedSolver.SolveSpeed(receiveMessage.bulletspeed, frame.header_.stamp_)};
            } else {
                receiveMessage.shoot_platform.bulletspeed = bulletSpeedSolver.SolveSpeed(receiveMessage.bulletspeed, frame.header_.stamp_);
            }
            std::cout << "tUsartRecv = " << (cv::getTickCount() - t2) / cv::getTickFrequency() * 1000 << "ms\n";
            //
            if (!armors.empty()) {
                double t_resolver = cv::getTickCount();
                sendStruct.no_Obj = false;

                auto resolved_armors = armorResolver.Resolve(armors, receiveMessage.shoot_platform);
                for (int i = 0; i < resolved_armors.size(); i++) {
                    if (!resolved_armors[i].GetResolvedStatus()) {
                        resolved_armors.erase(resolved_armors.begin() + i);
                    }
                }
                if (resolved_armors.empty())
                    goto NO_ARMOR;
                cv::Point2d orientedProj = armorResolver.ProjectArmor(receiveMessage.shoot_platform, resolved_armors[0]);
                Debug::AddCircle("解算点投影", orientedProj, 6, cv::Scalar(255, 255, 0));

                // double      t2      = cv::getTickCount();
                Robot robot = robotDecision.Decide(resolved_armors, receiveMessage);
                if (robot.GetRobotType() == TYPEUNKNOW)
                    goto NO_ARMOR;
                resolved_armors = robot.GetResolvedArmors();
                // double      tDetect0 = (cv::getTickCount() - t2) / cv::getTickFrequency() * 1000;
                // std::cout << "Decide耗时: " << tDetect0 << "ms" << std::endl;

                //     // -----------------------------静态打击联调--------------------------------
                //     // sendStruct.yaw = (resolved_armors[0].GetPolar().yaw)/CV_PI*180.0f;
                //     // sendStruct.pitch = (resolved_armors[0].GetPolar().pitch)/CV_PI*180*;

                //     cv::Point3f orientedPoint = MatToPoint3f(resolved_armors[0].GetPositionInWorld());

                //     float x, y;
                //     y                   = orientedPoint.y;
                //     x                   = sqrt(orientedPoint.x * orientedPoint.x + orientedPoint.z * orientedPoint.z);
                //     std::cout << receiveMessage.bulletspeed << std::endl;
                //     cv::Vec2f exitAngle = ParabolaSolve(cv::Point2f(x, y), receiveMessage.bulletspeed); //速度单位取厘米每秒
                //     float pitch    = fabs(exitAngle[0] - receiveMessage.shoot_platform.pitch) < fabs(exitAngle[1] - receiveMessage.shoot_platform.pitch) ? exitAngle[0] : exitAngle[1];

                //     sendStruct.yaw      = (resolved_armors[0].GetPolar().yaw - receiveMessage.shoot_platform.yaw - 0.01)/CV_PI*180.0f;
                //     sendStruct.pitch    = (pitch - receiveMessage.shoot_platform.pitch - 0.02)/CV_PI*180*100;

                //     std::cout << "限幅前: " << sendStruct.yaw << ", " << sendStruct.pitch << std::endl;

                //     // todo 等电控Struct.pitch类型更正为short之后，对y和p做限幅度
                //     // AngleCorrect(sendStruct.yaw);
                //     // AngleCorrect(tempPitch);

                //     std::cout << "限幅后: " << sendStruct.yaw << ", " << sendStruct.pitch << std::endl;

                //     Polar3f orientedPolar(resolved_armors[0].GetPolar().distance, resolved_armors[0].GetPolar().yaw, pitch);
                //     cv::Point3f point = PolarToRectangular(orientedPolar);
                //     cv::Point2d compProj = CalcProjectPoint(point, resolved_armors[0].GetPNPTvec(), resolved_armors[0].GetPNPRvec(), receiveMessage.shoot_platform, true);
                //     Debug::AddCircle("重力补偿投影", compProj, 6, cv::Scalar(0, 255, 255));

                //     std::cout << "-----TEST1-----: " << resolved_armors[0].GetPNPTvec() << std::endl;
                //     std::cout << "投影解算点: " << orientedProj << std::endl;
                //     std::cout << "投影重力补偿点: " << compProj << std::endl;

                //     // -----------------------------静态打击联调--------------------------------

#ifdef VIDEO_DEBUG
                if (resolved_armors.size() == 1) {
                    Debug::SetMessage("装甲板解算", "单装甲板世界直角坐标", cv::Vec3f(resolved_armors[0].GetPositionInWorld()));
                    Debug::SetMessage("装甲板解算", "单装甲板世界极坐标", cv::Vec3f(tdttoolkit::PolarToPoint(resolved_armors[0].GetPolar())));
                } else {
                    Debug::SetMessage("装甲板解算", "左装甲板世界直角坐标", cv::Vec3f(resolved_armors[0].GetPositionInWorld()));
                    Debug::SetMessage("装甲板解算", "右装甲板世界直角坐标", cv::Vec3f(resolved_armors[1].GetPositionInWorld()));
                    Debug::SetMessage("装甲板解算", "左装甲板世界极坐标", cv::Vec3f(tdttoolkit::PolarToPoint(resolved_armors[0].GetPolar())));
                    Debug::SetMessage("装甲板解算", "右装甲板世界极坐标", cv::Vec3f(tdttoolkit::PolarToPoint(resolved_armors[1].GetPolar())));
                }
#endif
                double t_predicter = cv::getTickCount();
                robotPredictor.Update(resolved_armors, camera.GetImageTime() == -1 ? tdttoolkit::Time::GetTimeNow() : camera.GetImageTime());
                static double t_predicter_time;
                if (camera.GetImageTime() == -1)
                    robotPredictor.fire_delay_ = 0.13 + t_predicter_time + sizeof(sendStruct) / TimeSimulaneity.get_bit_rate();
                else
                    robotPredictor.fire_delay_ = 0.13 + cv::getTickCount() / cv::getTickFrequency() - camera.GetImageTime() / 1e6 + t_predicter_time + sizeof(sendStruct) * 8 / TimeSimulaneity.get_bit_rate();
                std::cout << "开火延时fire_delay为" << robotPredictor.fire_delay_ << "s" << std::endl;
                robotPredictor.FireCommand(receiveMessage, sendStruct);
                t_predicter_time = (cv::getTickCount() - t_predicter) / cv::getTickFrequency();
                std::cout << "开火决策耗时" << t_predicter_time * 1000 << "ms" << std::endl;
            } else {
            NO_ARMOR:
                sendStruct.no_Obj = true;
                if (!receiveMessage.lock_command)
                    robotDecision = tdtdecision::RobotDecision();
                ///下面这个部分是防止某些情况下丢失装甲板但仍能看到灯条，此时摄像头往面积最大灯条方向转，降低丢失目标率。
                if (!armorDetector.trace_light_bars_.empty()) {
                    //armorDetector
                    //屏幕右下 yaw负pitch正
                    //yaw的2.5和pitch的2是实际上车调过的，在1～3m效果可以，后续可以再调
                    if (armorDetector.trace_light_bars_[0].GetCenter().x > 1080)     //输出图像长1440的3/4处
                        sendStruct.yaw -= 2.5;                                       // 2.5可能需要根据距离调整
                    else if (armorDetector.trace_light_bars_[0].GetCenter().x < 360) //输出图像长1440的1/4处
                        sendStruct.yaw += 2.5;
                    if (armorDetector.trace_light_bars_[0].GetCenter().y > 810)      // 810是输出图像宽1080的3/4处
                        sendStruct.pitch += 2.0;                                     // 2.0应该需要根据距离调整
                    else if (armorDetector.trace_light_bars_[0].GetCenter().y < 270) // 270是输出图像宽1080的1/4处
                        sendStruct.pitch -= 2.0;

                    sendStruct.no_Obj = false;// 想要让这部分代码起效必须把这里置为false

                }
            }
#ifdef VIDEO_DEBUG
            Debug::SetMessage("串口信息", "接收yaw和pitch", cv::Vec2f(receiveMessage.shoot_platform.yaw / CV_PI * 180, receiveMessage.shoot_platform.pitch / CV_PI * 180));
            Debug::SetMessage("串口信息", "发送yaw和pitch", cv::Vec2f(sendStruct.yaw, sendStruct.pitch));
            Debug::SetMessage("串口信息", "弹速", int(receiveMessage.bulletspeed));
            Debug::SetMessage("串口信息", "位置环静差", cv::Vec2f(receiveMessage.shoot_platform.yaw / CV_PI * 180 - sendStruct.yaw, receiveMessage.shoot_platform.pitch / CV_PI * 180 - sendStruct.pitch));
            Debug::SetMessage("串口信息", "开火命令", bool(sendStruct.beat));
#endif
        }
        else if (receiveMessage.mode == tdttoolkit::EnergyBuffMode) {
            sendStruct.pitch  = 0;
            sendStruct.yaw    = 0;
            sendStruct.beat   = false;
            sendStruct.no_Obj = true;
            std::cout << "--------------------------------BUFF-MODE-------------------------" << std::endl;

            buff_armors = buffDetector.Get(frame.cvimage_);

            double t2        = cv::getTickCount();
            int    usart_ret = usart.UsartRecv(recvStruct);
            std::cout << "Usart_Recv_RetCode: " << usart_ret << std::endl;
            Debug::GetOrSetReceiveMessage(&recvStruct);
            // simultaneity.calcTimeDiff(recvStruct.recieveTime, recieve_time);
            TimeSimulaneity.readData(recvStruct.data_, 120);
            tdttoolkit::TransformRecv(recvStruct, receiveMessage);
            std::cout << "GetImageTime=" << camera.GetImageTime() / 1000000 << std::endl;
            if (camera.GetImageTime() != -1) {
                int   t = TimeSimulaneity.timeMatch(camera.GetImageTime() / 1000000);
                float y, p;
                TimeSimulaneity.at(t, -2) >> y >> p;
                receiveMessage.shoot_platform = {float(p / 180 * CV_PI), float(y / 180 * CV_PI), bulletSpeedSolver.SolveSpeed(receiveMessage.bulletspeed, frame.header_.stamp_)};
            } else {
                receiveMessage.shoot_platform.bulletspeed = bulletSpeedSolver.SolveSpeed(receiveMessage.bulletspeed, frame.header_.stamp_);
            }
            std::cout << "tUsartRecv = " << (cv::getTickCount() - t2) / cv::getTickFrequency() * 1000 << "ms\n";

            if (buff_armors[0].GetEmpty()) {
                std::cout << "未找到流水灯" << std::endl;
                sendStruct.no_Obj = true;

#ifdef VIDEO_DEBUG

                Debug::SetMessage("串口信息", "接收pitch", (float)(receiveMessage.shoot_platform.pitch / CV_PI * 180));
                Debug::SetMessage("串口信息", "接收yaw", (float)(receiveMessage.shoot_platform.yaw / CV_PI * 180));
                Debug::SetMessage("串口信息", "发送pitch", (float)sendStruct.pitch);
                Debug::SetMessage("串口信息", "发送yaw", (float)sendStruct.yaw);
                Debug::SetMessage("串口信息", "接收弹速", float(receiveMessage.bulletspeed));
                Debug::SetMessage("串口信息", "是否击打", bool(sendStruct.beat));
#endif
#ifdef VIDEO_DEBUG
                Debug::UpdateView();
                Debug::LoopOver();
#endif
                

                continue;
            }
            sendStruct.no_Obj = false;
            buff_complete     = buffResolver.BuffResolve(buff_armors, receiveMessage);
            buffPredictor.SetNewTarget(buff_complete);
            buffPredictor.BuffPredict(receiveMessage, sendStruct);
            //            if(buff_complete.GetFinal()!=0)
            //            {
            //                std::cout<<"is_final_-----------------------------------------------------------------------------------------------------"<<std::endl;
            //                if(sendMessage.beat) {
            //                    usart.UsartSend(sendStruct);
            //                    usart.UsartRecv(recvStruct);
            //                    tdttoolkit::TransformRecv(recvStruct, receiveMessage);
            //                    buffPredictor.BuffPredict(receiveMessage,sendStruct);
            //                }
            //                else
            //                {
            //#ifdef VIDEO_DEBUG
            //                    Debug::UpdateView();
            //                    Debug::LoopOver();
            //#endif
            //                    continue;
            //                }
            //        }
            // BUFF_END:

        }
        else if (receiveMessage.mode == tdttoolkit::BuffDisturb) {
            sendStruct.pitch  = 0;
            sendStruct.yaw    = 0;
            sendStruct.beat   = false;
            sendStruct.no_Obj = true;
            std::cout << "--------------------------------BUFF-MODE-------------------------" << std::endl;
            buffDetector.disturb        = true;
            receiveMessage.disturb_buff = true;
            buff_complete.disturbbuff   = true;
            buff_armors                 = buffDetector.Get(frame.cvimage_);

            if (buff_armors[0].GetEmpty()) {
                std::cout << "未找到流水灯" << std::endl;
#ifdef VIDEO_DEBUG
                Debug::UpdateView();
                Debug::LoopOver();
#endif
                continue;
            }

            buff_complete = buffResolver.BuffResolve(buff_armors, receiveMessage);
            buffPredictor.SetNewTarget(buff_complete);
            buffPredictor.BuffPredict(receiveMessage, sendStruct);
        }
        else if(receiveMessage.mode == tdttoolkit::LobMode)
        {
            std::cout << "--------------------------------LOB-MODE-------------------------" << std::endl;
        }
        // sendStruct.beat = true;

        // std::cout << std::fixed << "ReceiveMessage: " << receiveMessage.shoot_platform.yaw / CV_PI * 180 << ", " << receiveMessage.shoot_platform.pitch / CV_PI * 180 << "（角度)\n";
        // std::cout << "SendMessage: " << sendStruct.yaw << ", " << sendStruct.pitch / 100.0f << "（角度）\n";
        // std::cout << "ReceiveMessage - SendMessage = " << receiveMessage.shoot_platform.yaw / CV_PI * 180 - sendStruct.yaw << ", " << receiveMessage.shoot_platform.pitch / CV_PI * 180 - sendStruct.pitch << std::endl;
        // std::cout << "SendMessage.beat = " << bool(sendStruct.beat) << std::endl;
        // std::cout << "SendMessage.LockCommand: " << (int)recvStruct.lockCommand << std::endl;
        // std::cout << "SendMessage.bulletSpeed: " << (float)receiveMessage.bulletspeed << std::endl;

        double t3 = cv::getTickCount();
        std::cout << "Usart_Send_RetCode: " << usart.UsartSend(sendStruct) << std::endl;
        std::cout << "tUsartSend = " << (cv::getTickCount() - t3) / cv::getTickFrequency() * 1000 << "ms\n";
        Debug::AddPoint("串口发送", "yaw", cv::Point2f(Time::GetTimeNow() / 1000000.0f, sendStruct.yaw));
        Debug::AddPoint("串口发送", "pitch", cv::Point2f(Time::GetTimeNow() / 1000000.0f, sendStruct.pitch));
        Debug::AddPoint("串口接收", "yaw", cv::Point2f(camera.GetImageTime(), receiveMessage.shoot_platform.yaw / CV_PI * 180));
        Debug::AddPoint("串口接收", "pitch", cv::Point2f(camera.GetImageTime(), receiveMessage.shoot_platform.pitch / CV_PI * 180));
        Debug::UpdateView();
        Debug::LoopOver();
        std::signal(SIGINT, ProgEnd); //增加了按住Ctrl+C，保存视频
        std::signal(SIGHUP, ProgEnd); //关机前保存视频
        // std::signal(SIGINT,Debug::DeleteDebug);         //TODO:不知道为什么时间短可以直接停时间长不能停
        // std::signal(SIGINT,Debug::DeleteDebug);         //TODO:不知道为什么时间短可以直接停时间长不能停
    }
#pragma clang diagnostic pop
}
void qtdog() {
    int          argc;
    char **      argv;
    QApplication a(argc, argv);
    Debug::Init();
    Debug::Run(ProgOnImage);
    a.exec();
}

int main(int argc, char **argv) {
    nice(-20);
    int times  = 0;
    int times2 = 0;
#ifdef HAVE_TBB
    TDT_INFO("当前Opencv使用的TBB加速线程数为%d", cv::getNumThreads());
#endif
    LoadParam::Init("../../config/robot_param.yaml");
    tdtml::NumberDetector::Init(new tdtml::MxnetNumberDetector());
    tdttoolkit::Time::Init();
    tdtlog::Log::Init();
    tdtlog::Log::AddChannelList("ArmorDetector");
    tdtlog::Log::AddChannelList("BuffDetector");
    /// tdtimm::ModelGenerator Model;

//    ProgOnImage();
//    return 0;
#ifdef VIDEO_DEBUG
#ifdef WATCH_DOG
    watchdog.StartWatchDog(qtdog);
#else
    QApplication a(argc, argv);
    Debug::Init();
    Debug::Run(ProgOnImage);
    a.exec();
#endif
#else
#ifdef WATCH_DOG
    watchdog.StartWatchDog(ProgOnImage);
#else
    ProgOnImage();

#endif // WATCH_DOG
    return 0;
#endif
}
