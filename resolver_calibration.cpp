#ifdef RESOLVER_CALIBRATION

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

#include <opencv2/videoio.hpp>

using namespace tdttoolkit;
#ifdef WATCH_DOG
tdttoolkit::WatchDog watchdog = tdttoolkit::WatchDog();
#endif
std::deque<cv::Point3f> camera_points;

void solver(cv::Point3f point1, cv::Point3f point2, cv::Point3f point3, cv::Point3f point4, cv::Point3f &center, double &radius) {

    std::vector<float> p1 = {point1.x, point1.y, point1.z};
    std::vector<float> p2 = {point2.x, point2.y, point2.z};
    std::vector<float> p3 = {point3.x, point3.y, point3.z};
    std::vector<float> p4 = {point4.x, point4.y, point4.z};

    double a = p1[0] - p2[0], b = p1[1] - p2[1], c = p1[2] - p2[2];
    double a1 = p3[0] - p4[0], b1 = p3[1] - p4[1], c1 = p3[2] - p3[2];
    double a2 = p2[0] - p3[0], b2 = p2[1] - p3[1], c2 = p2[2] - p3[2];
    double D = a * b1 * c2 + a2 * b * c1 + c * a1 * b2 - (a2 * b1 * c + a1 * b * c2 + a * b2 * c1);
    if (D == 0) {
        radius = -1;
        return;
    }

    double A  = p1[0] * p1[0] - p2[0] * p2[0];
    double B  = p1[1] * p1[1] - p2[1] * p2[1];
    double C  = p1[2] * p1[2] - p2[2] * p2[2];
    double A1 = p3[0] * p3[0] - p4[0] * p4[0];
    double B1 = p3[1] * p3[1] - p4[1] * p4[1];
    double C1 = p3[2] * p3[2] - p4[2] * p4[2];
    double A2 = p2[0] * p2[0] - p3[0] * p3[0];
    double B2 = p2[1] * p2[1] - p3[1] * p3[1];
    double C2 = p2[2] * p2[2] - p3[2] * p3[2];
    double P  = (A + B + C) / 2;
    double Q  = (A1 + B1 + C1) / 2;
    double R  = (A2 + B2 + C2) / 2;

    double Dx = P * b1 * c2 + b * c1 * R + c * Q * b2 - (c * b1 * R + P * c1 * b2 + Q * b * c2);
    double Dy = a * Q * c2 + P * c1 * a2 + c * a1 * R - (c * Q * a2 + a * c1 * R + c2 * P * a1);
    double Dz = a * b1 * R + b * Q * a2 + P * a1 * b2 - (a2 * b1 * P + a * Q * b2 + R * b * a1);

    center.x = Dx / D;
    center.y = Dy / D;
    center.z = Dz / D;
    radius   = sqrt((p1[0] - center.x) * (p1[0] - center.x) + (p1[1] - center.y) * (p1[1] - center.y) + (p1[2] - center.z) * (p1[2] - center.z));
}
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

#ifdef VIDEO_DEBUG
    tdtusart::Usart &usart       = *(Debug::GetVirtualUsart());
    robotPredictor.armorResolver = &armorResolver;
#else
    tdtusart::RealUsart     realUsart;
    tdtusart::Usart &       usart = realUsart;
#endif
    cv::Mat                         last_src;
    tdttoolkit::ResolvedArmor       last_resolved_armor;
    double                          last_tick = 0;
    tdt_communication::SharedMemory sharedMemory(0, sizeof(unsigned long long), tdt_communication::SharedMemory::NodeType::Server);
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
        if (frame.cvimage_.empty()) {
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
        tdttoolkit::Time::UpdateFrameTime();
        Debug::AddText("时间戳", "Time:" + std::to_string(Time::GetFrameTime() / 1000000.0f), cv::Point2d(20, 30), cv::Scalar(255, 255, 255), 1, 3);
        tdtlog::Log::AllChannelLog("A new Frame.");
        double t_sm = cv::getTickCount();

#ifndef VIDEO_DEBUG
        receiveMessage.mode = tdttoolkit::ArmorMode;
#endif

        static auto                        p = new TimeCounter("Armor Detector");
        pthread_t                          num_predict;
        std::vector<tdttoolkit::RobotType> types;
        auto                               armors = armorDetector.Get(frame.cvimage_, receiveMessage, num_predict, &types);
        //
        int usart_ret = usart.UsartRecv(recvStruct);
        std::cout << "Usart_Recv_RetCode: " << usart_ret << std::endl;
        Debug::GetOrSetReceiveMessage(&recvStruct);
        TimeSimulaneity.readData(recvStruct.data_, 120);
        tdttoolkit::TransformRecv(recvStruct, receiveMessage);
        if (camera.GetImageTime() != -1) {
            int   t = TimeSimulaneity.timeMatch(camera.GetImageTime() / 1000000);
            float y, p;
            TimeSimulaneity.at(t, -2) >> y >> p;
            receiveMessage.shoot_platform = {p / 180 * CV_PI, y / 180 * CV_PI, bulletSpeedSolver.SolveSpeed(receiveMessage.bulletspeed, frame.header_.stamp_)};
        } else {
            receiveMessage.shoot_platform.bulletspeed = bulletSpeedSolver.SolveSpeed(receiveMessage.bulletspeed, frame.header_.stamp_);
        }
        //
        if (!armors.empty()) {
            double t_resolver = cv::getTickCount();
            sendStruct.no_Obj = false;

            auto                       resolved_armors = armorResolver.Resolve(armors, receiveMessage.shoot_platform);
            std::vector<ResolvedArmor> output_armors;
            void **                    retval;
            pthread_join(num_predict, retval);
#if defined(AUTO_DECIDE) && !defined(RADER_UPDATE)
            // Decide(armors, output_armors);
            for (int i = 0; i < resolved_armors.size(); i++) {
                if ((types[i] != tdttoolkit::RobotType::TYPEUNKNOW) && (resolved_armors[i].GetResolvedStatus())) {
                    resolved_armors[i].SetRobotType(types[i]);
                    output_armors.emplace_back(resolved_armors[i]);
                }
            }
#else
            if (armors.empty()) {
                std::vector<ResolvedArmor>().swap(output_armors);
            } else {
                for (int i = 0; i < armors.size(); i++) {
                    if (types[i] != tdttoolkit::RobotType::TYPEUNKNOW) {
                        resolved_armors[i].SetRobotType(types[i]);
                        output_armors.emplace_back(armors[i]);
                    }
                }
            }
#endif      // defined(AUTO_DECIDE) && !defined(RADER_UPDATE)
            /************************ARMOR_DETECT_DEBUG***************************/
#ifdef ARMOR_DETECT_DEBUG
#ifdef VIDEO_DEBUG
            Debug::SetMessage("装甲板检测", "机器学习后装甲板数量", int(output_armors.size()));
            for (auto const &armor : output_armors) {
                for (auto const &point : armor.GetImagePoints()) {
                    Debug::AddPoint("输出装甲板", point, 10, cv::Scalar(255, 0, 0));
                }
                Debug::AddCustomRect("输出装甲板", armor.GetStickerRect(), cv::Scalar(0, 255, 0), 3);
                Debug::AddText("输出装甲板", {char('0' + armor.GetRobotType())}, armor.GetStickerRect().GetBl(), cv::Scalar(255, 255, 0), 1, 2);
            }
#endif // VIDEO_DEBUG
#endif // ARMOR_DETECT_DEBUG
            std::cout << "找到" << output_armors.size() << "个装甲板\n";
            tdtlog::Log::ChannelLog("ArmorDetector", "找到%d个装甲板", output_armors.size());
            /****************************************************************************/
            cv::Point2d orientedProj = armorResolver.ProjectArmor(receiveMessage.shoot_platform, output_armors[0]);
            Debug::AddCircle("解算点投影", orientedProj, 6, cv::Scalar(255, 255, 0));

            Robot robot     = robotDecision.Decide(output_armors, receiveMessage);
            resolved_armors = robot.GetResolvedArmors();
            if (output_armors.empty())
                goto WHILE_END;
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
            robotPredictor.time_diff_ = 0;
            /**********************************/
            if (resolved_armors.size() > 1) {
                TDT_INFO("[axis]找到的装甲板大于1个，将会对tag为%d的装甲板进行检测", resolved_armors[0].GetTag());
            }
            auto            rvec = resolved_armors[0].GetPNPRvec();
            auto            tvec = resolved_armors[0].GetPNPTvec();
            cv::Mat_<float> rotMat(3, 3);
            cv::Rodrigues(rvec, rotMat);
            cv::invert(rotMat, rotMat);
            auto CameraInObject = tdttoolkit::MatToPoint3f(-rotMat * tvec);
            if (!camera_points.empty()) {
                if (abs(tdttoolkit::CalcDistance(CameraInObject, camera_points.back())) > 10)
                    camera_points.push_back(CameraInObject);
            } else
                camera_points.push_back(CameraInObject);
            if (camera_points.size() > 6)
                camera_points.pop_front();
            std::vector<cv::Point3f> centers;
            if (camera_points.size() > 3) {
                for (int i = 0; i < camera_points.size() - 3; i++) {
                    for (int j = i + 1; j < camera_points.size() - 2; j++) {
                        for (int k = j + 1; k < camera_points.size() - 1; k++) {
                            for (int l = j + 1; l < camera_points.size() - 1; l++) {
                                cv::Point3f CenterOfCircle;
                                double      r;
                                solver(camera_points[i], camera_points[j], camera_points[k], camera_points[l], CenterOfCircle, r);
                                if (r > 0) {
                                    centers.push_back(CenterOfCircle);
                                };
                            }
                        }
                    }
                }
            }
            if (centers.size() > 0) {
                cv::Point3f per;
                int         t = centers.size();
                for (int i = 0; i < t; i++) {
                    per += centers[i];
                }
                per /= t;
                cv::Point3f variance;
                for (int i = 0; i < t; i++) {
                    variance += cv::Point3f((centers[i].x - per.x) * (centers[i].x - per.x), (centers[i].y - per.y) * (centers[i].y - per.y), (centers[i].z - per.z) * (centers[i].z - per.z));
                }
                variance /= t;
                cv::Mat center(per);
                cv::Rodrigues(rvec, rotMat);
                center = rotMat * center + tvec;
                std::cout << "[axis]所求得的轴心坐标为" << tdttoolkit::MatToPoint3f(center) << ",方差为" << variance << std::endl;
            }
            /***********************************/
#define SPECIALMODE
            robotPredictor.Update(resolved_armors, tdttoolkit::Time::GetTimeNow());
            robotPredictor.FireCommand(receiveMessage, sendStruct);
        }
#ifdef VIDEO_DEBUG
        Debug::SetMessage("串口信息", "接收yaw和pitch", cv::Vec2f(receiveMessage.shoot_platform.yaw / CV_PI * 180, receiveMessage.shoot_platform.pitch / CV_PI * 180));
        Debug::SetMessage("串口信息", "发送yaw和pitch", cv::Vec2f(sendStruct.yaw, sendStruct.pitch));
        Debug::SetMessage("串口信息", "弹速", int(receiveMessage.bulletspeed));
        Debug::SetMessage("串口信息", "位置环静差", cv::Vec2f(receiveMessage.shoot_platform.yaw / CV_PI * 180 - sendStruct.yaw, receiveMessage.shoot_platform.pitch / CV_PI * 180 - sendStruct.pitch));
        Debug::SetMessage("串口信息", "开火命令", bool(sendStruct.beat));
#endif
    WHILE_END:
        std::cout << "Usart_Send_RetCode: " << usart.UsartSend(sendStruct) << std::endl;

        Debug::UpdateView();
        Debug::LoopOver();
        std::signal(SIGINT, tdtlog::VideoRecoder::Release);
    }
}
#pragma clang diagnostic pop

[[noreturn]] void qtdog() {
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
#endif