/*
 * @Name: file name
 * @Description:
 * @Version: 1.0.0.1
 * @Author: your name
 * @Date: 2019-10-14 19:42:40
 * @LastEditors: your name
 * @LastEditTime: 2019-11-22 20:37:14
 */

#include "tdtcamera.h"
#include "parameter/load_param.h"
#include "tdtcommon.h"
#include <chrono>

namespace tdtcamera {
    uint8_t Camera::cam_online_num_ = 0;
    uint8_t Camera::cam_total_num_  = 0;
    Header::Header(uint64_t seq) : seq_(seq) {
        struct timeval tv;
        gettimeofday(&tv, NULL);
        stamp_ = tv.tv_sec * 1E6 + tv.tv_usec;
    }

    Camera::Camera(std::string config_path) {
        path=config_path;
        LoadParam(config_path);
        identity_.camera_id = cam_total_num_;
        cam_total_num_++;
        cam_online_num_++;
    }
    Camera::~Camera() { cam_online_num_--; }

    int Camera::CameraCalibrate(const cv::Mat &src) {
        if (src.empty())
            return 0;
        cv::Size                 board_size = cv::Size(11, 8);
        std::vector<cv::Point2f> image_points_buf; /* 缓存每幅图像上检测到的角点 */

        //    cv::waitKey(2);
        cv::namedWindow("Calibrate", cv::WINDOW_NORMAL);

        cv::Mat show;

        if (camera_calibrate_points_sequence_.size() < 15) {
            src.copyTo(show);
            putText(show, "press 'f'", cv::Point(100, 100), 0, 1.5, cv::Scalar(255, 255, 255), 1);
            putText(show, "to find or 'q' to quit" + std::to_string(camera_calibrate_points_sequence_.size() + 1) + "of15 Chessboard Corners", cv::Point(100, 150), 0, 1.5, cv::Scalar(255, 255, 255), 1);
            putText(show, "Aention:Make sure whole chessboard in view", cv::Point(100, 200), 0, 1, cv::Scalar(0, 0, 255), 2);
            putText(show, "or it will crash", cv::Point(100, 250), 0, 1, cv::Scalar(0, 0, 255), 2);
            imshow("Calibrate", show);

            if (cv::waitKey(5) == 'f') {
                cv::Mat view_gray;
                cvtColor(show, view_gray, cv::COLOR_BGR2GRAY);
                src.copyTo(show);
                putText(show, "finding...", cv::Point(src.size() / 2), 0, 2, cv::Scalar(0, 255, 0), 2);
                putText(show, "Warning :If it crash, Please wait 30 seconds", cv::Point(src.size() / 5), 0, 1, cv::Scalar(0, 0, 255), 1);
                imshow("Calibrate", show);
                cv::waitKey(10);
                if (findChessboardCorners(src, board_size, image_points_buf)) {
                    find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(5, 5)); //对粗提取的角点进行精确化

                    drawChessboardCorners(show, board_size, image_points_buf, false); //在图片中标记角点
                    putText(show, std::to_string(camera_calibrate_points_sequence_.size() + 1) + "of15", cv::Point(show.size().width / 3, 100), 0, 1.5, cv::Scalar(255, 255, 255), 2);
                    putText(show, "whether to save these ChessboardCorners?", cv::Point(src.size().width / 3, 200), 0, 1, cv::Scalar(255, 255, 255), 2);
                    putText(show, "mouse d:delete ", cv::Point(show.size().width * 2 / 3, src.size().height / 2), 0, 1.5, cv::Scalar(255, 0, 0), 1);
                    imshow("Calibrate", show);
                    if (cv::waitKey(2000) == 'd')
                        return 0;
                    camera_calibrate_points_sequence_.push_back(image_points_buf);
                } else {
                    putText(show, "can't find cornets", cv::Point(show.size().width / 3, 100), 0, 1.5, cv::Scalar(255, 255, 255), 2);
                    cv::waitKey(500);
                    imshow("Calibrate", show);
                }
            } else if (cv::waitKey(5) == 'q')
                return 1;
        } else {
            cv::destroyWindow("Calibrate");
            std::cout << "开始标定………………";
            /*棋盘三维信息*/
            cv::Size                              square_size = cv::Size(2, 2);                                /* 实际测量得到的标定板上每个棋盘格的大小 厘米*/
            std::vector<std::vector<cv::Point3f>> object_points;                                               /* 保存标定板上角点的三维坐标 */
            cv::Mat                               camera_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 摄像机内参数矩阵 */
            int                                   point_counts  = board_size.width * board_size.height;        // 每幅图像中角点的数量
            cv::Mat                               dist_coeffs   = cv::Mat(5, 1, CV_32FC1, cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
            std::vector<cv::Mat>                  tvecs_mat;                                                   /* 每幅图像的旋转向量 */
            std::vector<cv::Mat>                  rvecs_mat;                                                   /* 每幅图像的平移向量 */
            /* 初始化标定板上角点的三维坐标 */
            int i, j, t;
            for (t = 0; t < camera_calibrate_points_sequence_.size(); t++) {
                std::vector<cv::Point3f> temp_point_set;
                for (i = 0; i < board_size.height; i++) {
                    for (j = 0; j < board_size.width; j++) {
                        cv::Point3f realPoint;
                        /* 假设标定板放在世界坐标系中z=0的平面上 */
                        realPoint.x = i * square_size.width;
                        realPoint.y = j * square_size.height;
                        realPoint.z = 0;
                        temp_point_set.push_back(realPoint);
                    }
                }
                object_points.push_back(temp_point_set);
            }

            /* 开始标定 */
            calibrateCamera(object_points, camera_calibrate_points_sequence_, src.size(), camera_matrix, dist_coeffs, rvecs_mat, tvecs_mat, 0);

            std::cout << camera_matrix << std::endl;
            std::cout << dist_coeffs << std::endl;
            std::cout << "标定完成！\n";

            //对标定结果进行评价
            std::cout << "开始评价标定结果………………\n";
            double                   total_err = 0.0; /* 所有图像的平均误差的总和 */
            double                   err       = 0.0; /* 每幅图像的平均误差 */
            std::vector<cv::Point2f> image_points2;   /* 保存重新计算得到的投影点 */
            std::cout << "\t每幅图像的标定误差：\n";

            for (int i = 0; i < camera_calibrate_points_sequence_.size(); i++) {
                std::vector<cv::Point3f> tempPointSet = object_points[i];
                /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */

                projectPoints(tempPointSet, rvecs_mat[i], tvecs_mat[i], camera_matrix, dist_coeffs, image_points2);

                /* 计算新的投影点和旧的投影点之间的误差*/
                std::vector<cv::Point2f> tempImagePoint       = camera_calibrate_points_sequence_[i];
                cv::Mat                  temp_image_point_mat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
                cv::Mat                  image_points2mat     = cv::Mat(1, image_points2.size(), CV_32FC2);
                for (int j = 0; j < tempImagePoint.size(); j++) {
                    image_points2mat.at<cv::Vec2f>(0, j)     = cv::Vec2f(image_points2[j].x, image_points2[j].y);
                    temp_image_point_mat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
                }
                std::cout << "flag" << std::endl;
                err = norm(image_points2mat, temp_image_point_mat, cv::NORM_L2);
                total_err += err /= point_counts;
                std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << std::endl;
            }
            std::cout << "总体平均误差：" << total_err / camera_calibrate_points_sequence_.size() << "像素" << std::endl;
            std::cout << "评价完成！" << std::endl;

//            std::string     file_address  = "../../config/camera_param.yaml";
            std::string     file_address_ = "../../config/robot_param.yaml";
            cv::FileStorage fs(file_address_, cv::FileStorage::APPEND);
//            fs << "matrix" << camera_matrix;
//            fs << "dist_coeffs" << dist_coeffs;
//            fs.release();
//            fs.open(file_address_, cv::FileStorage::APPEND);
            fs << "matrix" << camera_matrix;
            fs << "dist_coeffs" << dist_coeffs;
            fs.release();
            return 1;
        }
        return 0;
    }
    bool Camera::LoadParam(std::string config_path) {
        YAML::Node fs = YAML::LoadFile(config_path);
        if (fs.IsNull()) {
            TDT_ERROR("Can not open the configuration file of camera!");
            return false;
        } else {
            identity_.type      = TDT_CAMERA_CAMTYPE_HIKVISION;
            identity_.dev_index = 0;
            identity_.name      = fs["name"].as<std::string>();
            identity_.type      = static_cast<TDT_CAMERA_CAMTYPE>(fs["type"].as<uint32_t>());

            identity_.guid = fs["guid"].as<std::string>();

            identity_.dev_index = fs["dev_index"].as<uint32_t>();
            identity_.dev_path  = fs["path"].as<std::string>();
#ifndef Calibrate
            identity_.camera_matrix = fs["matrix"].as<cv::Mat>();
            identity_.dist_coeffs   = fs["dist_coeffs"].as<cv::Mat>();
#endif
            format_.width        = fs["width"].as<uint32_t>();
            format_.height       = fs["height"].as<uint32_t>();
            format_.fps          = fs["fps"].as<float>();
            format_.pixel_format = fs["pixel_format"].as<uint32_t>();

            int value=0;  //用来先将参数转换为int，然后转换为uint32_t
            LoadParam::ReadTheParam("exposure",setting_.exposure);
            LoadParam::ReadTheParam("gain",setting_.gain);
            LoadParam::ReadTheParam("balance_val",value);
            setting_.balance_val   = value;
            LoadParam::ReadTheParam("balance_red",value);
            setting_.balance_red   = value;
            LoadParam::ReadTheParam("balance_green",value);
            setting_.balance_green = value;
            LoadParam::ReadTheParam("balance_blue",value);
            setting_.balance_blue  = value;
            LoadParam::ReadTheParam("brightness",value);
            setting_.brightness    = value;
            LoadParam::ReadTheParam("saturation",value);
            setting_.saturation    = value;
            LoadParam::ReadTheParam("contrast",value);
            setting_.contrast      = value;
            LoadParam::ReadTheParam("gamma",setting_.gamma);
            LoadParam::ReadTheParam("sharpness",value);
            setting_.sharpness     = value;
            LoadParam::ReadTheParam("black_level",value);
            setting_.black_level   = value;
            return true;
        }
    }
    //用于在debug中动态的设置相机参数
    bool Camera::LoadParam() {
            int value=0;  //用来先将参数转换为int，然后转换为uint32_t
            LoadParam::ReadTheParam("exposure",setting_.exposure);
            LoadParam::ReadTheParam("gain",setting_.gain);
            LoadParam::ReadTheParam("balance_val",value);
            setting_.balance_val   = value;
            LoadParam::ReadTheParam("balance_red",value);
            setting_.balance_red   = value;
            LoadParam::ReadTheParam("balance_green",value);
            setting_.balance_green = value;
            LoadParam::ReadTheParam("balance_blue",value);
            setting_.balance_blue  = value;
            LoadParam::ReadTheParam("brightness",value);
            setting_.brightness    = value;
            LoadParam::ReadTheParam("saturation",value);
            setting_.saturation    = value;
            LoadParam::ReadTheParam("contrast",value);
            setting_.contrast      = value;
            LoadParam::ReadTheParam("gamma",setting_.gamma);
            LoadParam::ReadTheParam("sharpness",value);
            setting_.sharpness     = value;
            LoadParam::ReadTheParam("black_level",value);
            setting_.black_level   = value;
            return true;
    }

    UVCCam::UVCCam(std::string config_path) : Camera(config_path) {
        if (identity_.type != TDT_CAMERA_CAMTYPE_UVCCAM) {
            TDT_FATAL("Failed to open UVC camera. (incorrect type %d)", identity_.type);
        }
        if (identity_.guid != "") { // yaml文件中guid一直是空字符串
            if (!uvccam_.InitCamera(identity_.guid)) {
                TDT_FATAL("Failed to open UVC camera. (incorrect guid %s)", identity_.guid.c_str());
            }
        } else { //默认打开第0个字符串
            if (!uvccam_.InitCamera(identity_.dev_index)) {
                TDT_FATAL("Failed to open UVC camera. (incorrect dev_index %d)", identity_.dev_index);
            }
            identity_.guid = uvccam_.get_guid();
        }
        Set(TDT_CAMERA_FORMAT_PIXEL, (unsigned int)format_.pixel_format);
        Set(TDT_CAMERA_FORMAT_WIDTH, format_.width); // TODO 静态，长宽同时设定
        Set(TDT_CAMERA_FORMAT_HEIGHT, format_.height);
        Set(TDT_CAMERA_FORMAT_FPS, format_.fps);
        Set(TDT_CAMERA_SETTING_EXPOSURE, setting_.exposure);
        Set(TDT_CAMERA_SETTING_GAIN, setting_.gain);
        Set(TDT_CAMERA_SETTING_BRIGHTNESS, setting_.brightness);
        Set(TDT_CAMERA_SETTING_BALANCE_VAL, setting_.balance_val);
        Set(TDT_CAMERA_SETTING_HUE, setting_.hue);
        Set(TDT_CAMERA_SETTING_SATURATION, setting_.saturation);
        Set(TDT_CAMERA_SETTING_CONTRAST, setting_.contrast);
        Set(TDT_CAMERA_SETTING_GAMMA, setting_.gamma);
        Set(TDT_CAMERA_SETTING_SHARPNESS, setting_.sharpness);
        if (!uvccam_.StartStream()) {
            TDT_FATAL("Failed to open UVC camera. (cannot start stream %d)", identity_.dev_index);
        }
    }

    bool UVCCam::Set(tdtcamera::Camera::TDT_CAMERA_FORMAT tdt_camera_format, unsigned int val) {
        bool ret = true;
        // uvccam_.CloseStream();
        switch (tdt_camera_format) {
        case TDT_CAMERA_FORMAT_PIXEL:
            if (val == 0) {
                ret &= uvccam_.SetPixelformat(V4L2_PIX_FMT_MJPEG);
            } else if (val == 1) {
                ret &= uvccam_.SetPixelformat(V4L2_PIX_FMT_YUYV);
            } else {
                TDT_ERROR("Failed to set UVC camera format. (incorrect pixel_format %d)", val);
                // uvccam_.StartStream();
                return false;
            }
            if (ret) {
                format_.pixel_format = val;
                // uvccam_.StartStream();
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera format. (pixel_format %d)", val);
                // uvccam_.StartStream();
                return false;
            }
            break;
        case TDT_CAMERA_FORMAT_WIDTH:
            ret &= uvccam_.SetResolution(val, format_.height);
            if (ret) {
                format_.width = val;
                // uvccam_.StartStream();
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera format. (width %d)", val);
                // uvccam_.StartStream();
                return false;
            }
            break;
        case TDT_CAMERA_FORMAT_HEIGHT:
            ret &= uvccam_.SetResolution(format_.width, val);
            if (ret) {
                format_.height = val;
                // uvccam_.StartStream();
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera format. (height %d)", val);
                // uvccam_.StartStream();
                return false;
            }
            break;
        case TDT_CAMERA_FORMAT_FPS:
            ret &= uvccam_.SetFps(val);
            if (ret) {
                format_.fps = val;
                // uvccam_.StartStream();
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera format. (fps %d)", val);
                // uvccam_.StartStream();
                return false;
            }
            break;
        default:
            TDT_ERROR("Failed to set UVC camera format. (TDT_CAMERA_SETTING : %d)", tdt_camera_format);
            return false;
            break;
        }
    }

    bool UVCCam::Set(tdtcamera::Camera::TDT_CAMERA_SETTING tdt_camera_setting, unsigned int val) {
        bool ret = true;
        switch (tdt_camera_setting) {
        case TDT_CAMERA_SETTING_EXPOSURE:
            if (val == 0xFFFFFFFF) {
                ret &= uvccam_.SetExposureAuto(true);
            } else {
                ret &= uvccam_.SetExposureAuto(false);
                ret &= uvccam_.SetExposure(val);
            }
            if (ret) {
                setting_.exposure = val;
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera setting. (exposure : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_GAIN:
            if (val == 0xFFFFFFFF) {
                ret &= uvccam_.SetGainAuto(true);
            } else {
                ret &= uvccam_.SetGainAuto(false);
                ret &= uvccam_.SetGain(val);
            }
            if (ret) {
                setting_.gain = val;
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera setting. (gain : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_BRIGHTNESS:
            if (val == 0xFFFFFFFF) {
                ret &= uvccam_.SetBrightnessAuto(true);
            } else {
                ret &= uvccam_.SetBrightnessAuto(false);
                ret &= uvccam_.SetBrightness(val);
            }
            if (ret) {
                setting_.brightness = val;
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera setting. (brightness : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_BALANCE_RED:
            if (val == 0xFFFFFFFF) {
                ret &= uvccam_.SetWhitebalanceAuto(true);
            } else {
                ret &= uvccam_.SetWhitebalanceAuto(false);
                ret &= uvccam_.SetWhitebalance(val);
            }
            if (ret) {
                setting_.balance_val = val;
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera setting. (balance_val : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_HUE:
            if (val != 0xFFFFFFFF) {
                ret &= uvccam_.SetHue(val);
            }
            if (ret) {
                setting_.hue = val;
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera setting. (hue : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_SATURATION:
            if (val != 0xFFFFFFFF) {
                ret &= uvccam_.SetSaturation(val);
            }
            if (ret) {
                setting_.saturation = val;
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera setting. (saturation : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_CONTRAST:
            if (val != 0xFFFFFFFF) {
                ret &= uvccam_.SetContrast(val);
            }
            if (ret) {
                setting_.contrast = val;
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera setting. (contrast : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_GAMMA:
            if (val != 0xFFFFFFFF) {
                ret &= uvccam_.SetGamma(val);
            }
            if (ret) {
                setting_.gamma = val;
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera setting. (gamma : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_SHARPNESS:
            if (val != 0xFFFFFFFF) {
                ret &= uvccam_.SetSharpness(val);
            }
            if (ret) {
                setting_.sharpness = val;
                return true;
            } else {
                TDT_ERROR("Failed to set UVC camera setting. (sharpness : %d)", val);
                return false;
            }
            break;
        default:
            TDT_ERROR("Failed to set UVC camera setting. (TDT_CAMERA_SETTING : %d)", tdt_camera_setting);
            return false;
            break;
        }
    }

    bool UVCCam::GetImage(tdtcamera::TImage &timage) {
        cv::Mat src;
        if (!uvccam_.GetMat(src)) {
            TDT_ERROR("Failed to get image from UVC cam");
        }
        timage = TImage(src, ++seq_);
    }

    bool UVCCam::RestartCamera() { return uvccam_.RestartCamera(); }

    HikvisionCam::HikvisionCam(std::string config_path) : Camera(config_path) {
        Stop=false;
        std::cout << "pass1" << std::endl;
        if (identity_.type != TDT_CAMERA_CAMTYPE_HIKVISION) {
            TDT_FATAL("Failed to open hikvision camera. (incorrect type %d)", identity_.type);
        }
        if (identity_.guid != "") {
            if (!hikvisioncam_.InitHandle(identity_.guid)) {
                TDT_FATAL("Failed to open hikvision camera. (incorrect guid %s)", identity_.guid.c_str());
            }
        } else {
            if (!hikvisioncam_.InitHandle(identity_.dev_index)) {
                TDT_FATAL("Failed to open hikvision camera. (incorrect dev_index %d)", identity_.dev_index);
            }
            identity_.guid = hikvisioncam_.get_guid();
        }
        if (!hikvisioncam_.OpenCamera())
            exit(-1); //相机存在但是相机错误，就exit
        if (format_.pixel_format == 0) {
            hikvisioncam_.SetPixelformat(PixelType_Gvsp_RGB8_Packed);

        } else if (format_.pixel_format == 1) {
            hikvisioncam_.SetPixelformat(PixelType_Gvsp_BayerRG8);
        }
        hikvisioncam_.SetResolution(format_.width, format_.height);
        hikvisioncam_.SetFps(format_.fps);
        Set(TDT_CAMERA_SETTING_EXPOSURE, setting_.exposure);
        Set(TDT_CAMERA_SETTING_GAIN, setting_.gain);
        Set(TDT_CAMERA_SETTING_BRIGHTNESS, setting_.brightness);
        Set(TDT_CAMERA_SETTING_BALANCE_RED, setting_.balance_red);
        Set(TDT_CAMERA_SETTING_BALANCE_GREEN, setting_.balance_green);
        Set(TDT_CAMERA_SETTING_BALANCE_BLUE, setting_.balance_blue);
        Set(TDT_CAMERA_SETTING_HUE, setting_.hue);
        Set(TDT_CAMERA_SETTING_SATURATION, setting_.saturation);
        Set(TDT_CAMERA_SETTING_GAMMA, setting_.gamma);
        Set(TDT_CAMERA_SETTING_SHARPNESS, setting_.sharpness);
        Set(TDT_CAMERA_SETTING_TRIGER_IMAGE, (float)1.0f);
        std::cout << setting_.sharpness << std::endl;
        std::cout << 0xFFFFFFFF << std::endl;
        Set(TDT_CAMERA_SETTING_BLACK_LEVEL, setting_.black_level);
        hikvisioncam_.StartGrabbing();
        std::thread t(&HikvisionCam::TakeFrame, this);
        t.detach();
    }

    bool HikvisionCam::Set(tdtcamera::Camera::TDT_CAMERA_FORMAT tdt_camera_format, unsigned int val) {
        bool ret = true;
        hikvisioncam_.CloseGrabbing();
        switch (tdt_camera_format) {
        case TDT_CAMERA_FORMAT_PIXEL:
            if (val == 0) {
                ret &= hikvisioncam_.SetPixelformat(PixelType_Gvsp_RGB8_Packed);
            } else if (val == 1) {
                ret &= hikvisioncam_.SetPixelformat(PixelType_Gvsp_BayerRG8);
            } else {
                TDT_ERROR("Failed to set hikvision camera format. (incorrect pixel_format %d)", val);
                hikvisioncam_.StartGrabbing();
                return false;
            }
            if (ret) {
                format_.pixel_format = val;
                hikvisioncam_.StartGrabbing();
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera format. (pixel_format %d)", val);
                hikvisioncam_.StartGrabbing();
                return false;
            }
            break;
        case TDT_CAMERA_FORMAT_WIDTH:
            ret &= hikvisioncam_.SetResolution(val, format_.height);
            if (ret) {
                format_.width = val;
                hikvisioncam_.StartGrabbing();
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera format. (width %d)", val);
                hikvisioncam_.StartGrabbing();
                return false;
            }
            break;
        case TDT_CAMERA_FORMAT_HEIGHT:
            ret &= hikvisioncam_.SetResolution(format_.width, val);
            if (ret) {
                format_.height = val;
                hikvisioncam_.StartGrabbing();
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera format. (height %d)", val);
                hikvisioncam_.StartGrabbing();
                return false;
            }
            break;
        case TDT_CAMERA_FORMAT_FPS:
            if (val == 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetFpsDisable(true);
            } else {
                ret &= hikvisioncam_.SetFps(val);
            }
            if (ret) {
                format_.fps = val;
                hikvisioncam_.StartGrabbing();
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera format. (fps %d)", val);
                hikvisioncam_.StartGrabbing();
                return false;
            }
            break;
        default:
            TDT_ERROR("Failed to set hikvision camera format. (TDT_CAMERA_SETTING : %d)", tdt_camera_format);
            return false;
            break;
        }
    }

    bool HikvisionCam::Set(tdtcamera::Camera::TDT_CAMERA_FORMAT tdt_camera_format, float val) {
        bool ret = true;
        hikvisioncam_.CloseGrabbing();
        switch (tdt_camera_format) {
        case TDT_CAMERA_FORMAT_PIXEL:
            if ((unsigned int)val == 0) {
                ret &= hikvisioncam_.SetPixelformat(PixelType_Gvsp_RGB8_Packed);
            } else if ((unsigned int)val == 1) {
                ret &= hikvisioncam_.SetPixelformat(PixelType_Gvsp_BayerRG8);
            } else {
                TDT_ERROR("Failed to set hikvision camera format. (incorrect pixel_format %d)", (unsigned int)val);
                hikvisioncam_.StartGrabbing();
                return false;
            }
            if (ret) {
                format_.pixel_format = (unsigned int)val;
                hikvisioncam_.StartGrabbing();
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera format. (pixel_format %d)", (unsigned int)val);
                hikvisioncam_.StartGrabbing();
                return false;
            }
            break;
        case TDT_CAMERA_FORMAT_WIDTH:
            ret &= hikvisioncam_.SetResolution((unsigned int)val, format_.height);
            if (ret) {
                format_.width = (unsigned int)val;
                hikvisioncam_.StartGrabbing();
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera format. (width %d)", (unsigned int)val);
                hikvisioncam_.StartGrabbing();
                return false;
            }
            break;
        case TDT_CAMERA_FORMAT_HEIGHT:
            ret &= hikvisioncam_.SetResolution(format_.width, (unsigned int)val);
            if (ret) {
                format_.height = (unsigned int)val;
                hikvisioncam_.StartGrabbing();
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera format. (height %d)", (unsigned int)val);
                hikvisioncam_.StartGrabbing();
                return false;
            }
            break;
        case TDT_CAMERA_FORMAT_FPS:
            if (val < 0) {
                ret &= hikvisioncam_.SetFpsDisable(true);
            } else {
                ret &= hikvisioncam_.SetFps(val);
            }
            if (ret) {
                format_.fps = val;
                hikvisioncam_.StartGrabbing();
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera format. (fps %d)", val);
                hikvisioncam_.StartGrabbing();
                return false;
            }
            break;
        default:
            TDT_ERROR("Failed to set hikvision camera format. (TDT_CAMERA_SETTING : %d)", tdt_camera_format);
            return false;
            break;
        }
    }

    bool HikvisionCam::Set(tdtcamera::Camera::TDT_CAMERA_SETTING tdt_camera_setting, unsigned int val) {
        bool ret = true;
        switch (tdt_camera_setting) {
        case TDT_CAMERA_SETTING_EXPOSURE:
            if (val == 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetExposureAuto(true);
            } else {
                ret &= hikvisioncam_.SetExposureAuto(false);
                ret &= hikvisioncam_.SetExposure(val);
            }
            if (ret) {
                setting_.exposure = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (exposure : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_GAIN:
            if (val == 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetGainAuto(true);
            } else {
                ret &= hikvisioncam_.SetGainAuto(false);
                ret &= hikvisioncam_.SetGain(val);
            }
            if (ret) {
                setting_.gain = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (gain : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_BRIGHTNESS:
            if (val != 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetBrightness(val);
            }
            if (ret) {
                setting_.brightness = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (brightness : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_BALANCE_RED:
            if (val == 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetWhitebalanceAuto(true);
            } else if (setting_.balance_green != 0xFFFFFFFF && setting_.balance_blue != 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetWhitebalanceAuto(false);
                ret &= hikvisioncam_.SetWhitebalance(val, setting_.balance_green, setting_.balance_blue);
            }
            if (ret) {
                setting_.balance_red = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (balance_red : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_BALANCE_GREEN:
            if (val == 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetWhitebalanceAuto(true);
            } else if (setting_.balance_red != 0xFFFFFFFF && setting_.balance_blue != 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetWhitebalanceAuto(false);
                ret &= hikvisioncam_.SetWhitebalance(setting_.balance_red, val, setting_.balance_blue);
            }
            if (ret) {
                setting_.balance_green = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (balance_green : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_BALANCE_BLUE:
            if (val == 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetWhitebalanceAuto(true);
            } else if (setting_.balance_green != 0xFFFFFFFF && setting_.balance_red != 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetWhitebalanceAuto(false);
                ret &= hikvisioncam_.SetWhitebalance(setting_.balance_red, setting_.balance_green, val);
            }
            if (ret) {
                setting_.balance_blue = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (balance_blue : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_HUE:
            if (val == 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetHueDisable(true);
            } else {
                ret &= hikvisioncam_.SetHueDisable(false);
                ret &= hikvisioncam_.SetHue(val);
            }
            if (ret) {
                setting_.hue = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (hue : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_SATURATION:
            if (val == 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetSaturationDisable(true);
            } else {
                ret &= hikvisioncam_.SetSaturationDisable(false);
                ret &= hikvisioncam_.SetSaturation(val);
            }
            if (ret) {
                setting_.saturation = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (saturation : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_GAMMA:
            if (val == 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetGammaDisable(true);
            } else {
                ret &= hikvisioncam_.SetGammaDisable(true);
                ret &= hikvisioncam_.SetGammaDisable(false);
                if (val < 1) {
                    ret &= hikvisioncam_.SetGamma(val, MV_GAMMA_SELECTOR_USER);
                } else {
                    ret &= hikvisioncam_.SetGamma();
                }
            }
            if (ret) {
                setting_.gamma = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (gamma : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_SHARPNESS:
            if (val == 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetSharpnessDisable(true);
            } else {
                ret &= hikvisioncam_.SetSharpnessDisable(false);
                ret &= hikvisioncam_.SetSharpness(val);
            }
            if (ret) {
                setting_.sharpness = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (sharpness : %d)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_BLACK_LEVEL:
            if (val == 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetBlacklevelDisable(true);
            } else {
                ret &= hikvisioncam_.SetBlacklevelDisable(false);
                ret &= hikvisioncam_.SetBlacklevel(val);
            }
            if (ret) {
                setting_.black_level = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (black_level : %d)", val);
                return false;
            }
            break;
        default:
            TDT_ERROR("Failed to set hikvision camera setting. (TDT_CAMERA_SETTING : %d)", tdt_camera_setting);
            return false;
            break;
        }
    }

    bool HikvisionCam::Set(tdtcamera::Camera::TDT_CAMERA_SETTING tdt_camera_setting, float val) {
        bool ret = true;
        switch (tdt_camera_setting) {
        case TDT_CAMERA_SETTING_EXPOSURE:
            if (val < 0) {
                ret &= hikvisioncam_.SetExposureAuto(true);
            } else {
                ret &= hikvisioncam_.SetExposureAuto(false);
                ret &= hikvisioncam_.SetExposure(val);
            }
            if (ret) {
                setting_.exposure = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (exposure : %f)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_GAIN:
            if (val < 0) {
                ret &= hikvisioncam_.SetGainAuto(true);
            } else {
                ret &= hikvisioncam_.SetGainAuto(false);
                ret &= hikvisioncam_.SetGain(val);
            }
            if (ret) {
                setting_.gain = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (gain : %f)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_BRIGHTNESS:
            if (val < 0) {
                ret &= hikvisioncam_.SetBrightness((unsigned int)val);
            }
            if (ret) {
                setting_.brightness = (unsigned int)val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (brightness : %f)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_BALANCE_RED:
            if (val < 0) {
                ret &= hikvisioncam_.SetWhitebalanceAuto(true);
            } else if (setting_.balance_green != 0xFFFFFFFF || setting_.balance_blue != 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetWhitebalanceAuto(false);
                ret &= hikvisioncam_.SetWhitebalance((unsigned int)val, setting_.balance_green, setting_.balance_blue);
            }
            if (ret) {
                setting_.balance_red = (unsigned int)val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (balance_red : %f)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_BALANCE_GREEN:
            if (val < 0) {
                ret &= hikvisioncam_.SetWhitebalanceAuto(true);
            } else if (setting_.balance_red != 0xFFFFFFFF || setting_.balance_blue != 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetWhitebalanceAuto(false);
                ret &= hikvisioncam_.SetWhitebalance(setting_.balance_red, (unsigned int)val, setting_.balance_blue);
            }
            if (ret) {
                setting_.balance_green = (unsigned int)val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (balance_green : %f)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_BALANCE_BLUE:
            if (val < 0) {
                ret &= hikvisioncam_.SetWhitebalanceAuto(true);
            } else if (setting_.balance_green != 0xFFFFFFFF || setting_.balance_red != 0xFFFFFFFF) {
                ret &= hikvisioncam_.SetWhitebalanceAuto(false);
                ret &= hikvisioncam_.SetWhitebalance(setting_.balance_red, setting_.balance_green, (unsigned int)val);
            }
            if (ret) {
                setting_.balance_blue = (unsigned int)val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (balance_blue : %f)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_HUE:
            if (val < 0) {
                ret &= hikvisioncam_.SetHueDisable(true);
            } else {
                ret &= hikvisioncam_.SetHueDisable(false);
                ret &= hikvisioncam_.SetHue((unsigned int)val);
            }
            if (ret) {
                setting_.hue = (unsigned int)val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (hue : %f)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_SATURATION:
            if (val < 0) {
                ret &= hikvisioncam_.SetSaturationDisable(true);
            } else {
                ret &= hikvisioncam_.SetSaturationDisable(false);
                ret &= hikvisioncam_.SetSaturation((unsigned int)val);
            }
            if (ret) {
                setting_.saturation = (unsigned int)val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (saturation : %f)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_GAMMA:
            if (val < 0) {
                ret &= hikvisioncam_.SetGammaDisable(true);
            } else {
                ret &= hikvisioncam_.SetGammaDisable(true);
                ret &= hikvisioncam_.SetGammaDisable(false);
                if (val < 1) {
                    ret &= hikvisioncam_.SetGamma(val, MV_GAMMA_SELECTOR_USER);
                } else {
                    ret &= hikvisioncam_.SetGamma();
                }
            }
            if (ret) {
                setting_.gamma = val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (gamma : %f)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_SHARPNESS:
            if (val < 0) {
                ret &= hikvisioncam_.SetSharpnessDisable(true);
            } else {
                ret &= hikvisioncam_.SetSharpnessDisable(false);
                ret &= hikvisioncam_.SetSharpness((unsigned int)val);
            }
            if (ret) {
                setting_.sharpness = (unsigned int)val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (sharpness : %f)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_BLACK_LEVEL:
            if (val < 0) {
                ret &= hikvisioncam_.SetBlacklevelDisable(true);
            } else {
                ret &= hikvisioncam_.SetBlacklevelDisable(false);
                ret &= hikvisioncam_.SetBlacklevel((unsigned int)val);
            }
            if (ret) {
                setting_.black_level = (unsigned int)val;
                return true;
            } else {
                TDT_ERROR("Failed to set hikvision camera setting. (black_level : %f)", val);
                return false;
            }
            break;
        case TDT_CAMERA_SETTING_TRIGER_IMAGE:
            if (val == 0.0f) {
                ret &= hikvisioncam_.SetTrigger(false);
            } else {
                ret &= hikvisioncam_.SetTrigger(true);
            }
            break;
        default:
            TDT_ERROR("Failed to set hikvision camera setting. (TDT_CAMERA_SETTING : %d)", tdt_camera_setting);
            return false;
            break;
        }
    }

    void HikvisionCam::TakeFrame() {

        while (true) {
            std::unique_lock<std::mutex> locker_(stop_mtx_);
            if(Stop) {
                std::cout<<"stop"<<std::endl;
                break;
            }
            locker_.unlock();
            if (!hikvisioncam_.GetMat(takeImg_, takeTime_)) {
                TDT_ERROR("Failed to get image from hikvision cam");
            }
            std::unique_lock<std::mutex> locker(img_mtx_);
            swap(takeImg_, swapImg_);
            swapTime_ = takeTime_;
            locker.unlock();
            condVar_.notify_all();
        }
    }

    bool HikvisionCam::GetImage(tdtcamera::TImage &timage) {
        std::unique_lock<std::mutex> locker(img_mtx_, std::try_to_lock);
        cv::Mat                      tmp1;
        swap(tmp1, srcImg_);
        if (swapImg_.empty())
            return 0;
        // condVar_.wait(locker);
        swap(srcImg_, swapImg_);
        srcTime_ = swapTime_;
        // locker.unlock();
        timage = TImage(srcImg_, ++seq_);
    }

    bool HikvisionCam::RestartCamera() { return hikvisioncam_.RestartCamera(); }

    VideoDebug::VideoDebug(std::string config_path) : Camera(config_path) {
        if (!capture_.open(identity_.dev_path)) {
            char path[25];
            sprintf(path, "../videodebug/%d.avi", identity_.dev_index);
            if (!capture_.open(path)) {
                sprintf(path, "../videodebug/%s", identity_.dev_path.c_str());
                if (!capture_.open(path)) {
                    TDT_ERROR("open video file fail!\n");
                }
            }
        }
        if (capture_.isOpened()) {
            std::thread t(&VideoDebug::TakeFrame, this);
            t.detach();
        }
    }

    void VideoDebug::TakeFrame() {
        while (true) {
            // 虚拟摄像头特有: 一帧一帧读
            if (swapImg_.empty()) {
                capture_ >> takeImg_;

                std::unique_lock<std::mutex> locker(img_mtx_);
                swap(takeImg_, swapImg_);
                locker.unlock();
                condVar_.notify_all();
            }
        }
    }

    bool VideoDebug::GetImage(tdtcamera::TImage &timage) {
        std::unique_lock<std::mutex> locker(img_mtx_, std::try_to_lock);
        cv::Mat                      tmp;
        swap(tmp, srcImg_);
        if (swapImg_.empty())
            return 0;
        // condVar_.wait(locker);
        swap(srcImg_, swapImg_);

        // locker.unlock();
        timage = TImage(srcImg_, ++seq_);
    }

} // namespace tdtcamera

namespace YAML {
    std::map<char, int> mat_type2int     = {{'u', 0}, {'c', 1}, {'w', 2}, {'s', 3}, {'i', 4}, {'f', 5}, {'d', 6}, {'r', 7}};
    char                mat_type2char[8] = {'u', 'c', 'w', 's', 'i', 'f', 'd', 'r'};

    template <typename T> void *MatDataEncode(const Node &node, cv::Mat rhs, int buf_size) {
        T *data = rhs.data;
        for (int i = 0; i < buf_size; i++) {
            node["data"][i] = data[i];
        }
    }

    template <typename T> void *MatDataDecode(const Node &node, int buf_size) {
        T *data = (T *)(malloc(sizeof(T) * buf_size));
        for (int i = 0; i < buf_size; i++) {
            data[i] = node["data"][i].as<T>();
        }
        return data;
    }

    template <> struct convert<cv::Mat> {
        static Node encode(const cv::Mat rhs) {
            Node node;
            int  buf_size = rhs.rows * rhs.cols;
            node["rows"]  = rhs.rows;
            node["cols"]  = rhs.cols;
            node["dt"]    = mat_type2char[rhs.type()];

            return node;
        }

        static bool decode(const Node &node, cv::Mat &rhs) {
            if (!node.IsMap() || node.size() != 4) {
                return false;
            }
            int   rows     = node["rows"].as<int>();
            int   cols     = node["cols"].as<int>();
            auto  it       = mat_type2int.find(node["dt"].as<char>());
            int   dt       = it->second;
            int   buf_size = rows * cols;
            void *data     = nullptr;
            switch (dt) {
            case 0:
                data = MatDataDecode<uchar>(node, buf_size);
                break;
            case 1: {
                data = MatDataDecode<char>(node, buf_size);
                break;
            }
            case 2: {
                data = MatDataDecode<ushort>(node, buf_size);
                break;
            }
            case 3: {
                data = MatDataDecode<short>(node, buf_size);
                break;
            }
            case 4: {
                data = MatDataDecode<int>(node, buf_size);
                break;
            }
            case 5: {
                data = MatDataDecode<float>(node, buf_size);
                break;
            }
            case 6: {
                data = MatDataDecode<double>(node, buf_size);
                break;
            }
                //                case 7:{
                //                    data = MatDataDecode<float16_t>(node, buf_size);
                //                    break;
                //                }
            default:
                return false;
            }

            cv::Mat rhs_tmp(rows, cols, dt, data);
            rhs_tmp.copyTo(rhs);
            free(data);
            data = NULL;
            return true;
        }
    };
} // namespace YAML
