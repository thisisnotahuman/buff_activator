/*
 * @Name: tdt_hikvisioncam.cpp
 * @Description:
 * @Version: 1.0.0
 * @Author: xi
 * @Date: 2019-10-13 11:02:20
 * @LastEditors: your name
 * @LastEditTime: 2019-11-22 20:14:10
 */

#include "hikvisioncam.h"

namespace tdtbasecam {

    bool HikvisionBasicCam::InitHandle(uint32_t n_device_index) {
        MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
        n_ret_                             = MV_CC_EnumDevices(MV_USB_DEVICE, &m_stDevList); //枚举所有的相机，并且返回错误码（如果错误的话）
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_EnumDevices fail [%x]\n", n_ret_);
            return false;
        }

        if (m_stDevList.nDeviceNum == 0) {
            TDT_ERROR("No camera found!\n");
            return false;
        }

        //选择查找到的一台在线设备，创建设备句柄
        MV_CC_DEVICE_INFO m_stDevInfo = {0};
        memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[n_device_index], sizeof(MV_CC_DEVICE_INFO));
        // pDeviceInfo保存着关于所有枚举得到的设备的MV_CC_DEVICE_INFO类型的指针
        n_ret_ = MV_CC_CreateHandle(&handle_, &m_stDevInfo);

        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_CreateHandle fail [%x]\n", n_ret_);
            return false;
        }
        // TODO:啥东西来的            //这个变量是为了保存打开的相机的序号，以备重启相机时使用,函数传入的参数是由yaml导入的，默认是0,并且2021赛季的车一直是一个视觉用的摄像头
        n_device_index_ = n_device_index;
        guid_           = std::string((char *)m_stDevInfo.SpecialInfo.stUsb3VInfo.chDeviceGUID); //获取设备的guid号,是个唯一标注的序号
        return true;
    }

    bool HikvisionBasicCam::InitHandle(std::string cam_guid) { //这个函数好像是要根据给定的guid号来链接设备
        MV_CC_DEVICE_INFO_LIST m_stDevList = {0};
        n_ret_                             = MV_CC_EnumDevices(MV_USB_DEVICE, &m_stDevList);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_EnumDevices fail [%x]\n", n_ret_);
            return false;
        }

        if (m_stDevList.nDeviceNum == 0) {
            TDT_ERROR("no camera found!\n");
            return false;
        }

        //选择查找到的一台在线设备，创建设备句柄
        for (int i = 0; i < m_stDevList.nDeviceNum; i++) {
            MV_CC_DEVICE_INFO m_stDevInfo = {0};
            memcpy(&m_stDevInfo, m_stDevList.pDeviceInfo[i], sizeof(MV_CC_DEVICE_INFO));
            if (cam_guid == std::string((char *)m_stDevInfo.SpecialInfo.stUsb3VInfo.chDeviceGUID)) { //？
                n_ret_ = MV_CC_CreateHandle(&handle_, &m_stDevInfo);
                if (MV_OK != n_ret_) {
                    TDT_WARNING("MV_CC_CreateHandle fail [%x]\n", n_ret_);
                    continue;
                }
                n_device_index_ = i;
                guid_           = cam_guid;
                return true;
            }
        }
        TDT_ERROR("no camera found with guid [%s]!", cam_guid.c_str());
        return false;
    }

    bool HikvisionBasicCam::OpenCamera() {
        //注册数据回调函数，采集图像数据在回调函数中获取
        uint32_t       nAccessMode    = MV_ACCESS_Exclusive;
        unsigned short nSwitchoverKey = 0;
        //连接设备
        n_ret_ = MV_CC_OpenDevice(handle_, nAccessMode, nSwitchoverKey); //通过获得的句柄打开相机
        if (MV_OK != n_ret_) {                                           //在相机可以找到的情况下，如果相机坏了，就会报错
            TDT_ERROR("MV_CC_OpenDevice fail [%x]\n", n_ret_);
            TDT_ERROR("Camera may be broken\n");
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::CloseCamera() {
        n_ret_ = MV_CC_CloseDevice(handle_);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_CloseDevice fail [%x]\n", n_ret_);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::StartGrabbing() {
        //开始采集图像
        n_ret_ = MV_CC_StartGrabbing(handle_);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_StartGrabbing fail! n_ret_ [%x]\n", n_ret_);
            MV_CC_StopGrabbing(handle_);
            MV_CC_DestroyHandle(handle_);
            return false;
        }
        //获取一帧数据的大小
        MVCC_INTVALUE stIntvalue = {0};
        n_ret_                   = MV_CC_GetIntValue(handle_, "PayloadSize", &stIntvalue);
        if (n_ret_ != MV_OK) {
            TDT_ERROR("MV_CC_GetPayloadSize failed! n_ret_ [%x]\n", n_ret_);
            return false;
        }

        n_data_size_ = stIntvalue.nCurValue + 2048; //一帧数据大小+预留字节(用于SDK内部处理)
        //抓取一帧图片
        p_data_ = (unsigned char *)malloc(n_data_size_);

        memset(&st_img_info_, 0, sizeof(MV_FRAME_OUT_INFO_EX));

        return true;
    }

    bool HikvisionBasicCam::CloseGrabbing() {
        n_ret_ = MV_CC_StopGrabbing(handle_);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_StopGrabbing fail! n_ret_ [%x]\n", n_ret_);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::RestartCamera() {
        bool ret = true;
        TDT_INFO("===RESTART HIKVISION CAMERA===");
        ret &= CloseGrabbing();
        ret &= CloseCamera();
        n_ret_ = MV_CC_DestroyHandle(handle_);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_DestroyHandle_ fail! n_ret_ [%x]\n", n_ret_);
        }
        ret &= InitHandle(n_device_index_);
        ret &= OpenCamera();
        ret &= StartGrabbing();
        if (ret) {
            return true;
        } else {
            return false;
        }
    }
    //取帧时间过长或者取帧错误 如果内存炸掉，malloc申请出错 像素转换错误 return false
    bool HikvisionBasicCam::GetMat(cv::Mat &img, double &time) {
        /* 旧取帧法 */
        /*n_ret_ = MV_CC_GetImageForBGR(handle_, p_data_, n_data_size_, &st_img_info_, 1000);
        struct timeval get_mat_time;
        gettimeofday(&get_mat_time,NULL);
        cv::Mat frame(st_img_info_.nHeight, st_img_info_.nWidth, CV_8UC3, p_data_);//读入图片到frame
        img=frame;//赋给Camera_Output，注意img和frame共享一个矩阵
        if (save_res_ == 1){
            OutputVideo << img;
        }
        return true;*/
    OPEN_LOOP:
        // double startTime = cv::getTickCount() / cv::getTickFrequency()* 1000000;

        n_ret_ = MV_CC_SetCommandValue(handle_, "TriggerSoftware");
        if (MV_OK != n_ret_) {
            TDT_ERROR("failed in TriggerSoftware[%x]\n", n_ret_);
        }
        double triggerTime  = (cv::getTickCount() / cv::getTickFrequency()) * 1000000;
        int    RestartTimes = 1;
        n_ret_              = MV_CC_GetOneFrameTimeout(handle_, p_data_, n_data_size_, &st_img_info_, 1000);
        if (n_ret_ != MV_OK) {                                                   //如果取一帧超过时间，或者取帧错误，都不对
            TDT_ERROR("MV_CC_GetOneFrameTimeout failed! n_ret_ [%x]\n", n_ret_); //如果卡帧就报错（现在估计是温度高的问题）
            RestartCamera();
            TDT_WARNING("the %d times !   Get Frame Error , will restart the camera \n", RestartTimes);
            ++RestartTimes;
            if (RestartTimes < 5)
                goto OPEN_LOOP; // 5为重启相机的次数，如果重启多次解决不了问题，就重启程序
            else {
                TDT_ERROR("Restart ERROR, restart this executable\n");
                exit(-1);
            }
            return false;
        }
        MVCC_FLOATVALUE exposeTime = {-1, -1, -1};
        n_ret_                     = MV_CC_GetFloatValue(handle_, "ExposureTime", &exposeTime);
        if (MV_OK != n_ret_) {
            TDT_ERROR("failed in GetExposedTime[%x]\n", n_ret_);
        }
        // std::cout << "exposed time=" << exposeTime.fCurValue << std::endl;
        // double recieveTime = cv::getTickCount() / cv::getTickFrequency()* 1000000;
        // std::cout << " triggerTime=" << (triggerTime - startTime)  << " recieveTime=" << (recieveTime - triggerTime) << std::endl;
        MV_CC_PIXEL_CONVERT_PARAM StParam = {0};
        memset(&StParam, 0, sizeof(MV_CC_PIXEL_CONVERT_PARAM));
        //源数据
        StParam.pSrcData       = p_data_;                  //原始图像数据
        StParam.nSrcDataLen    = st_img_info_.nFrameLen;   //原始图像数据长度
        StParam.enSrcPixelType = st_img_info_.enPixelType; //原始图像数据的像素格式
        StParam.nWidth         = st_img_info_.nWidth;      //图像宽
        StParam.nHeight        = st_img_info_.nHeight;     //图像高
        //目标数据pImage
        StParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;                            //需要保存的像素格式类型，转换成BGR格式
        StParam.nDstBufferSize = st_img_info_.nWidth * st_img_info_.nHeight * 4 + 2048; //存储节点的大小
        unsigned char *p_image = (unsigned char *)malloc(st_img_info_.nWidth * st_img_info_.nHeight * 4 + 2048);
        if (p_image == NULL) {
            return false;
        }
        StParam.pDstBuffer = p_image; //输出数据缓冲区，存放转换之后的数据

        n_ret_ = MV_CC_ConvertPixelType(handle_, &StParam); //转换像素
        if (n_ret_ != MV_OK) {
            TDT_ERROR("MV_CC_ConvertPixelType failed! n_ret_ [%x]\n", n_ret_);
            return false;
        }
        cv::Mat frame(st_img_info_.nHeight, st_img_info_.nWidth, CV_8UC3, p_image);

        if (open_lut_) {
            LUT(frame, lut_, img);
        } else {
            frame.copyTo(img);
        }
        time = triggerTime + exposeTime.fCurValue / 2;
        free(p_image);
        p_image = NULL;
        return true;
    }

    bool HikvisionBasicCam::SetExposureAuto(bool if_auto) {
        if (if_auto) {
            n_ret_ = MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_CONTINUOUS);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetExposureAuto fail! n_ret_ [%x]\n", n_ret_);
                MVCC_ENUMVALUE CurrentValue = {0};
                n_ret_                      = MV_CC_GetEnumValue(handle_, "ExposureAuto", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
                return false;
            }
        } else {
            n_ret_ = MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetExposureAuto fail! n_ret_ [%x]\n", n_ret_);
                MVCC_ENUMVALUE CurrentValue = {0};
                n_ret_                      = MV_CC_GetEnumValue(handle_, "ExposureAuto", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
                return false;
            }
        }
        return true;
    }

    bool HikvisionBasicCam::SetExposure(float val) {
        n_ret_ = MV_CC_SetFloatValue(handle_, "ExposureTime", val);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetExposureTime fail! n_ret_ [%x]\n", n_ret_);
            MVCC_FLOATVALUE CurrentValue = {0};
            n_ret_                       = MV_CC_GetFloatValue(handle_, "ExposureTime", &CurrentValue);
            TDT_INFO("current value: %f\n", CurrentValue.fCurValue);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::DoExposure() {
        n_ret_ = MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_ONCE);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetExposureAutoMode fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_                      = MV_CC_GetEnumValue(handle_, "ExposureAuto", &CurrentValue);
            TDT_ERROR("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::SetGainAuto(bool if_auto) {
        if (if_auto) {
            n_ret_ = MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_CONTINUOUS);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetGainAuto fail! n_ret_ [%x]\n", n_ret_);
                MVCC_ENUMVALUE CurrentValue = {0};
                n_ret_                      = MV_CC_GetEnumValue(handle_, "GainAuto", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
                return false;
            }
        } else {
            n_ret_ = MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetGainAuto fail! n_ret_ [%x]\n", n_ret_);
                MVCC_ENUMVALUE CurrentValue = {0};
                n_ret_                      = MV_CC_GetEnumValue(handle_, "GainAuto", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
                return false;
            }
        }
        return true;
    }

    bool HikvisionBasicCam::SetGain(float val) {
        n_ret_ = MV_CC_SetFloatValue(handle_, "Gain", val);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetGain fail! n_ret_ [%x]\n", n_ret_);
            MVCC_FLOATVALUE CurrentValue = {0};
            n_ret_                       = MV_CC_GetFloatValue(handle_, "Gain", &CurrentValue);
            TDT_INFO("current value: %f\n", CurrentValue.fCurValue);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::SetBrightness(uint32_t val) {
        n_ret_ = MV_CC_SetIntValue(handle_, "Brightness", val);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetBrightness fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_                     = MV_CC_GetIntValue(handle_, "Brightness", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::SetWhitebalanceAuto(bool if_auto) {
        if (if_auto) {
            n_ret_ = MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetBalanceWhiteAuto fail! n_ret_ [%x]\n", n_ret_);
                MVCC_ENUMVALUE CurrentValue = {0};
                n_ret_                      = MV_CC_GetEnumValue(handle_, "BalanceWhiteAuto", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
                return false;
            }
        } else {
            n_ret_ = MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_OFF);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetBalanceWhiteAuto fail! n_ret_ [%x]\n", n_ret_);
                MVCC_ENUMVALUE CurrentValue = {0};
                n_ret_                      = MV_CC_GetEnumValue(handle_, "BalanceWhiteAuto", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
                return false;
            }
        }
        return true;
    }

    bool HikvisionBasicCam::SetWhitebalance(uint32_t r, uint32_t g, uint32_t b) {
        n_ret_ = MV_CC_SetEnumValue(handle_, "BalanceRatioSelector", 0);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetBalanceRatioSelector fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_                      = MV_CC_GetEnumValue(handle_, "BalanceRatioSelector", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }
        n_ret_ = MV_CC_SetIntValue(handle_, "BalanceRatio", r);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetBalanceRatioRed fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_                     = MV_CC_GetIntValue(handle_, "BalanceRatio", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }

        n_ret_ = MV_CC_SetEnumValue(handle_, "BalanceRatioSelector", 1);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetBalanceRatioSelector fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_                      = MV_CC_GetEnumValue(handle_, "BalanceRatioSelector", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }
        n_ret_ = MV_CC_SetIntValue(handle_, "BalanceRatio", g);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetBalanceRatioGreen fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_                     = MV_CC_GetIntValue(handle_, "BalanceRatio", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }

        n_ret_ = MV_CC_SetEnumValue(handle_, "BalanceRatioSelector", 2);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetBalanceRatioSelector fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_                      = MV_CC_GetEnumValue(handle_, "BalanceRatioSelector", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }
        n_ret_ = MV_CC_SetIntValue(handle_, "BalanceRatio", b);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetBalanceRatioBlue fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_                     = MV_CC_GetIntValue(handle_, "BalanceRatio", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::DoWhitebalance() {
        n_ret_ = MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_ONCE);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_DoBalanceWhite fail! n_ret_ [%x]\n", n_ret_);
            MVCC_ENUMVALUE CurrentValue = {0};
            n_ret_                      = MV_CC_GetEnumValue(handle_, "BalanceWhiteAuto", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::SetHueDisable(bool if_disable) {
        if (if_disable) {
            n_ret_ = MV_CC_SetBoolValue(handle_, "HueEnable", false);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetHueEnable fail! n_ret_ [%x]\n", n_ret_);
                bool CurrentValue;
                n_ret_ = MV_CC_GetBoolValue(handle_, "HueEnable", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue);
                return false;
            }
        } else {
            n_ret_ = MV_CC_SetBoolValue(handle_, "HueEnable", true);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetHueEnable fail! n_ret_ [%x]\n", n_ret_);
                bool CurrentValue;
                n_ret_ = MV_CC_GetBoolValue(handle_, "HueEnable", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue);
                return false;
            }
        }
        return true;
    }

    bool HikvisionBasicCam::SetHue(uint32_t val) {
        n_ret_ = MV_CC_SetIntValue(handle_, "Hue", val);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetHue fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_                     = MV_CC_GetIntValue(handle_, "Hue", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::SetSaturationDisable(bool if_disable) {
        if (if_disable) {
            n_ret_ = MV_CC_SetBoolValue(handle_, "SaturationEnable", false);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetSaturationEnable fail! n_ret_ [%x]\n", n_ret_);
                bool CurrentValue;
                n_ret_ = MV_CC_GetBoolValue(handle_, "SaturationEnable", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue);
                return false;
            }
        } else {
            n_ret_ = MV_CC_SetBoolValue(handle_, "SaturationEnable", true);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetSaturationEnable fail! n_ret_ [%x]\n", n_ret_);
                bool CurrentValue;
                n_ret_ = MV_CC_GetBoolValue(handle_, "SaturationEnable", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue);
                return false;
            }
        }
        return true;
    }

    bool HikvisionBasicCam::SetSaturation(uint32_t val) {
        n_ret_ = MV_CC_SetIntValue(handle_, "Saturation", val);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetSaturation fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_                     = MV_CC_GetIntValue(handle_, "Saturation", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::SetGammaDisable(bool if_disable) {
        MVCC_ENUMVALUE CurrentValue = {0};
        n_ret_                      = MV_CC_GetEnumValue(handle_, "PixelFormat", &CurrentValue);

        if (if_disable) {
            if (CurrentValue.nCurValue == PixelType_Gvsp_BayerRG8) {
                SetLut(-1);
            } else {
                n_ret_ = MV_CC_SetBoolValue(handle_, "GammaEnable", false);
                if (MV_OK != n_ret_) {
                    TDT_ERROR("MV_CC_SetGammaEnable fail! n_ret_ [%x]\n", n_ret_);
                    bool CurrentValue;
                    n_ret_ = MV_CC_GetBoolValue(handle_, "GammaEnable", &CurrentValue);
                    TDT_INFO("current value: %d\n", CurrentValue);
                    return false;
                }
            }
        } else {
            if (CurrentValue.nCurValue != PixelType_Gvsp_BayerRG8) {
                n_ret_ = MV_CC_SetBoolValue(handle_, "GammaEnable", true);
                if (MV_OK != n_ret_) {
                    TDT_ERROR("MV_CC_SetGammaEnable fail! n_ret_ [%x]\n", n_ret_);
                    bool CurrentValue;
                    n_ret_ = MV_CC_GetBoolValue(handle_, "GammaEnable", &CurrentValue);
                    TDT_INFO("current value: %d\n", CurrentValue);
                    return false;
                }
            }
        }
        return true;
    }

    bool HikvisionBasicCam::SetGamma(float val, unsigned char selector) {
        MVCC_ENUMVALUE CurrentValue = {0};
        n_ret_                      = MV_CC_GetEnumValue(handle_, "PixelFormat", &CurrentValue);
        if (CurrentValue.nCurValue == PixelType_Gvsp_BayerRG8) {
            SetLut(val);
        } else {
            if (selector == MV_GAMMA_SELECTOR_SRGB) {
                n_ret_ = MV_CC_SetEnumValue(handle_, "GammaSelector", MV_GAMMA_SELECTOR_SRGB);
                if (MV_OK != n_ret_) {
                    TDT_ERROR("MV_CC_SetGammaSelector fail! n_ret_ [%x]\n", n_ret_);
                    MVCC_ENUMVALUE CurrentValue = {0};
                    n_ret_                      = MV_CC_GetEnumValue(handle_, "GammaSelector", &CurrentValue);
                    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
                    return false;
                }
            } else {
                n_ret_ = MV_CC_SetEnumValue(handle_, "GammaSelector", MV_GAMMA_SELECTOR_USER);
                if (MV_OK != n_ret_) {
                    TDT_ERROR("MV_CC_SetGammaSelector fail! n_ret_ [%x]\n", n_ret_);
                    MVCC_ENUMVALUE CurrentValue = {0};
                    n_ret_                      = MV_CC_GetEnumValue(handle_, "GammaSelector", &CurrentValue);
                    TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
                    return false;
                }
                n_ret_ = MV_CC_SetFloatValue(handle_, "Gamma", val);
                if (MV_OK != n_ret_) {
                    TDT_ERROR("MV_CC_SetGamma fail! n_ret_ [%x]\n", n_ret_);
                    MVCC_FLOATVALUE CurrentValue = {0};
                    n_ret_                       = MV_CC_GetFloatValue(handle_, "Gamma", &CurrentValue);
                    TDT_INFO("current value: %f\n", CurrentValue.fCurValue);
                    return false;
                }
            }
        }
        return true;
    }

    bool HikvisionBasicCam::SetSharpnessDisable(bool if_disable) {
        if (if_disable) {
            n_ret_ = MV_CC_SetBoolValue(handle_, "SharpnessEnable", false);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetSharpnessEnable fail! n_ret_ [%x]\n", n_ret_);
                bool CurrentValue;
                n_ret_ = MV_CC_GetBoolValue(handle_, "SharpnessEnable", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue);
                return false;
            }
        } else {
            n_ret_ = MV_CC_SetBoolValue(handle_, "SharpnessEnable", true);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetSharpnessEnable fail! n_ret_ [%x]\n", n_ret_);
                bool CurrentValue;
                n_ret_ = MV_CC_GetBoolValue(handle_, "SharpnessEnable", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue);
                return false;
            }
        }
        return true;
    }

    bool HikvisionBasicCam::SetSharpness(uint32_t val) {
        n_ret_ = MV_CC_SetIntValue(handle_, "Sharpness", val);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetSharpness fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_                     = MV_CC_GetIntValue(handle_, "Sharpness", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::SetBlacklevelDisable(bool if_disable) {
        if (if_disable) {
            n_ret_ = MV_CC_SetBoolValue(handle_, "BlackLevelEnable", false);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetBlackLevelEnable fail! n_ret_ [%x]\n", n_ret_);
                bool CurrentValue;
                n_ret_ = MV_CC_GetBoolValue(handle_, "BlackLevelEnable", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue);
                return false;
            }
        } else {
            n_ret_ = MV_CC_SetBoolValue(handle_, "BlackLevelEnable", true);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetBlackLevelEnable fail! n_ret_ [%x]\n", n_ret_);
                bool CurrentValue;
                n_ret_ = MV_CC_GetBoolValue(handle_, "BlackLevelEnable", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue);
                return false;
            }
        }
        return true;
    }

    bool HikvisionBasicCam::SetBlacklevel(uint32_t val) {
        n_ret_ = MV_CC_SetIntValue(handle_, "BlackLevel", val);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetBlackLevel fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_                     = MV_CC_GetIntValue(handle_, "BlackLevel", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::SetResolution(uint32_t width, uint32_t height) {
        n_ret_ = MV_CC_SetIntValue(handle_, "Width", width);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetWidth fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_                     = MV_CC_GetIntValue(handle_, "Width", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }

        n_ret_ = MV_CC_SetIntValue(handle_, "Height", height);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetHeight fail! n_ret_ [%x]\n", n_ret_);
            MVCC_INTVALUE CurrentValue = {0};
            n_ret_                     = MV_CC_GetIntValue(handle_, "Height", &CurrentValue);
            TDT_INFO("current value: %d\n", CurrentValue.nCurValue);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::SetFpsDisable(bool if_disable) {
        if (if_disable) {
            n_ret_ = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", false);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetAcquisitionFrameRateEnable fail! n_ret_ [%x]\n", n_ret_);
                bool CurrentValue;
                n_ret_ = MV_CC_GetBoolValue(handle_, "AcquisitionFrameRateEnable", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue);
                return false;
            }
        } else {
            n_ret_ = MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetAcquisitionFrameRateEnable fail! n_ret_ [%x]\n", n_ret_);
                bool CurrentValue;
                n_ret_ = MV_CC_GetBoolValue(handle_, "AcquisitionFrameRateEnable", &CurrentValue);
                TDT_INFO("current value: %d\n", CurrentValue);
                return false;
            }
        }
        return true;
    }

    bool HikvisionBasicCam::SetFps(float fps) {
        n_ret_ = MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", fps);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetAcquisitionFrameRate fail! n_ret_ [%x]\n", n_ret_);
            MVCC_FLOATVALUE CurrentValue = {0};
            n_ret_                       = MV_CC_GetFloatValue(handle_, "AcquisitionFrameRate", &CurrentValue);
            TDT_INFO("current value: %f\n", CurrentValue.fCurValue);
            return false;
        }
        return true;
    }

    bool HikvisionBasicCam::SetTrigger(bool isTrigger) {
        if (!isTrigger) {
            n_ret_ = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
            if (MV_OK != n_ret_) {
                TDT_ERROR("MV_CC_SetTriggerMode fail! nRet [%x]\n", n_ret_);
                return false;
            }
            return true;
        }
        // 设置触发模式为on
        // set trigger mode as on
        n_ret_ = MV_CC_SetEnumValue(handle_, "TriggerMode", 1);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetTriggerMode fail! nRet [%x]\n", n_ret_);
            return false;
        }

        // 设置触发源
        // set trigger source
        n_ret_ = MV_CC_SetEnumValue(handle_, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetTriggerSource fail! nRet [%x]\n", n_ret_);
            return false;
        }
        TDT_INFO("Trigger Mode on");
        return true;
    }

    bool HikvisionBasicCam::SetPixelformat(uint32_t pixelformat) {
        n_ret_ = MV_CC_SetEnumValue(handle_, "PixelFormat", pixelformat); //设定像素的类型,TODO:但是没有看懂像素的两个类型是什么
        if (MV_OK != n_ret_) {
            TDT_ERROR("MV_CC_SetPixelFormat fail! n_ret_ [%x]\n", n_ret_);
            return false;
        }
        return true;
    }

    void HikvisionBasicCam::SetLut(float val, unsigned char mode) {
        if (val < 0) {
            open_lut_ = false;
        } else {
            lut_.create(1, 256, CV_8UC1);
            cv::Mat_<uchar> table = lut_;
            switch (mode) {
            case 0:
                for (int i = 0; i < 256; i++) {
                    double f;
                    f           = (i + 0.5F) / 256;
                    f           = pow(f, val);
                    table(0, i) = (f * 256 - 0.5F);
                }
                open_lut_ = true;
                break;
            case 1:
                for (int i = 0; i < val; i++) {

                    table(0, i) = 255 * i / val;
                }
                for (int i = val; i < 256; i++) {

                    table(0, i) = 255;
                }
                open_lut_ = true;
                break;
            }
        }
    }

} // namespace tdtbasecam