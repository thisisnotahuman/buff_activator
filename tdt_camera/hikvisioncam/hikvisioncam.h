/*
 * @Name: file name
 * @Description:
 * @Version: 1.0.0.1
 * @Author: your name
 * @Date: 2019-10-13 11:05:52
 * @LastEditors: your name
 * @LastEditTime: 2019-11-17 10:15:48
 */

#ifndef LIB_TDTCAMERA_TDTHIKVISIONCAM_H
#define LIB_TDTCAMERA_TDTHIKVISIONCAM_H

#include "libhikvision/ImageProcess/MvCameraControl.h"
#include "libhikvision/ImageProcess/MvGigEDevice.h"
#include <iostream>
#include <opencv2/opencv.hpp>

#include "tdtcommon.h"

namespace tdtbasecam {

    class HikvisionBasicCam {
    public:
        explicit HikvisionBasicCam() = default;
        ~HikvisionBasicCam()         = default;

        bool InitHandle(uint32_t nDeviceIndex); //设备连接
        bool InitHandle(std::string cam_guid);  //设备连接//为什么要两个版本  //这个版本是要打开guid制定的设备，但是

        bool OpenCamera();
        bool CloseCamera();

        bool StartGrabbing(); //采集流
        bool CloseGrabbing();

        bool RestartCamera();

        bool GetMat(cv::Mat &img, double &time); // 获取Opencv Mat图像

        bool SetExposureAuto(bool if_auto);
        bool SetExposure(float val); // 设置曝光
        bool DoExposure();           // 单帧自动曝光
        bool SetGainAuto(bool if_auto);
        bool SetGain(float val);          // 设置增益
        bool SetBrightness(uint32_t val); // 设置亮度

        bool SetWhitebalanceAuto(bool if_auto);
        bool SetWhitebalance(uint32_t r, uint32_t g, uint32_t b); // 设置白平衡
        bool DoWhitebalance();                                    // 单帧自动白平衡
        bool SetHueDisable(bool if_disable);
        bool SetHue(uint32_t val); // 设置色调
        bool SetSaturationDisable(bool if_disable);
        bool SetSaturation(uint32_t val); // 设置饱和度

        bool SetGammaDisable(bool if_disable);
        bool SetGamma(float val = 1. / 2.2, unsigned char selector = MV_GAMMA_SELECTOR_SRGB); // 设置伽玛
        bool SetSharpnessDisable(bool if_disable);
        bool SetSharpness(uint32_t val); // 设置锐度
        bool SetBlacklevelDisable(bool if_disable);
        bool SetBlacklevel(uint32_t val); // 设置黑位

        bool SetPixelformat(uint32_t pixelformat = PixelType_Gvsp_RGB8_Packed);
        bool SetResolution(uint32_t width, uint32_t height);
        bool SetFpsDisable(bool if_disable);
        bool SetFps(float fps);

        bool SetTrigger(bool isTrigger);

        inline std::string get_guid() { return guid_; }

    private:
        void SetLut(float val, unsigned char mode = 0);

    private:
        void *         handle_ = NULL;
        int32_t        n_ret_  = -1;
        unsigned char *p_data_ = NULL;

        uint32_t             n_data_size_ = 0;
        MV_FRAME_OUT_INFO_EX st_img_info_ = {0};

        uint32_t    n_device_index_;
        std::string guid_;
        cv::Mat     lut_;
        bool        open_lut_ = false;
    };

} // namespace tdtbasecam

#endif