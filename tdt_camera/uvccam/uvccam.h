//
// Created by li on 2019/11/28.
//

#ifndef TDTVision_RM2021_UVCCAM_H
#define TDTVision_RM2021_UVCCAM_H

#include <iostream>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>

#include "tdtcommon.h"

namespace tdtbasecam {

class UVCBasicCam {
public:
    explicit UVCBasicCam() = default;
    ~UVCBasicCam() = default;

    bool InitCamera(__u8 dev_index, __u32 size_buffer = 4);
    bool InitCamera(std::string cam_guid, __u32 size_buffer = 4);

    bool StartStream();
    bool CloseStream();

    bool RestartCamera();

    bool GetMat(cv::Mat &img);  // 获取Opencv Mat图像

    bool SetExposureAuto(bool if_auto);
    bool SetExposure(__u32 t);  // 设置曝光
    bool SetGainAuto(bool if_auto);
    bool SetGain(__u32 val);  // 置增益
    bool SetBrightnessAuto(bool if_auto);
    bool SetBrightness(__u32 val);  // 置亮度

    bool SetWhitebalanceAuto(bool if_auto);
    bool SetWhitebalance(__u32 val);  // 设置白平衡(2800-6500)
    bool DoWhiteBalance();  // 单帧自动白平衡
    bool SetHue(__u32 val);  // 设置色调
    bool SetSaturation(__u32 val);  // 置饱和度

    bool SetContrast(__u32 val);  // 设置对比度
    bool SetGamma(float val);  // 设置伽玛
    bool SetSharpness(__u32 val);  // 设置锐度

    bool SetBacklightCompensation(__u32 val);  // 设置背光补偿
    bool SetPowerlineFrequency(__u32 val);  // 设置防闪烁频率(关:0 50Hz:1 60Hz:2)

    bool SetPixelformat(__u32 pixelformat = V4L2_PIX_FMT_MJPEG);
    bool SetResolution(__u32 width, __u32 height);
    bool SetFps(__u32 fps);

    std::string get_guid();// 获取相机在哪一个接口，用于分辨

    bool get_query_ctrl(__u32 id);  // 获取可调范围，参数如：V4L2_CID_GAIN

private:

    bool InitMMap();

    int xioctl(int fd, unsigned long request, void *arg);

    // 须安装第三方库libjpeg-turbo
    std::tuple<bool, std::vector<uint8_t>, uint64_t, uint64_t, uint64_t>
    DecodeJpeg2X(uint8_t *p_jpeg_data, uint64_t jpeg_data_size);

    cv::Mat Jpeg2Mat(uint8_t *jpeg_data, uint64_t jpeg_size);

private:
    typedef struct VideoBuffer {
        void *start;
        size_t length;
    } video_buffer_;
    VideoBuffer *buffers_;
    __u32 buffer_size_ = 4;
    unsigned int buffer_idx_ = 0;
    __u32 pixel_format_ = V4L2_PIX_FMT_MJPEG;
    const char *video_path_;
    int fd_;
    __u32 cam_bus_;
    __u32 width_;
    __u32 height_;
    __u32 fps_;
};

}

#endif //TDTVision_RM2021_UVCCAM_H
