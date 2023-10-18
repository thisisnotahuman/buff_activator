#include <cstdio>
#include <linux/videodev2.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <opencv2/opencv.hpp>
#include <turbojpeg.h>

#include "uvccam.h"

namespace tdtbasecam {

bool UVCBasicCam::InitCamera(__u8 dev_index, __u32 size_buffer) {
    char video_path[12];
    sprintf(video_path,"/dev/video%d",dev_index);
    //打开设备
    //fd_ = open(video_path,O_RDWR | O_NONBLOCK,0); // 用非阻塞模式打开摄像头设备
    fd_ = open(video_path, O_RDWR, 0); //用阻塞模式打开摄像头设备
    if (fd_ < 0) {
        TDT_ERROR("Fail to open a uvc camera with path [%s]!",video_path);
        return false;
    }
    struct v4l2_capability cap;

    if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
        TDT_ERROR("VIDIOC_QUERYCAP error! path [%s]",video_path);
        return false;
    }
    video_path_ = video_path;
    buffer_size_ = size_buffer;
    cam_bus_ = cap.bus_info[17];
    return true;
}

bool UVCBasicCam::InitCamera(std::string cam_guid, __u32 size_buffer) {
    //打开设备
    int index=-1;
    char video_path[12];
    int cam_bus = std::stoi(cam_guid);
    while(true){
        sprintf(video_path,"/dev/video%d",++index);
        if(access(video_path,F_OK)!=0) break;
        //fd_ = open(video_path,O_RDWR | O_NONBLOCK,0); // 用非阻塞模式打开摄像头设备
        fd_ = open(video_path_, O_RDWR, 0); //用阻塞模式打开摄像头设备
        if (fd_ < 0) {
            TDT_WARNING("Fail to open a uvc camera with path [%s]!",video_path);
            continue;
        }
        struct v4l2_capability cap;

        if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
            TDT_WARNING("VIDIOC_QUERYCAP error! path [%s]",video_path);
            continue;
        }
        if(cap.bus_info[17]==cam_bus){
            video_path_ = video_path;
            buffer_size_ = size_buffer;
            cam_bus_ = cap.bus_info[17];
            return true;
        }else{
            continue;
        }
    }
    TDT_ERROR("no camera found with guid [%s]!",cam_guid.c_str());
    return false;
}

bool UVCBasicCam::StartStream() {
    //开始视频流数据的采集
    if (!InitMMap()) return false;

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        TDT_ERROR("VIDIOC_STREAMON error! [%s]", video_path_);
        return false;
    }
    return true;
}

bool UVCBasicCam::CloseStream() {
    buffer_idx_ = 0;
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMOFF, &type) < 0) {
        TDT_ERROR("VIDIOC_STREAMOFF error! [%s]", video_path_);
        return false;
    }
    for (int i = 0; i < buffer_size_; i++) {
        munmap(buffers_[i].start, buffers_[i].length);
    }
    StartStream();
    return true;
}

bool UVCBasicCam::RestartCamera(){
    bool ret = true;
    TDT_INFO("===RESTART UVC CAMERA===");
    ret &= CloseStream();
    close(fd_);
    //int fd_ = open(video_path,O_RDWR | O_NONBLOCK,0);
    fd_ = open(video_path_,O_RDWR,0);
    if (fd_ < 0){
        TDT_ERROR("Fail to open a uvc camera with path [%s]!",video_path_);
        return false;
    }
    buffer_idx_ = 0;
    ret &= StartStream();
    if(ret){
        return true;
    }else{
        return false;
    }
}

bool UVCBasicCam::GetMat(cv::Mat &img) {
    //从视频采集输出队列中取出已含有采集数据的帧缓冲区
    struct v4l2_buffer bufferinfo = {0};
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = buffer_idx_;
    if(ioctl(fd_, VIDIOC_DQBUF, &bufferinfo) < 0){
        TDT_ERROR("VIDIOC_DQBUF error! [%s]", video_path_);
        return false;
    }

    if (pixel_format_ == V4L2_PIX_FMT_MJPEG){
        img = Jpeg2Mat((uint8_t*)buffers_[buffer_idx_].start,buffers_[buffer_idx_].length);
        /*
        Opencv自带解码与海康库不兼容
        cv::Mat src(Parameter.cam_parameter[parameter_index_].height-300,Parameter.cam_parameter[parameter_index_].width,CV_8UC1,buffers_[buffer_idx_].start);
        img = cv::imdecode(src, 1);
        */
    }
    else if(pixel_format_ == V4L2_PIX_FMT_YUYV){
        cv::Mat yuyv(height_,width_,CV_8UC2,buffers_[buffer_idx_].start);
        cv::cvtColor(yuyv, img, cv::COLOR_YUV2BGR_YUYV);
    }
    else{
        TDT_ERROR("UVC camera pixel format undefined!");
        return false;
    }
    /*
    if(open_lut_){
        LUT(img,lut_,img);
    }
    */

    //将该帧缓冲区重新排入输入队列
    bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufferinfo.memory = V4L2_MEMORY_MMAP;
    bufferinfo.index = buffer_idx_;

    // Queue the next one.
    if(ioctl(fd_, VIDIOC_QBUF, &bufferinfo) < 0){
        TDT_ERROR("VIDIOC_QBUF error! [%s]", video_path_);
        exit(1);
    }
    ++buffer_idx_;
    buffer_idx_ = buffer_idx_ >= buffer_size_ ? buffer_idx_ - buffer_size_ : buffer_idx_;
    return true;
}

bool UVCBasicCam::SetExposureAuto(bool if_auto) {
    struct v4l2_control ctrl;
    if(if_auto){
        ctrl.id = V4L2_CID_EXPOSURE_AUTO;
        ctrl.value = V4L2_EXPOSURE_AUTO;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            TDT_ERROR("V4L2_CID_EXPOSURE_AUTO fail! [%s]", video_path_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_EXPOSURE_AUTO;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            TDT_INFO("current value!: %d\n",get_ctrl.value);
            return false;
        }
    } else{
        ctrl.id = V4L2_CID_EXPOSURE_AUTO;
        ctrl.value = V4L2_EXPOSURE_MANUAL;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            TDT_ERROR("V4L2_CID_EXPOSURE_AUTO fail! [%s]", video_path_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_EXPOSURE_AUTO;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            TDT_INFO("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    return true;
}

bool UVCBasicCam::SetExposure(__u32 t){
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = t;
    if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
        TDT_ERROR("V4L2_CID_EXPOSURE_ABSOLUTE fail! [%s]", video_path_);
        struct v4l2_control get_ctrl;
        get_ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
        ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
        TDT_INFO("current value!: %d\n",get_ctrl.value);
        return false;
    }
    return true;
}
bool UVCBasicCam::SetGainAuto(bool if_auto) {
    struct v4l2_control ctrl;
    if(if_auto){
        ctrl.id = V4L2_CID_AUTOGAIN;
        ctrl.value = true;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0)
        {
            TDT_ERROR("V4L2_CID_AUTOGAIN fail! [%s]", video_path_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_AUTOGAIN;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            TDT_INFO("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }else {
        ctrl.id = V4L2_CID_AUTOGAIN;
        ctrl.value = false;
        if (xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
            TDT_ERROR("V4L2_CID_AUTOGAIN fail! [%s]", video_path_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_AUTOGAIN;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            TDT_INFO("current value!: %d\n", get_ctrl.value);
            return false;
        }
    }
    return true;
}
bool UVCBasicCam::SetGain(__u32 val) {
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_GAIN;
    ctrl.value = val;
    if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
        TDT_ERROR("V4L2_CID_GAIN fail! [%s]", video_path_);
        struct v4l2_control get_ctrl;
        get_ctrl.id = V4L2_CID_GAIN;
        ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
        TDT_INFO("current value!: %d\n",get_ctrl.value);
        return false;
    }
    return true;
}
bool UVCBasicCam::SetBrightnessAuto(bool if_auto) {
    struct v4l2_control ctrl;
    if (if_auto) {
        ctrl.id = V4L2_CID_AUTOBRIGHTNESS;
        ctrl.value = true;
        if (xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
            TDT_ERROR("V4L2_CID_AUTOBRIGHTNESS fail! [%s]", video_path_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_AUTOBRIGHTNESS;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            TDT_INFO("current value!: %d\n", get_ctrl.value);
            return false;
        } else {
            ctrl.id = V4L2_CID_AUTOBRIGHTNESS;
            ctrl.value = false;
            if (xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0) {
                TDT_ERROR("V4L2_CID_AUTOBRIGHTNESS fail! [%s]", video_path_);
                struct v4l2_control get_ctrl;
                get_ctrl.id = V4L2_CID_AUTOBRIGHTNESS;
                ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
                TDT_INFO("current value!: %d\n", get_ctrl.value);
                return false;
            }
        }
    }
    return true;
}
bool UVCBasicCam::SetBrightness(__u32 val){
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_BRIGHTNESS;
    ctrl.value =val;
    if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
        TDT_ERROR("V4L2_CID_BRIGHTNESS fail! [%s]", video_path_);
        struct v4l2_control get_ctrl;
        get_ctrl.id = V4L2_CID_BRIGHTNESS;
        ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
        TDT_INFO("current value!: %d\n",get_ctrl.value);
        return false;
    }
    return true;
}

bool UVCBasicCam::SetWhitebalanceAuto(bool if_auto) {
    struct v4l2_control ctrl;
    if(if_auto){
        ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
        ctrl.value = V4L2_WHITE_BALANCE_AUTO;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            TDT_ERROR("V4L2_CID_AUTO_WHITE_BALANCE fail! [%s]", video_path_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            TDT_INFO("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }else{
        ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
        ctrl.value = V4L2_WHITE_BALANCE_MANUAL;
        if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
            TDT_ERROR("V4L2_CID_AUTO_WHITE_BALANCE fail! [%s]", video_path_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_AUTO_WHITE_BALANCE;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            TDT_INFO("current value!: %d\n",get_ctrl.value);
            return false;
        }
    }
    return true;
}
bool UVCBasicCam::SetWhitebalance(__u32 val){
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
    ctrl.value = val;
    if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
        TDT_ERROR("V4L2_CID_WHITE_BALANCE_TEMPERATURE fail! [%s]", video_path_);
        struct v4l2_control get_ctrl;
        get_ctrl.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
        ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
        TDT_INFO("current value!: %d\n",get_ctrl.value);
        return false;
    }
    return true;
}
bool UVCBasicCam::DoWhiteBalance(){
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_DO_WHITE_BALANCE;
    if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
        TDT_ERROR("V4L2_CID_DO_WHITE_BALANCE fail! [%s]", video_path_);
        struct v4l2_control get_ctrl;
        get_ctrl.id = V4L2_CID_DO_WHITE_BALANCE;
        ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
        TDT_INFO("current value!: %d\n",get_ctrl.value);
        return false;
    }
    return true;
}
bool UVCBasicCam::SetHue(__u32 val){
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_HUE;
    ctrl.value = val;
    if(xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0){
        TDT_ERROR("V4L2_CID_HUE fail! [%s]", video_path_);
        struct v4l2_control get_ctrl;
        get_ctrl.id = V4L2_CID_HUE;
        ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
        TDT_INFO("current value!: %d\n",get_ctrl.value);
        return false;
    }
    return true;
}
bool UVCBasicCam::SetSaturation(__u32 val){
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_SATURATION;
    ctrl.value = val;
    if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
        TDT_ERROR("V4L2_CID_SATURATION [%s]", video_path_);
        struct v4l2_control get_ctrl;
        get_ctrl.id = V4L2_CID_SATURATION;
        ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
        TDT_INFO("current value!: %d\n",get_ctrl.value);
        return false;
    }
    return true;
}
bool UVCBasicCam::SetContrast(__u32 val){
struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_CONTRAST;
    ctrl.value = val;
    if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
        TDT_ERROR("V4L2_CID_CONTRAST [%s]", video_path_);
        struct v4l2_control get_ctrl;
        get_ctrl.id = V4L2_CID_CONTRAST;
        ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
        TDT_INFO("current value!: %d\n",get_ctrl.value);
        return false;
    }
    return true;
}
bool UVCBasicCam::SetGamma(float val){
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_GAMMA;
    ctrl.value = (__signed__ int)val;
    if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
        TDT_ERROR("V4L2_CID_GAMMA [%s]", video_path_);
        struct v4l2_control get_ctrl;
        get_ctrl.id = V4L2_CID_GAMMA;
        ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
        TDT_INFO("current value!: %d\n",get_ctrl.value);
        return false;
    }
    return true;
}
bool UVCBasicCam::SetSharpness(__u32 val){
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_SHARPNESS;
    ctrl.value = val;
    if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
        TDT_ERROR("V4L2_CID_SHARPNESS [%s]", video_path_);
        struct v4l2_control get_ctrl;
        get_ctrl.id = V4L2_CID_SHARPNESS;
        ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
        TDT_INFO("current value!: %d\n",get_ctrl.value);
        return false;
    }
    return true;
}
bool UVCBasicCam::SetBacklightCompensation(__u32 val){
    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_BACKLIGHT_COMPENSATION;
    ctrl.value = val;
    if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
        TDT_ERROR("V4L2_CID_BACKLIGHT_COMPENSATION [%s]", video_path_);
        struct v4l2_control get_ctrl;
        get_ctrl.id = V4L2_CID_BACKLIGHT_COMPENSATION;
        ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
        TDT_INFO("current value!: %d\n",get_ctrl.value);
        return false;
    }
    return true;
}
bool UVCBasicCam::SetPowerlineFrequency(__u32 val){
    struct v4l2_control ctrl;
        ctrl.id = V4L2_CID_POWER_LINE_FREQUENCY;
        ctrl.value = val;
        if(xioctl(fd_,VIDIOC_S_CTRL,&ctrl) < 0){
            TDT_ERROR("V4L2_CID_POWER_LINE_FREQUENCY [%s]", video_path_);
            struct v4l2_control get_ctrl;
            get_ctrl.id = V4L2_CID_POWER_LINE_FREQUENCY;
            ioctl(fd_, VIDIOC_G_CTRL, &get_ctrl);
            TDT_INFO("current value!: %d\n",get_ctrl.value);
            return false;
        }
    return true;
}

bool UVCBasicCam::SetPixelformat(__u32 pixelformat) {
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.pixelformat = pixelformat;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        TDT_ERROR("VIDIOC_S_FMT error! [%s]", video_path_);
        return false;
    }
    pixel_format_=fmt.fmt.pix.pixelformat;
    return true;
}

bool UVCBasicCam::SetResolution(__u32 width, __u32 height) {
    struct v4l2_format fmt = {0};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        TDT_ERROR("VIDIOC_S_FMT error! [%s]", video_path_);
        return false;
    }
    width_=fmt.fmt.pix.width;
    height_=fmt.fmt.pix.height;
    return true;
}

bool UVCBasicCam::SetFps(__u32 fps) {
    struct v4l2_streamparm stream_param = {0};
    stream_param.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    stream_param.parm.capture.timeperframe.denominator = fps;
    stream_param.parm.capture.timeperframe.numerator = 1;
    if (ioctl(fd_, VIDIOC_S_PARM, &stream_param) < 0) {
        TDT_ERROR("VIDIOC_S_PARM error! [%s]", video_path_);
        return false;
    }
    fps_ = stream_param.parm.capture.timeperframe.denominator
          / stream_param.parm.capture.timeperframe.numerator;
    return true;
}

bool UVCBasicCam::InitMMap(){
    //申请若干个帧缓冲区
    struct v4l2_requestbuffers bufrequest = {0};
    bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    bufrequest.memory = V4L2_MEMORY_MMAP;
    bufrequest.count = buffer_size_;

    if(ioctl(fd_, VIDIOC_REQBUFS, &bufrequest) < 0){
        TDT_ERROR("VIDIOC_REQBUFS error! [%s]", video_path_);
        return false;
    }

    buffers_ = (VideoBuffer*)calloc(bufrequest.count, sizeof(*buffers_));
    if (!buffers_){
        TDT_ERROR("out of memory! [%s]", video_path_);
        return false;
    }

    //查询帧缓冲区在内核空间中的长度和偏移量
    for(unsigned int i = 0; i < bufrequest.count; ++i){
        struct v4l2_buffer bufferinfo = {0};
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = i;

        if(ioctl(fd_, VIDIOC_QUERYBUF, &bufferinfo) < 0){
            TDT_ERROR("VIDIOC_QUERYBUF error! [%s]", video_path_);
            return false;
        }

        //通过内存映射将帧缓冲区的地址映射到用户空间
        buffers_[i].length = bufferinfo.length;
        buffers_[i].start = (char*)mmap(
                NULL,
                bufferinfo.length,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                fd_,
                bufferinfo.m.offset);

        if(buffers_[i].start == MAP_FAILED){
            TDT_ERROR("MMAP fail! [%s]", video_path_);
            return false;
        }
        memset(buffers_[i].start, 0, bufferinfo.length);

        //将申请到的帧缓冲全部放入视频采集输出队列
        memset(&bufferinfo, 0, sizeof(bufferinfo));
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = i;
        if(ioctl(fd_, VIDIOC_QBUF, &bufferinfo) < 0){
            TDT_ERROR("VIDIOC_QBUF error! [%s]", video_path_);
            return false;
        }
    }
    return true;
}

int UVCBasicCam::xioctl(int fd_, unsigned long request, void *arg){
    int r;
    do r = ioctl (fd_, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}

std::tuple<bool,std::vector<uint8_t>,uint64_t,uint64_t,uint64_t> UVCBasicCam::DecodeJpeg2X(uint8_t* p_jpeg_data,uint64_t jpeg_data_size) {
    assert( p_jpeg_data != NULL );
    int width = 0,height = 0,jpegsubsamp = 0;

    tjhandle jpeg = tjInitDecompress();

    if(jpeg == nullptr)
    {
        return std::make_tuple(false, std::vector<uint8_t>(0), 0, 0, 0);
    }

    if(tjDecompressHeader2(jpeg,p_jpeg_data,jpeg_data_size,&width,&height,&jpegsubsamp) != 0)
    {
        return std::make_tuple(false, std::vector<uint8_t>(0), 0, 0, 0);
    }

    TJPF eformat = TJPF::TJPF_BGR;

    uint64_t pitch = tjPixelSize[eformat] * width;
    uint64_t size = pitch * height;
    std::vector<uint8_t> output(size);

    if(tjDecompress2(jpeg,p_jpeg_data,jpeg_data_size,&output.front(),width,pitch,height,eformat,0) != 0)
    {
        return std::make_tuple(false, std::vector<uint8_t>(0), 0, 0, 0);
    }

    return std::make_tuple(true, std::move(output), size, width, height);
}

cv::Mat UVCBasicCam::Jpeg2Mat(uint8_t *jpeg_data, uint64_t jpeg_size) {
    auto res = DecodeJpeg2X( (uint8_t*)jpeg_data,jpeg_size);
    bool success = false;
    std::vector<uint8_t> buff;
    int width,height,size;

    std::tie(success,buff,size,width,height) = res;
    cv::Mat dst(height,width,CV_8UC3,(uint8_t*)&buff.front());
    return dst.clone();
}

std::string UVCBasicCam::get_guid() {
    std::string guid = std::to_string(cam_bus_);
    return guid;
}

bool UVCBasicCam::get_query_ctrl(unsigned int id){
    struct v4l2_queryctrl  Setting;
    Setting.id = id;
    if(xioctl(fd_, VIDIOC_QUERYCTRL, &Setting) < 0){
        TDT_ERROR("VIDIOC_QUERYCTRL error! [%s]", video_path_);
        return false;
    }
    TDT_INFO("max_value:%d\nmin_value:%d\ndefault_value:%d\nstep:%d\n",Setting.maximum,Setting.minimum,Setting.default_value,Setting.step);

    return true;
}

}