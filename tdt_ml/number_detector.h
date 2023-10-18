/**
 * @Name: 数字识别头文件
 * @Description:
 * @Version: 1.0.0.1
 * @Author: 杨泽旭
 * @Date: 2019-11-23 21:26:32
 * @LastEditors: your name
 * @LastEditTime: 2019-11-16 16:00:04
 */

#ifndef TDTVision_RM2021_NUMBERDETECTOR_H
#define TDTVision_RM2021_NUMBERDETECTOR_H

#include "tdtcommon.h"

#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>

#include <mxnet-cpp/MxNetCpp.h>
#include <mxnet/c_predict_api.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace tdtml {

    /**
     * @brief 数字识别调用说明
     * @brief 所有数字识别都应该继承NumberDetector类,
     * @brief 重写virtual void Predict_ (std::vector<cv::Mat> &samples,std::vector<int> &flags) const接口
     * @brief 每次主程序初始化时,需要调用NumberDetector::Init(),传入一个可用的NumberDetector子类指针进行初始化
     * @brief 预测时调用NumberDetector::Predict进行预测
     */
    class NumberDetector {
    public:
        /**
         * @brief 构造函数
         */
        NumberDetector() = default;

        /**
         * @brief 预测功能实现的函数
         * @note 子类应该重写这个函数, 完成具体的数字识别活动
         * @param samples 传入的装甲板图形容器
         * @param flags 传出的预测结果, 应该与传入的图形一一对应
         */
        virtual void Predict_(std::vector<cv::Mat> &samples, std::vector<int> &flags) const { TDT_FATAL("NumberDetector未初始化, 至少在调用NumberDetector前调用Init()传入一个可用的数字识别类完成初始化!"); };

        /**
         * @brief 数字识别初始化
         * @note 应该在主程序初始化时被调用, 传入一个NumberDetector的有效子类的指针
         * @param number_detector 一个NumberDetector的子类指针
         */
        static inline void Init(const NumberDetector *number_detector) {
            delete number_detector_;
            number_detector_ = number_detector;
        }

        /**
         * @brief 预测, 需要进行数字识别的地方应当调用这个接口来进行识别, 会自动调用初始化时传入的数字识别类进行预测
         * @param samples 传入的装甲板图形容器
         * @param flags 传出的预测结果, 与传入的图形一一对应
         */
        static inline void Predict(std::vector<cv::Mat> &samples, std::vector<int> &flags) { number_detector_->Predict_(samples, flags); };

    private:
        static const NumberDetector *number_detector_;
    };

    // TODO: 搬运去年tnet,改名MxnetNumberDetector,要求模型在整个程序只载入一次
    // TODO: 并新增输入一组vector<Mat>也能输出结果
    /**
     * @name: NumberDetector
     * @description: 用于数字识别，实例化请用 static NumberDetector number_detector;
     * @Author:-
     */
    class MxnetNumberDetector : public NumberDetector {
    public:
        MxnetNumberDetector();
        ~MxnetNumberDetector() { MXPredFree(pred_hnd_); }

        /**
         * @name: tdtml::NumberDetector::Predict
         * @description: 输入一个矩阵数组，输出一个识别结果的数组
         * @param: vector<Mat> 装甲板ROI区域
         * @return:vector<int> 每个ROI区域对应的数字
         */
        void Predict_(std::vector<cv::Mat> &samples, std::vector<int> &flags) const final;

    private:
        int             width_      = 28;
        int             height_     = 28;
        int             channels_   = 1;
        int             image_size_ = width_ * height_ * channels_;
        PredictorHandle pred_hnd_   = nullptr;
    };

    /**
     * @name: 读取数字模型
     * @description: 用于读取数字模型
     * @Author:
     */
    class BufferFile {
    public:
        std::string file_path_;
        int         length_;
        char *      buffer_;
        explicit BufferFile(const std::string &file_path) : file_path_(file_path) {
            std::ifstream ifs(file_path.c_str(), std::ios::in | std::ios::binary);
            if (!ifs) {
                std::cerr << "Can't open the file. Please check " << file_path << ". \n";
                length_ = 0;
                buffer_ = nullptr;
                return;
            }

            ifs.seekg(0, std::ios::end);
            length_ = ifs.tellg();
            ifs.seekg(0, std::ios::beg);
            std::cout << file_path.c_str() << " ... " << length_ << " bytes\n";

            buffer_ = new char[sizeof(char) * length_];
            ifs.read(buffer_, length_);
            ifs.close();
        }

        int   GetLength() { return length_; }
        char *GetBuffer() { return buffer_; }

        ~BufferFile() {
            if (buffer_) {
                delete[] buffer_;
                buffer_ = nullptr;
            }
        }
    };

} // namespace tdtml

#endif // TDTVision_RM2021_NUMBERDETECTOR_H
