/**
 * @Name: 数字识别程序
 * @Description:
 * @Version: 1.0.0.1
 * @Author: 杨泽旭
 * @Date: 2019-11-24 14:32:32
 * @LastEditors: your name
 * @LastEditTime: 2019-11-16 16:00:04
 */
#ifdef O3ENABLE
#pragma GCC optimize(3, "Ofast", "inline")
#endif
#include "number_detector.h"

using namespace std;
using namespace mxnet::cpp;
using namespace cv;

const tdtml::NumberDetector *tdtml::NumberDetector::number_detector_ = new tdtml::NumberDetector();

tdtml::MxnetNumberDetector::MxnetNumberDetector() {
    static std::string json_file  = "../../model/TDT_Armor-4.20.json";
    static std::string param_file = "../../model/TDT_Armor-4.20.params";
    // Models path for your model,but you have no choice,we just get one avaliable modle

    static BufferFile json_data(json_file);
    static BufferFile param_data(param_file);
    // Parameters
    int          dev_type        = 1; // 1: cpu, 2: gpu//gpu就崩了
    int          dev_id          = 0; // arbitrary.
    mx_uint      num_input_nodes = 1; // 1 for feedforward
    const char * input_key[1]    = {"data"};
    const char **input_keys      = input_key;

    // Image size and channels

    const mx_uint input_shape_indptr[2] = {0, 4};
    const mx_uint input_shape_data[4]   = {4, static_cast<mx_uint>(channels_), static_cast<mx_uint>(height_), static_cast<mx_uint>(width_)};

    if (json_data.GetLength() == 0 || param_data.GetLength() == 0) {
        ;
    }
    bool created = MXPredCreate((const char *)json_data.GetBuffer(), (const char *)param_data.GetBuffer(), static_cast<size_t>(param_data.GetLength()), dev_type, dev_id, num_input_nodes, input_keys, input_shape_indptr, input_shape_data, &pred_hnd_);
    // Create Predictor
    assert(0 == created);
    assert(pred_hnd_);
}

void tdtml::MxnetNumberDetector::Predict_(vector<Mat> &samples, vector<int> &flags) const {
    flags.clear();
    vector<Mat> ml_rois;
    for (int i = 0; i < samples.size(); i++) {
        Mat roi_areas = samples[i];
//        namedWindow("asd",0);
//        imshow("asd",roi_areas);

//        static int a=0;
//        a++;
//        int seconds = std::time(NULL);
//        a+=seconds;
//        std::string dirname="/home/tdt/ArmorNum_distinguish_MX-Net/新搞得/5/";
//        std::string number=std::to_string(a);
//        std::string filename=dirname.append(number);
//        filename.append(".png");
//        cv::imwrite(filename,roi_areas);

        roi_areas.convertTo(roi_areas, CV_32F);
        normalize(roi_areas, roi_areas); // ROI区域，画面增强



        ml_rois.push_back(roi_areas);
    }
    static Mat         tmpzero = Mat::ones(Size(28, 28), CV_8U);
    std::vector<float> alldata;

    for (size_t i = 0; i < ml_rois.size(); i++) {
        if (i % 4 == 0) {
            vector<mx_float> image_data;
            for (size_t j = 0; j < 4; j++) {
                vector<mx_float> tmp;
                if (i + j >= ml_rois.size()) {
                    tmp = (vector<mx_float>)(tmpzero.reshape(1, 1));

                } else {
                    tmp = (vector<mx_float>)(ml_rois[i + j].reshape(1, 1));
                }
                image_data.insert(image_data.end(), tmp.begin(), tmp.end());
                tmp.clear();
            }

            // Set Input Image
            MXPredSetInput(pred_hnd_, "data", image_data.data(), image_size_ * 4);
            image_data.clear();
            // Do Predict Forward
            MXPredForward(pred_hnd_);
            mx_uint output_index = 0;

            mx_uint *shape = 0;
            size_t   size  = 1;
            mx_uint  shape_len;
            // Get Output Result
            MXPredGetOutputShape(pred_hnd_, output_index, &shape, &shape_len);
            for (mx_uint i = 0; i < shape_len; ++i)
                size *= shape[i];

            std::vector<float> data(size);
            MXPredGetOutput(pred_hnd_, 0, &(data[0]), size);
            alldata.insert(alldata.end(), data.begin(), data.end());
        }
    }
    for (size_t i = 0; i < ml_rois.size() * 9; i++) {
        if (i % 9 == 0) {
            int   result        = 0;
            float best_accuracy = 0.0;

            for (int j = 0; j < 9; j++) {

                if (alldata[i + j] > best_accuracy &&alldata[i+j]>0.8) {
                    best_accuracy = alldata[i + j];
                    result        = j;
                }
            }

            flags.push_back(result);
        }
    }
    alldata.clear();
}