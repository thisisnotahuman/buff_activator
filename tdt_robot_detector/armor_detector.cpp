#ifdef O3ENABLE
#pragma GCC optimize(3, "Ofast", "inline")
#endif
#include "armor_detector.h"
#ifdef VIDEO_DEBUG
#include "Debug.h"
#endif
#include "log.h"
#include "number_detector.h"
#include "timecounter.h"
#include <mutex>
#include <thread>
//#include <opencv2/gapi/core.hpp>
//#include <opencv2/gapi/imgbproc.hpp>

using namespace tdttoolkit;

// #define DOUBLE_LIGHT_BAR_DEBUG
//#define SINGLE_LIGHT_BAR_DEBUG
//#define NO_LIGHT_BAR_DEBUG
// #define LIGHT_BAR_MATCH_DEBUG
// #define OUTPUT_LIGHT_BAR_INFORMATION
// #define OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION

namespace tdtrobot {
    ArmorDetector::ArmorDetector() {
        std::vector<ArmorDetectInfo>().swap(this->last_armors_info_);
        int src_width, src_height;
        LoadParam::ReadTheParam("SrcWidth", src_width);
        LoadParam::ReadTheParam("SrcHeight", src_height);
        this->armor_detect_roi_ = cv::Rect2i(cv::Point2i(0, 0), cv::Size2i(src_width, src_height));
        this->src_area_         = cv::Rect2i(cv::Point2i(0, 0), cv::Size2i(src_width, src_height));
        this->element_          = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 1));
        this->element2_         = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        LoadParam::ReadTheParam("EnemyColor", this->enemy_color_);
        LoadParam::ReadTheParam("ArmorDetect_lightbarthre", this->threshold_);
    }

    std::vector<tdttoolkit::RobotArmor> ArmorDetector::Get(cv::Mat &src, const tdttoolkit::ReceiveMessage &receive_message) {
        /*********************检测前数据更新************************/
        this->is_locking_status_ = receive_message.lock_command;
        // RoiFilter();

#ifdef VIDEO_DEBUG
        LoadParam::ReadTheParam("ArmorDetect_lightbarthre", this->threshold_);
#endif

        //        if(last_armors_info_.size()!=0){
        //            std::vector<tdttoolkit::RobotArmor> output_armors(0);
        //            track(src,output_armors);
        //            return output_armors;
        //        }

        /*************************声明变量*************************/
        std::vector<ArmorDetectInfo>        output_armors(0);
        std::vector<LightBarInfo>           light_bars(0);
        std::vector<ArmorDetectInfo>        armors(0);
        std::vector<tdttoolkit::RobotArmor> output_armors_data_package(0);
        //        std::vector<ArmorDetectInfo> matched_armor(0);

        /***************************检测**************************/
        //        static auto p1 = new TimeCounter("Find light bar");
        //        p1 -> AverageTimerStart();
        //        p1 -> AverageTimerEnd();
        FindLightBar(src, light_bars);

        /**********************仅用于开火决策***********************/
        ArmorDetector::trace_light_bars_;//获取处理后的灯条以便没有装甲板时跟踪目标
        trace_light_bars_.assign(light_bars.begin(), light_bars.end());
        //将灯条按面积从大到小排序
        sort(ArmorDetector::trace_light_bars_.begin(), ArmorDetector::trace_light_bars_.end(), [](const LightBarInfo &a, const LightBarInfo &b) -> bool { return a.GetArea() > b.GetArea(); });
        /**********************仅用于开火决策***********************/

        if (!light_bars.empty()) {
            //            static auto p2 = new TimeCounter("DoubleLightBarDetect");
            //            p2 -> AverageTimerStart();
            DoubleLightBarDetect(src, light_bars, armors);
            //            p2 -> AverageTimerEnd();

            //            static auto p3 = new TimeCounter("SingleLightBarDetect");
            //            p3 -> AverageTimerStart();
            //            p3 -> AverageTimerEnd();
            // if (armors.empty()) {
            // double time = cv::getTickCount(); 
            // SingleLightBarDetect(src, light_bars, armors);
            // std::cout<<"单灯条检测耗时"<<(cv::getTickCount()-time) * 1000 / cv::getTickFrequency()<<std::endl;
            // }
        }


        if (armors.empty()) {
            NumberStickerDetect(src, armors);
        }
#ifdef VIDEO_DEBUG
        for (auto &i : armors) {
            Debug::AddCustomRect("疑似装甲板", i.GetArmorRotRect(), cv::Scalar(255, 0, 0));
        }
        Debug::SetMessage("装甲板检测", "机器学习前装甲板数量", int(armors.size()));
#endif

        //无机器学习, 暂时不保险
        //		MatchWithLastArmor(armors, matched_armor);
        //		if(matched_armor.empty()) {
        //			TDT_DEBUG("机器学习");
        //			NumPredict(src, armors);
        //		} else {
        //			TDT_DEBUG("无机器学习");
        //			armors.swap(matched_armor);
        //		}
        //        static auto p3 = new TimeCounter("NumPredict");
        //        p3 -> AverageTimerStart();
        double t3 = cv::getTickCount();
        NumPredict(src, armors);
        // if(!armors.empty()) std::cout<<armors[0].GetLeftBarRect().GetAngle()<<"******"<<armors[0].GetRightBarRect().GetAngle()<<std::endl;

        //        p3 -> AverageTimerEnd();
        std::cout << "机器学习耗时:" << (cv::getTickCount() - t3) / cv::getTickFrequency() * 1000 / armors.size() << "ms" << std::endl;
        if (!armors.empty()) {
            if (armors.begin()->GetArmorType() != tdttoolkit::RobotType::TYPEUNKNOW) {
                last_robot_type_ = armors.begin()->GetArmorType();
            }
            EraseDuplicate(armors);
        }
#if defined(AUTO_DECIDE) && !defined(RADER_UPDATE)
        // Decide(armors, output_armors);
        output_armors = armors;
#else
        if (armors.empty()) {
            std::vector<ArmorDetectInfo>().swap(output_armors);
        } else {
            for (auto &it : armors) {
                if (it.GetArmorType() != tdttoolkit::RobotType::TYPEUNKNOW) {
                    output_armors.emplace_back(it);
                }
            }
        }
#endif // defined(AUTO_DECIDE) && !defined(RADER_UPDATE)
        for (const auto &armor_info : output_armors) {
            tdttoolkit::RobotArmor output_armor;
            ArmorTransform(armor_info, output_armor);
            output_armors_data_package.emplace_back(output_armor);
        }
        /************************ARMOR_DETECT_DEBUG***************************/
#ifdef ARMOR_DETECT_DEBUG
#ifdef VIDEO_DEBUG
        Debug::SetMessage("装甲板检测", "机器学习后装甲板数量", int(output_armors.size()));
        Debug::AddRect("输出装甲板", armor_detect_roi_, cv::Scalar(0, 0, 255), 3);
        for (auto const &armor : output_armors_data_package) {
            for (auto const &point : armor.GetImagePointsList()) {
                Debug::AddPoint("输出装甲板", point, 10, cv::Scalar(255, 0, 0));
            }
            Debug::AddCustomRect("输出装甲板", armor.GetStickerRect(), cv::Scalar(0, 255, 0), 3);
            Debug::AddText("输出装甲板", {char('0' + armor.GetRobotType())}, armor.GetStickerRect().GetBl(), cv::Scalar(255, 255, 0), 1, 2);
        }
#endif // VIDEO_DEBUG
#endif // ARMOR_DETECT_DEBUG
        std::cout << "找到" << output_armors_data_package.size() << "个装甲板\n";
        tdtlog::Log::ChannelLog("ArmorDetector", "找到%d个装甲板", output_armors_data_package.size());
        /************************检测后数据更新**********************/
        if (output_armors.empty()) {
            std::vector<ArmorDetectInfo>().swap(last_armors_info_);
        } else {
            last_armors_info_ = output_armors;
        }
        /***************************返回*************************/
        return output_armors_data_package;
    }

    void ArmorDetector::track(const cv::Mat &src, std::vector<tdttoolkit::RobotArmor> &output_armors) {
        cv::Rect                     track_rect;
        std::vector<LightBarInfo>    light_bars(0);
        std::vector<ArmorDetectInfo> armors(0);
        cv::Mat                      img;
        if (last_armors_info_.size() == 1) {
            track_rect = last_armors_info_[0].GetRect();
        } else {
            track_rect = last_armors_info_[0].GetRect() | track_rect;
            track_rect = last_armors_info_[1].GetRect() | track_rect;
        }
        track_rect = RectEnlarge(track_rect, cv::Size2f(4, 4));
        track_rect = track_rect & this->src_area_;
        img        = src(track_rect);
        // cv::imshow("1",img);cv::waitKey(100);
        FindLightBar_track(img, light_bars, track_rect.tl());
        if (light_bars.size() > 1) {

            std::cout << light_bars.size() << "lightsize*****************************\n";
            DoubleLightBarDetect(src, light_bars, armors);
            std::cout << armors.size() << "armors*****************************\n";

            if (armors.empty()) {
                std::vector<ArmorDetectInfo>().swap(last_armors_info_);
                return;
            } else {
                //大于2个，删除多余的装甲板
                if (armors.size() > 2) {
                    sort(armors.begin(), armors.end(), [this](const ArmorDetectInfo &a, const ArmorDetectInfo &b) -> bool { return tdttoolkit::CalcDistance(a.GetArmorRotRect().GetCenter(), last_armors_info_[0].GetArmorRotRect().GetCenter()) < tdttoolkit::CalcDistance(b.GetArmorRotRect().GetCenter(), last_armors_info_[0].GetArmorRotRect().GetCenter()); });
                    for (auto it = armors.begin() + 2; it != armors.end();) {
                        armors.erase(it);
                    }
                }
#ifdef VIDEO_DEBUG
                for (const auto &armor : armors) {
                    Debug::AddRect("输出装甲板", armor.GetRect(), cv::Scalar(0, 0, 255), 3);
                }
#endif
                for (auto &armor : armors) {
                    armor.SetArmorType(last_robot_type_);
                    tdttoolkit::RobotArmor output_armor;
                    ArmorTransform(armor, output_armor);
                    output_armors.emplace_back(output_armor);
                }
                //更新信息
                last_armors_info_ = armors;
                return;
            }
        } else {
            std::vector<ArmorDetectInfo>().swap(last_armors_info_);
        }
    }

    void ArmorDetector::RoiFilter() {
        if (last_armors_info_.empty()) { //上一次的armor非空
            this->armor_detect_roi_ = this->src_area_;
            return;
        }
        cv::Size2i enlarge_size;
        if (last_armors_info_[0].HaveRightBar() && last_armors_info_[0].HaveLeftBar()) {
            enlarge_size = cv::Size2i(9, 5);
        } else if (last_armors_info_[0].HaveRightBar() || last_armors_info_[0].HaveLeftBar()) {
            enlarge_size = cv::Size2i(14, 5);
        } else {
            enlarge_size = cv::Size2i(11, 7);
        }

        this->armor_detect_roi_ = RectEnlarge(last_armors_info_[0].GetRect(), enlarge_size);

        if (!RectSafety(armor_detect_roi_)) {
            this->armor_detect_roi_ = this->src_area_;
        }
        this->armor_detect_roi_ = RectEnlarge(last_armors_info_[0].GetRect(), cv::Size2i(11, 7));
    }

    void ArmorDetector::Decide(std::vector<ArmorDetectInfo> &input_armors, std::vector<ArmorDetectInfo> &output_armors) {
        tdttoolkit::RobotType armor_type = tdttoolkit::RobotType::TYPEUNKNOW;
        if (input_armors.empty()) {
            std::vector<ArmorDetectInfo>().swap(output_armors);
            return;
        }
        if (this->is_locking_status_ && !last_armors_info_.empty()) {
            armor_type = this->last_armors_info_[0].GetArmorType();
        } else {
            //排序函数, 装甲板排序, 算出装甲板中心到屏幕中心的距离, 到屏幕中心距离小的装甲板在前
            sort(input_armors.begin(), input_armors.end(), [this](const ArmorDetectInfo &a, const ArmorDetectInfo &b) -> bool { return tdttoolkit::CalcDistance(tdttoolkit::GetRectCenter(a.GetRect()), tdttoolkit::GetRectCenter(this->src_area_)) < tdttoolkit::CalcDistance(tdttoolkit::GetRectCenter(b.GetRect()), tdttoolkit::GetRectCenter(this->src_area_)); });

            for (auto i = input_armors.begin(); i != input_armors.end();) {
                /*if (i->GetArmorType() == tdttoolkit::RobotType::TYPEUNKNOW) {
                    input_armors.erase(i);
                    continue;
                }*/
                if (i->GetArmorType() == tdttoolkit::RobotType::ENGINEER && i != (input_armors.end()--)) {
                    ++i;
                    continue;
                }

                i          = (i == (input_armors.end()--) ? input_armors.begin() : i);
                armor_type = i->GetArmorType();
                break;
            }
        }
#ifdef VIDEO_DEBUG
        Debug::SetMessage("装甲板检测", "当前装甲板类型", int(armor_type));
#endif
        if (armor_type == tdttoolkit::RobotType::TYPEUNKNOW) {
            input_armors.swap(output_armors);
            return;
        }
        for (const ArmorDetectInfo &armor : input_armors) {
            if (armor.GetArmorType() == armor_type) {
                output_armors.emplace_back(armor);
            }
        }
    }

    void ArmorDetector::EraseDuplicate(std::vector<ArmorDetectInfo> &output_armors) {
        if (!output_armors.empty()) {
            for (unsigned int i = 0; i < output_armors.size() - 1; i++) {
                for (unsigned int k = i + 1; k < output_armors.size(); k++) {
                    if (!(output_armors[i].GetRect() & output_armors[k].GetRect()).empty()) {
                        if (output_armors[i].GetArmorType() == output_armors[k].GetArmorType() || (output_armors[i].GetArmorType() != last_robot_type_ && output_armors[k].GetArmorType() != last_robot_type_)) {
                            output_armors.erase(output_armors.begin() + ((output_armors[i].GetArmorRotRect().GetRotRect().size.area() < output_armors[k].GetArmorRotRect().GetRotRect().size.area()) ? k : i));
                        } else if (output_armors[i].GetArmorType() == this->last_robot_type_) {
                            output_armors.erase(output_armors.begin() + k);
                        } else {
                            output_armors.erase(output_armors.begin() + i);
                        }
                    }
                }
            }
        }
    }

    int  time, times;
    void ArmorDetector::NumPredict(const cv::Mat &src, std::vector<ArmorDetectInfo> &armors) {
        std::vector<cv::Mat> ml_rois;
        std::vector<int>     results;
        for (ArmorDetectInfo &armor : armors) {
            cv::Mat armor_img;
            GetMlRoi(armor, src, armor_img);
            // cv::threshold(armor_img,armor_img,0,255,cv::THRESH_OTSU);
            // imshow("1",armor_img);
            ml_rois.push_back(armor_img);
        }
        tdtml::NumberDetector::Predict(ml_rois, results);

        for (uint64_t i = 0; i < armors.size(); i++) {
            switch (results[i]) {
            case 7:
                armors[i].SetArmorType(tdttoolkit::RobotType::SENTRY);
                break;
            case 8:
                armors[i].SetArmorType(tdttoolkit::RobotType::BASE);
                break;
            default:
                armors[i].SetArmorType(tdttoolkit::RobotType(results[i]));
            }
        }

#ifdef LIGHT_BAR_MATCH_DEBUG
        std::cout << "数字识别结果:";
        for (auto &temp : results) {
            std::cout << temp << ",";
        }
        std::cout << "\n";
#endif
        for (auto it = armors.begin(); it != armors.end();) //删除识别为0的装甲板
        {

            if (it->GetArmorType() == tdttoolkit::RobotType::TYPEUNKNOW) {
                armors.erase(it);
                time++;
                continue;
            }
            times++;
            it++;
        }
        std::cout << "识别数量:" << time << "-" << times << "*********\n";
    }

    void ArmorDetector::FindLightBar(const cv::Mat &src, std::vector<LightBarInfo> &output_light_bars) {
        double t1 = cv::getTickCount();
        ////////////////////////////// 图像灯条查找预处理 //////////////////////////////
        //        static cv::Mat binary_graph; //二值化图
        //        static cv::GMat gin,gout;
        //        static cv::Mat roi;
        //        roi = src(this->armor_detect_roi_);//TODO:0.020134

        //        static auto p41 = new TimeCounter("second part");
        //        p41 -> AverageTimerStart();
        //        auto p241 = new TimeCounter("second part");
        //        std::vector<cv::Mat> rgb;
        //        split(roi, rgb);//todo:0.665424ms
        //        cv::Mat img_gray = (rgb[this->enemy_color_] - rgb[2 - (this->enemy_color_)]) | rgb[1];  //通道相减取地方颜色处, 绿色通道或上去补足空洞//TODO:0.531427ms
        //        cv::Mat img_gray = rgb[this->enemy_color_];
        static cv::Mat img_gray;
        // cv::extractChannel(src, img_gray, this->enemy_color_);
        std::vector<cv::Mat> rgb;
        split(src, rgb);
        img_gray = rgb[this->enemy_color_] - rgb[2 - this->enemy_color_];
        //        static cv::GMat g_img_gray=std::get<2>(cv::gapi::split3(gin));//0.024244ms
        //        delete p241;
        //        p41 -> AverageTimerEnd();

        //        static auto p42 = new TimeCounter("third part");
        //        p42 -> AverageTimerStart();

        //         double time3= static_cast<double>(cv::getTickCount());
        //         cv::GMat g_binary_graph = cv::gapi::threshold(g_img_gray,threshold_,255,cv::THRESH_BINARY);//0.017076ms
        cv::threshold(img_gray, img_gray, this->threshold_, 255, cv::THRESH_BINARY); //将灰度图二值化 寻找灯条解//TODO:1.299093ms
                                                                                     //        std::cout<<"[DEBUG] third part耗时:"<<(static_cast<double>(cv::getTickCount())-time3)/cv::getTickFrequency()*1000<<"ms"<<std::endl;
                                                                                     //        p42 -> AverageTimerEnd();
        // cv::imshow("cd", img_gray);
        //        Debug::img=img_gray.clone();
        //        Debug::ShowMat();

        //        g_binary_graph = cv::gapi::dilate(g_binary_graph,this->element_);//0.011733ms

        //        cv::morphologyEx(binary_graph,binary_graph,cv::MORPH_CLOSE,element);
        //        erode(binary_graph, binary_graph, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1);

        //        static auto p44 = new TimeCounter("forth part");
        //        p44 -> AverageTimerStart();
        //        double time4= static_cast<double>(cv::getTickCount());
        //        static cv::GComputation ac(gin,g_binary_graph);
        //        ac.apply(src, img_gray);
        //        std::cout<<"[DEBUG] forth part耗时:"<<(static_cast<double>(cv::getTickCount())-time4)/cv::getTickFrequency()*1000<<"ms"<<std::endl;
        //        p44 -> AverageTimerEnd();

        //        static auto p45 = new TimeCounter("fifth part");
        //        p45 -> AverageTimerStart();
        //        auto p245 = new TimeCounter("fifth part");
        std::vector<std::vector<cv::Point2i>> contours; //轮廓容器

        findContours(img_gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, this->armor_detect_roi_.tl()); //寻找轮廓
                                                                                                                    //        delete p245;
                                                                                                                    //        p45 -> AverageTimerEnd();
        double t2 = cv::getTickCount();
        std::cout << "[Get][LightBar]findCounters耗时" << (t2 - t1) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
        ////////////////////////////// 根据轮廓查找灯条 //////////////////////////////
        //        static auto p43 = new TimeCounter("sixth part");
        //        p43 -> AverageTimerStart();
        //
        //        for (const auto &contour : contours) { //遍历所有轮廓

        for (int i = 0; i < contours.size(); i++) {
            tdttoolkit::CustomRect rot_rect(contours[i]);
            static cv::Point2i     div_point(10, 15);
            static cv::Rect        div_rect = cv::Rect();
            div_rect                        = cv::Rect(rot_rect.GetCenter() - div_point, rot_rect.GetCenter() + div_point);
            RectSafety(div_rect);
#ifdef VIDEO_DEBUG
            Debug::AddText("检测出的轮廓", std::to_string(i), contours[i][0], cv::Scalar(255, 125, 0), 1, 2);
//            Debug::AddRect("检测出的轮廓",div_rect,cv::Scalar(255,125,0));
#endif // VIDEO_DEBUG
            if (rot_rect.Vertical() < rot_rect.Cross() * 1.6) {
#ifdef OUTPUT_LIGHT_BAR_INFORMATION
                std::cout << i << "灯条检测-Warning: 竖向与横向比例不对:" << rot_rect.Vertical() / rot_rect.Cross() << std::endl;
#endif // OUTPUT_LIGHT_BAR_INFORMATION
                continue;
            }

            if (rot_rect.GetSize().area() < 20 || rot_rect.GetSize().area() > 19000) { //面积太小或者太大, 扔掉
#ifdef OUTPUT_LIGHT_BAR_INFORMATION
                std::cout << i << "灯条检测-Warning: 面积太小或者太大:" << rot_rect.GetSize().area() << std::endl;
#endif // OUTPUT_LIGHT_BAR_INFORMATION
                continue;
            }
            if (rot_rect.GetAngle() > 155 || rot_rect.GetAngle() < 25) {
#ifdef OUTPUT_LIGHT_BAR_INFORMATION
                std::cout << i << "灯条检测-Warning: 角度有问题:" << rot_rect.GetAngle() << std::endl;
#endif // OUTPUT_LIGHT_BAR_INFORMATION
                continue;
            }
//             if (rot_rect.GetWidth() > rot_rect.GetHeight() * 25) {
// #ifdef OUTPUT_LIGHT_BAR_INFORMATION
//                 std::cout << i << "灯条检测-Warning: 长短比例有问题:" << rot_rect.GetWidth() << "-" << rot_rect.GetHeight() << std::endl;
// #endif // OUTPUT_LIGHT_BAR_INFORMATION
//                 continue;
//             }
            cv::Scalar area_sum = sum(src(div_rect));
            if (area_sum[2 - enemy_color_] > area_sum[enemy_color_]) {
#ifdef OUTPUT_LIGHT_BAR_INFORMATION
                std::cout << i << "灯条检测-Warning: 队友" << std::endl;
#endif // OUTPUT_LIGHT_BAR_INFORMATION
                continue;
            }
#ifdef VIDEO_DEBUG
            Debug::AddText("检测出的轮廓", std::to_string(i), contours[i][0], cv::Scalar(0, 125, 255), 1, 2);
//            Debug::AddRect("检测出的轮廓",div_rect,cv::Scalar(0,125,255));
#ifdef OUTPUT_LIGHT_BAR_INFORMATION
            std::cout << i << "灯条检测-Information: 正常" << std::endl;
#endif // OUTPUT_LIGHT_BAR_INFORMATION
#endif // VIDEO_DEBUG

            LightBarInfo light_bar(rot_rect);
            output_light_bars.push_back(light_bar);
        }
        double t3 = cv::getTickCount();
        std::cout << "[Get][LightBar]for耗时" << (t3 - t2) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
        //        p43 -> AverageTimerEnd();

        //        static auto p46 = new TimeCounter("final part");
        //        p46 -> AverageTimerStart();
        sort(output_light_bars.begin(), output_light_bars.end(), [](const LightBarInfo &a, const LightBarInfo &b) -> bool { return a.GetCenter().x < b.GetCenter().x; });
        //        p46 -> AverageTimerEnd();

#ifdef VIDEO_DEBUG
        for (int i = 0; i < output_light_bars.size(); i++) {
            Debug::AddCustomRect("寻找到的灯条", output_light_bars[i], cv::Scalar(255, 192, 203), 2);
#ifdef LIGHT_BAR_MATCH_DEBUG
            Debug::AddText("寻找到的灯条", {char('0' + i)}, output_light_bars[i].GetTl(), cv::Scalar(255, 255, 0), 1, 2);
#endif // LIGHT_BAR_MATCH_DEBUG
        }
#endif // VIDEO_DEBUG
        double t4 = cv::getTickCount();
        std::cout << "[Get][LightBar]rest耗时" << (t4 - t3) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
        std::cout << "[Get]FindLightBar总耗时" << (t4 - t1) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
    }
    void ArmorDetector::FindLightBar_track(const cv::Mat &src, std::vector<LightBarInfo> &output_light_bars, const cv::Point2i &point) {
        ////////////////////////////// 图像灯条查找预处理 //////////////////////////////
        static cv::Mat img_gray;
        cv::extractChannel(src, img_gray, this->enemy_color_);
        cv::threshold(img_gray, img_gray, this->threshold_, 255, cv::THRESH_BINARY); //将灰度图二值化 寻找灯条解
        cv::dilate(img_gray, img_gray, this->element_);
        std::vector<std::vector<cv::Point2i>> contours;                                      //轮廓容器
        findContours(img_gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, point); //寻找轮廓
        ////////////////////////////// 根据轮廓查找灯条 //////////////////////////////

        for (int i = 0; i < contours.size(); i++) {
            tdttoolkit::CustomRect rot_rect(contours[i]);
            static cv::Point2i     div_point(10, 10);
            static cv::Rect        div_rect = cv::Rect();
            div_rect                        = cv::Rect(rot_rect.GetCenter() - div_point, rot_rect.GetCenter() + div_point);
            RectSafety(div_rect);
            if (rot_rect.Vertical() < rot_rect.Cross() * 1.6) {
                std::cout << i << "灯条检测-Warning: 竖向与横向比例不对:" << rot_rect.Vertical() / rot_rect.Cross() << std::endl;
                continue;
            }
            if (rot_rect.GetSize().area() < 20 || rot_rect.GetSize().area() > 19000) { //面积太小或者太大, 扔掉
                std::cout << i << "灯条检测-Warning: 面积太小或者太大:" << rot_rect.GetSize().area() << std::endl;
                continue;
            }
            if (rot_rect.GetAngle() > 125 || rot_rect.GetAngle() < 55) {
                std::cout << i << "灯条检测-Warning: 角度有问题:" << rot_rect.GetAngle() << std::endl;
                continue;
            }
            if (rot_rect.GetWidth() * 0.4 < rot_rect.GetHeight() || rot_rect.GetWidth() > rot_rect.GetHeight() * 20) {
                std::cout << i << "灯条检测-Warning: 长短比例有问题:" << rot_rect.GetWidth() << "-" << rot_rect.GetHeight() << std::endl;
                continue;
            }
            /*cv::Scalar area_sum = sum(src(div_rect));
            if (area_sum[2 - enemy_color_] > area_sum[enemy_color_]) {
                std::cout << i << "灯条检测-Warning: 队友" << std::endl;
                continue;
            }*/
            std::cout << i << "灯条检测-Information: 正常" << std::endl;
            LightBarInfo light_bar(rot_rect);
            output_light_bars.push_back(light_bar);
        }
        sort(output_light_bars.begin(), output_light_bars.end(), [](const LightBarInfo &a, const LightBarInfo &b) -> bool { return a.GetCenter().x < b.GetCenter().x; });
    }

    void ArmorDetector::SingleLightBarDetect(const cv::Mat &src, std::vector<LightBarInfo> &light_bars, std::vector<ArmorDetectInfo> &output_armors) {
        std::vector<std::vector<ArmorDetectInfo>> armors_detected_vector(light_bars.size());
        std::vector<ArmorDetectInfo>              tmp_output_armors;

            ////-------单灯条封装Armor函数-----------
        for (uint64_t i = 0; i < light_bars.size(); ++i) {
            if (light_bars[i].GetFindStatus() == -1 || light_bars[i].GetFindStatus() == 1 || light_bars[i].GetFindStatus() == 2) { //左边或者右边已经查过, 跳过
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    std::cout << i << " Warning:已匹配，跳过：" << light_bars[i].GetFindStatus() << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    continue;
            }
            LightBarInfo &light_bar = light_bars[i];
            std::vector<ArmorDetectInfo> &armors_detected = armors_detected_vector[i];
            for (int lr = -1; lr <= 1; lr += 2) { //搜索左侧右侧, 左-1, 右1

                ////----------------------通过单灯条确定查找区域-----------------------
                cv::Point2i rcentor = cv::Point2i(light_bar.GetCenter().x + 0.7f * light_bar.GetHeight() + 1.f * light_bar.GetWidth() - fabs(light_bar.GetWidth() * cos(light_bar.GetAngle() / 180.f * M_PI)), light_bar.GetCenter().y + (light_bar.GetHeight() + light_bar.GetWidth()) * cos(light_bar.GetAngle() / 180.f * M_PI));
                cv::Size2i  rsize   = cv::Size2i(2.3f * light_bar.GetWidth() - light_bar.GetWidth() * fabs(cos(light_bar.GetAngle() / 180.f * M_PI)) * (1 + 0.00001f * light_bar.GetRotRect().size.area()), 1.3f * light_bar.GetWidth());
                if (last_robot_type_ == RobotType::BASE || last_robot_type_ == RobotType::SENTRY || last_robot_type_ == RobotType::HERO) {
                    rcentor = cv::Point2i(light_bar.GetCenter().x + (0.7f * light_bar.GetHeight() + 1.f * light_bar.GetWidth()) * 1.7 - fabs(light_bar.GetWidth() * cos(light_bar.GetAngle() / 180.f * M_PI)), light_bar.GetCenter().y);
                    rsize.height *= 2.0f;
                }
                if (lr == -1) {
                    rcentor = 2 * light_bar.GetCenter() - rcentor;
                }
                cv::Rect2i search_rect = tdttoolkit::CustomRect(rcentor, rsize, light_bar.GetAngle()).GetRect();
#ifdef VIDEO_DEBUG
                Debug::AddRect("单灯条检测潜在装甲板区域", search_rect, cv::Scalar(255, 192, 203), 2);
#endif // VIDEO_DEBUG
       ////----------------------------------------------------------------

                //				if (search_rect.area() < 500) { //太小跳过
                //#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                //                    std::cout<<lr<<"：单灯条匹配-Warning:搜索区域面积太小："<<search_rect.area()<<std::endl;
                //#endif// OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                //					continue;
                //				}
                if (!RectSafety(search_rect)) { //在画面外,跳过
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    std::cout << lr << "：单灯条匹配-Warning:搜索区域在画面外" << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    continue;
                }

                cv::Mat              image_numb = src(search_rect); //更新last_image
                cv::Mat image_without_color;
                cv::extractChannel(image_numb, image_without_color, 2-enemy_color_);


                int threshold = RegionOustThreshold(image_without_color, image_without_color, lr);
                // cv::imshow(std::to_string(i)+"---"+std::to_string(lr), image_without_color);
                if ((sum(image_without_color)[0] * 7 / 255) > (image_without_color.size().area() * 7)) {
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    std::cout << lr << "：单灯条匹配-Warning:像素和太大：" << (sum(image_without_color)[0] * 7 / 255) << "-" << (image_without_color.size().area() * 7) << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    continue;
                }
                //				if((sum(image_without_color)[0] * 7 / 255) < (image_without_color.size().area())) {
                //#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                //                    std::cout<<lr<<"：单灯条匹配-Warning:像素和太小："<<(sum(image_without_color)[0] * 7 / 255)<<"-"<<image_without_color.size().area()<<std::endl;
                //#endif// OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                //					continue;
                //				}

                erode(image_without_color, image_without_color, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 1);
                dilate(image_without_color, image_without_color, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)), cv::Point(-1, -1), 2);

                std::vector<std::vector<cv::Point>> contours;
                findContours(image_without_color, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, search_rect.tl()); //查找轮廓
                if (contours.size() > 25 || contours.empty()) {
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    std::cout << lr << "：单灯条匹配-Warning:搜索区域为空或过于杂乱：" << contours.size() << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    continue;
                }

                std::vector<tdttoolkit::CustomRect> Numb_rects;

#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                int k = 0;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                for (std::vector<cv::Point> const &contour : contours) {
                    if (contour.size() < 18) { //包含的点太少, 跳过
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                        std::cout << k++ << "-" << lr << "：单灯条数字匹配-Warning:包含的点太少：" << contour.size() << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                        continue;
                    }
                    float                  angle     = light_bar.GetAngle();
                    tdttoolkit::CustomRect Numb_rect = tdttoolkit::CustomRect::minCustomRect(contour, angle);
                    cv::Rect               b_rect    = Numb_rect.GetRect();
#ifdef VIDEO_DEBUG
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    Debug::AddText("单灯条检测潜在装甲板区域", std::to_string(k), Numb_rect.GetCenter(), cv::Scalar(125, 255, 0), 1, 1);
#endif                                                  // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
#endif                                                  // VIDEO_DEBUG
                    if (b_rect.width > b_rect.height) { //长宽不合要求, 跳过
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                        std::cout << k++ << "-" << lr << "：单灯条匹配问题-Warning:长宽比例问题：" << b_rect.width << "-" << b_rect.height << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                        continue;
                    }
                    if (Numb_rect.GetSize().area() < 0.05 * search_rect.size().area()||Numb_rect.GetSize().area() < 350) { //面积太小, 跳过
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                        std::cout << k++ << "-" << lr << "：单灯条匹配问题-Warning:面积太小:" << Numb_rect.GetSize().area() << "-" << search_rect.size().area() << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                        continue;
                    }
                    if (Numb_rect.GetSize().area() < light_bar.GetSize().area() * 1.5) {
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                        std::cout << k++ << "-" << lr << "：单灯条匹配问题-Warning:面积与灯条相比太小:" << Numb_rect.GetSize().area() << "-" << light_bar.GetSize().area() << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                        continue;
                    }
                    if (Numb_rect.GetSize().width < light_bar.GetWidth()) {
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                        std::cout << k++ << "-" << lr << "：单灯条匹配问题-Warning:相比灯条太短:" << Numb_rect.GetSize().width << "-" << light_bar.GetWidth() << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                        continue;
                    }
                    if (Numb_rect.GetSize().area() > search_rect.size().area()) { //面积太大
                        Numb_rect.SetSize(cv::Size(search_rect.height, search_rect.width));
                    }
                    cv::Point2f Numb_to_Lightbar = Numb_rect.GetCenter()-light_bar.GetCenter();
                    float angle1 = atan2(Numb_to_Lightbar.y, Numb_to_Lightbar.x);
                    cv::Point2f Lightbar = light_bar.GetTl() - light_bar.GetBl();
                    angle1 = atan2(Lightbar.y, Lightbar.x) - angle1;
                    angle1 = angle1 / M_PI * 180;
                    if (fabs(angle1) < 70 || fabs(angle1) > 125) {
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                        std::cout << k++ << "-" << lr << "：单灯条匹配问题-Warning:角度不对:" << angle1 << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                        continue;
                    }
                    Numb_rects.emplace_back(Numb_rect);
#ifdef VIDEO_DEBUG
                    Debug::AddRotatedRect("单灯条检测潜在装甲板区域", Numb_rect.GetRotRect(), cv::Scalar(0, 0, 255), 2);
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    Debug::AddText("单灯条检测潜在装甲板区域", std::to_string(k), Numb_rect.GetCenter(), cv::Scalar(0, 0, 255), 1, 1);
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
#endif // VIDEO_DEBUG
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    std::cout << k++ << "-" << lr << "：没问题" << Numb_rect.GetSize().area() << "-" << light_bar.GetSize().area() << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                }
                if (Numb_rects.empty()) {
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    std::cout << "--------无匹配数字区域--------" << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
                    continue;
                }

                sort(Numb_rects.begin(), Numb_rects.end(), [](const tdttoolkit::CustomRect &a, const tdttoolkit::CustomRect &b) -> bool { return a.GetSize().area() > b.GetSize().area(); }); //以旋转矩形面积从大到小排序
                //选择靠近灯条位置的轮廓
                if (Numb_rects.size() > 1) {
                    Numb_rects[0] = (Numb_rects[0].GetCenter().x < Numb_rects[1].GetCenter().x) == bool(lr == 1) ? Numb_rects[0] : Numb_rects[1];
                }
#ifdef VIDEO_DEBUG
                Debug::AddRotatedRect("数字区域范围", Numb_rects[0].GetRotRect(), cv::Scalar(255, 192, 203), 2);
#endif
                ArmorDetectInfo tmp_armor = ArmorDetectInfo(light_bar, Numb_rects[0], lr, threshold);
                CalaArmorRotRect(tmp_armor);

                //				light_bar.SetFindStatus(light_bar.GetFindStatus() == -lr ? 2 : lr);//TODO

                auto func = [this, &tmp_output_armors](const ArmorDetectInfo &input_armor) -> bool {
                    for (auto &armor : tmp_output_armors) {
                        if (IsArmorNearby(armor, input_armor)) {
                            return true;
                        }
                    }
                    return false;
                };

                if (func(tmp_armor)) {
                    continue;
                }

                tmp_output_armors.push_back(tmp_armor);
                armors_detected.emplace_back(tmp_armor);
            }
        };
        for (uint64_t i = 0; i < light_bars.size(); ++i) {
#ifdef OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
            std::cout << "---------------" << i << "---------------" << std::endl;
#endif // OUTPUT_SINGLE_LIGHT_BAR_DETECT_INFORMATION
#ifdef VIDEO_DEBUG
            Debug::AddText("单灯条检测潜在装甲板区域", std::to_string(i), light_bars[i].GetRect().tl() + cv::Point(0, 100), cv::Scalar(255, 200, 0), 1, 2);
#endif // VIDEO_DEBUG
#ifdef SINGLE_LIGHT_BAR_DEBUG
            TDT_DEBUG("{单灯条}  线程%d结束", i);
            TDT_DEBUG("{单灯条}  第%d个灯条有%d个成员", i, armors_detected_vector[i].size());
#endif // SINGLE_LIGHT_BAR_DEBUG
            if (!armors_detected_vector[i].empty()) {
                output_armors.insert(output_armors.end(), armors_detected_vector[i].begin(), armors_detected_vector[i].end());
            }
            std::vector<ArmorDetectInfo>().swap(armors_detected_vector[i]);
        }
        std::vector<std::vector<ArmorDetectInfo>>().swap(armors_detected_vector);
    }

    //循环嵌套双灯条匹配
    void ArmorDetector::DoubleLightBarDetect(const cv::Mat &src, std::vector<LightBarInfo> &light_bars, std::vector<ArmorDetectInfo> &output_armors) {
        double                                    t0 = cv::getTickCount();
        std::vector<std::vector<ArmorDetectInfo>> armors_detected_vector(light_bars.size() - 1);
        for (uint64_t i = 0; i < (light_bars.size() - 1); ++i) {
            for (uint64_t j = i + 1; j < light_bars.size(); j++) {
#ifdef LIGHT_BAR_MATCH_DEBUG
                std::cout << i << "-" << j << ":灯条匹配：";
#endif // LIGHT_BAR_MATCH_DEBUG

                if (!IsLightBarMatched(light_bars[i], light_bars[j])) //不是匹配的灯条, 跳
                {
                    continue;
                }
                ////----------算出查找数字贴纸的区域---------------
                LightBarInfo light_bar = light_bars[i].GetWidth() > light_bars[j].GetWidth() ? light_bars[i] : light_bars[j];
                // CustomRect   search_number_rect = CustomRect((light_bars[i].GetCenter() + light_bars[j].GetCenter()) / 2, cv::Size((light_bars[i].GetWidth() + light_bars[j].GetWidth()), 0.7 * (light_bars[j].GetCenter().x - light_bars[i].GetCenter().x)), (light_bars[i].GetAngle() + light_bars[j].GetAngle()) / 2);
                float width = 0.7 * tdttoolkit::CalcDistance(light_bars[j].GetCenter(), light_bars[i].GetCenter());
                float height = 1.2 * (light_bars[i].GetWidth() + light_bars[j].GetWidth());
                float angle = (light_bars[i].GetAngle() + light_bars[j].GetAngle()) / 2;
                cv::Point center = (light_bars[i].GetCenter() + light_bars[j].GetCenter()) / 2;

                CustomRect search_number_rect;
                if(height > width){
                    search_number_rect = CustomRect(center, cv::Size(height, width), angle);
                } else if (angle > 90 && width > height){
                    search_number_rect = CustomRect(center, cv::Size(width, height), angle - 90);
                } else if (angle < 90 && width > height){
                    search_number_rect = CustomRect(center, cv::Size(width, height), angle + 90);
                } else if (angle == 90 && width > height){
                    search_number_rect = CustomRect(center, cv::Size(width, height), 0);
                } else {
                    height += 0.001;
                    search_number_rect = CustomRect(center, cv::Size(height, width), angle);
                }
                cv::Rect search_rect = search_number_rect.GetRect();
                if (!RectSafety(search_rect)) //若矩形在图像外, 跳过, 否则取
                {
                    continue; //安全性
                }
#ifdef VIDEO_DEBUG //灯条匹配的结果
                Debug::AddCustomRect("双灯条检测潜在装甲板区域", search_number_rect, cv::Scalar(255, 192, 203), 2);
                Debug::AddRect("双灯条检测潜在装甲板区域", search_rect, cv::Scalar(255, 192, 203), 2);
                Debug::AddText("双灯条检测潜在装甲板区域", std::to_string(j), search_rect.tl(), cv::Scalar(255, 192, 203), 2);
#endif // VIDEO_DEBUG
       ////-------------------------------------------

                cv::Mat image_numb = src(search_rect); //取感兴趣区域
                cv::Mat image_without_color;
                cv::extractChannel(image_numb, image_without_color, 2-enemy_color_);
                // for(int m=0; m<=light_bars[i].GetHeight(); m++){
                //     for(int n=0; n<=light_bars[i].GetWidth(); n++){
                //         float angle1 = atan((light_bars[i].GetTr()-light_bars[i].GetTl()).y/(light_bars[i].GetTr()-light_bars[i].GetTl()).x);
                //         int x = m * cos(angle1) + n * sin(angle1);
                //         int y = m * sin(angle1) - n * cos(angle1);
                //         if(search_rect.contains(cv::Point(x, y))){
                //             image_without_color.at<uchar>(x-search_rect.tl().x, y-search_rect.tl().y) = 0;
                //         }
                //     }
                // }
                // for(int m=0; m<=light_bars[j].GetHeight(); m++){
                //     for(int n=0; n<=light_bars[j].GetWidth(); n++){
                //         float angle1 = atan((light_bars[j].GetTr()-light_bars[j].GetTl()).y/(light_bars[j].GetTr()-light_bars[j].GetTl()).x);
                //         int x = m * cos(angle1) + n * sin(angle1);
                //         int y = m * sin(angle1) - n * cos(angle1);
                //         if(search_rect.contains(cv::Point(x, y))){
                //             image_without_color.at<uchar>(x-search_rect.tl().x, y-search_rect.tl().y) = 0;
                //         }
                //     }
                // }
                // cv::imshow("image_without_color", image_without_color);
                // cv::cvtColor(image_numb, image_without_color, cv::COLOR_BGR2GRAY);
                //                cv::GaussianBlur(image_without_color,image_without_color,cv::Size(3,3),0,0);
                //                cv::blur(image_without_color,image_without_color,cv::Size(5,5));

                int thre = RegionOustThreshold(image_without_color, image_without_color, 0);
                //				if((sum(image_without_color)[0] / 255) * 5 > (image_without_color.size().area() * 3))
                //				{
                //					continue;
                //				}
                //				if((sum(image_without_color)[0] / 255) * 5 < (image_without_color.size().area()))
                //				{
                //					continue;
                //				}

                erode(image_without_color, image_without_color, this->element2_, cv::Point(-1, -1), 1);
                dilate(image_without_color, image_without_color, this->element2_, cv::Point(-1, -1), 2);
                                //    Debug::img=image_without_color.clone();
                                //    Debug::ShowMat();
                std::vector<std::vector<cv::Point>> contours;
                findContours(image_without_color, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE,
                             search_rect.tl()); //查找轮廓

                tdttoolkit::CustomRect Numb_rect;
                // Numb_rect.SetSize(cv::Size(0, 0));
                tdttoolkit::CustomRect temp_numb_rect;

                light_bar.SetAngle(0.5 * (light_bars[i].GetAngle() + light_bars[j].GetAngle()));
                std::sort(contours.begin(), contours.end(), [](std::vector<cv::Point> &a, std::vector<cv::Point> &b) -> bool { return a.size() > b.size(); });
                uint8_t time = 0, last_contour = 0;
                for (auto &contour : contours) //遍历轮廓  只遍历包含点最多的两个contour
                {
                    if (time >= 2) {
                        break;
                    }
                    time++;
#ifdef LIGHT_BAR_MATCH_DEBUG
                    std::cout << i << "-" << j << "contour.size():" << contour.size() << "\n";
#endif                                       // LIGHT_BAR_MATCH_DEBUG
                    if (contour.size() < 10) //包含的点太少不要
                    {
                        continue; //点太少的不要
                    }
                    temp_numb_rect = tdttoolkit::CustomRect::minCustomRect(contour, light_bar.GetAngle());
                    //                    if (temp_numb_rect.GetRect().width > 1.3 * temp_numb_rect.GetRect().height) //筛掉横向区域
                    //                    {
                    //#ifdef LIGHT_BAR_MATCH_DEBUG
                    //                        std::cout << "横向区域!*****************************\n";
                    //#endif // LIGHT_BAR_MATCH_DEBUG
                    //                        continue;
                    //                    }
                    //#ifdef LIGHT_BAR_MATCH_DEBUG
                    //                    std::cout << temp_numb_rect.GetSize().area() / search_rect.area()<< "area()*****************************\n";
                    //#endif // LIGHT_BAR_MATCH_DEBUG
                    if (temp_numb_rect.GetSize().area() < 0.15 * search_number_rect.GetSize().area()) //面积太小不要
                    {
                        std::cout<<"面积不对"<<temp_numb_rect.GetSize().area()/search_number_rect.GetSize().area()<<std::endl;
                        continue;
                    }

                    //计算的两灯条中心距离数字贴纸中心的距离
                    float dist1 = tdttoolkit::CalcDistance(cv::Point(temp_numb_rect.GetCenter()), light_bars[i].GetCenter());
                    float dist2 = tdttoolkit::CalcDistance(cv::Point(temp_numb_rect.GetCenter()), light_bars[j].GetCenter());
                    float dist3 = tdttoolkit::CalcDistance(cv::Point(temp_numb_rect.GetCenter()), (light_bars[i].GetCenter()+light_bars[j].GetCenter())/2);
                    //如果数字贴纸离灯条确定的中心太远, 筛掉
                    if (light_bars[i].GetWidth() > light_bars[j].GetWidth()) {
                        if (dist1 > 1.7 * dist2 || dist2 > 1.8 * dist1) {
#ifdef LIGHT_BAR_MATCH_DEBUG
                            std::cout << "1.数字贴纸离灯条确定的中心太远!" << (float)dist1 / dist2 << "\n";
#endif // LIGHT_BAR_MATCH_DEBUG
                            continue;
                        }
                        std::cout<<"dist3 / dist2: "<<dist3 / dist2<<std::endl;
                        if (dist3 / dist2 > 0.3) continue;
                    } else {
                        if (dist1 > 1.8 * dist2 || dist2 > 1.7 * dist1) {
#ifdef LIGHT_BAR_MATCH_DEBUG
                            std::cout << "2.数字贴纸离灯条确定的中心太远!" << (float)dist1 / dist2 << "\n";
#endif // LIGHT_BAR_MATCH_DEBUG
                            continue;
                        }
                        std::cout<<"dist3 / dist1: "<<dist3 / dist2<<std::endl;
                        if (dist3 / dist1 > 0.3) continue;
                    }
#ifdef LIGHT_BAR_MATCH_DEBUG
                    std::cout << last_contour << "*******************lastcontour\n";
#endif // LIGHT_BAR_MATCH_DEBUG
                    // if (Numb_rect.GetSize().area() != 0 && contour.size() > last_contour / 3) {
                    //     std::vector<cv::Point> temp = Numb_rect.GetVertices();
                    //     contour.insert(contour.end(), temp.begin(), temp.end());
                    //     Numb_rect = CustomRect::minCustomRect(contour, light_bar.GetAngle());
                    //     std::vector<cv::Point>().swap(temp);
                    // } else if (Numb_rect.GetSize().area() < temp_numb_rect.GetSize().area()) {
                    //     Numb_rect    = temp_numb_rect;
                    //     last_contour = contour.size();
                    // } //保存可能存在的数字区域进列表
                    Numb_rect    = temp_numb_rect;
                    
                }
                // std::cout<<(float)Numb_rect.GetRect().width/Numb_rect.GetRect().height<<"****************************横向区域\n";
                if (Numb_rect.GetRect().width > 1.8 * Numb_rect.GetRect().height) //筛掉横向区域
                {
#ifdef LIGHT_BAR_MATCH_DEBUG
                    std::cout << "识别为横向区域!*****************************\n";
#endif // LIGHT_BAR_MATCH_DEBUG
                    continue;
                }
#ifdef LIGHT_BAR_MATCH_DEBUG
                std::cout << (float)Numb_rect.GetRect().area() / search_rect.area() << "面积比！***************\n";
#endif                                                                       // LIGHT_BAR_MATCH_DEBUG
                // if (Numb_rect.GetRect().area() > 1.2 * search_rect.area()) { //面积太大
                //     continue;
                // }
                if (Numb_rect.GetSize().area() == 0) {
#ifdef LIGHT_BAR_MATCH_DEBUG
                    std::cout << "面积为0!\n";
#endif // LIGHT_BAR_MATCH_DEBUG
                    continue;
                }
#ifdef VIDEO_DEBUG
                // Debug::AddRotatedRect("双灯条检测潜在装甲板区域", temp_numb_rect.GetRotRect(), cv::Scalar(0, 0, 255), 2);
                Debug::AddCustomRect("数字区域范围", Numb_rect, cv::Scalar(155, 155, 255), 2);
#endif // VIDEO_DEBUG
       //                else
       //                {
       //                    sort(Numb_rects.begin(), Numb_rects.end(), 
       //                            [](const tdttoolkit::CustomRect& a, const tdttoolkit::CustomRect& b) -> bool
       //                            {
       //                                return a.GetSize().area() > b.GetSize().area();
       //                            });//以旋转矩形面积从大到小排序
       //                }
                ArmorDetectInfo tmp_armor(light_bars[i], light_bars[j], Numb_rect, thre);
                CalaArmorRotRect(tmp_armor);
                light_bars[i].SetFindStatus(light_bars[i].GetFindStatus() == -1 ? 2 : 1); //-1表示此灯条寻找过左边
                light_bars[j].SetFindStatus(light_bars[j].GetFindStatus() == 1 ? 2 : -1); // 1表示寻找过右边
                armors_detected_vector[i].push_back(tmp_armor);
                break;
            }
            if (!armors_detected_vector[i].empty()) {
                output_armors.insert(output_armors.end(), armors_detected_vector[i].begin(), armors_detected_vector[i].end());
            }
            std::vector<ArmorDetectInfo>().swap(armors_detected_vector[i]);
        }
        std::vector<std::vector<ArmorDetectInfo>>().swap(armors_detected_vector);
        double t3 = cv::getTickCount();
        std::cout << "[Get]DoubleDetect耗时" << (t3 - t0) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
    }

    //多线程双灯条匹配
    /*void ArmorDetector::DoubleLightBarDetect(const cv::Mat &src, std::vector<LightBarInfo> &light_bars, std::vector<ArmorDetectInfo> &output_armors) {
        double                                    t0 = cv::getTickCount();
        std::vector<std::vector<ArmorDetectInfo>> armors_detected_vector(light_bars.size() - 1);
        std::vector<std::thread>                  threads(light_bars.size() - 1);
        ////--------双灯条封装Armor---------
        auto func = [this, &light_bars](const cv::Mat &src, int i, std::vector<ArmorDetectInfo> &armors_detected) -> void {
            double t1 = cv::getTickCount();
            for (uint64_t j = i + 1; j < light_bars.size(); j++) {
#ifdef LIGHT_BAR_MATCH_DEBUG
                std::cout << i << "-" << j << ":灯条匹配：";
#endif // LIGHT_BAR_MATCH_DEBUG

                if (!IsLightBarMatched(light_bars[i], light_bars[j])) //不是匹配的灯条, 跳
                {
                    continue;
                }
                ////----------算出查找数字贴纸的区域---------------
                LightBarInfo light_bar          = light_bars[i].GetWidth() > light_bars[j].GetWidth() ? light_bars[i] : light_bars[j];
                //CustomRect   search_number_rect = CustomRect((light_bars[i].GetCenter() + light_bars[j].GetCenter()) / 2, cv::Size((light_bars[i].GetWidth() + light_bars[j].GetWidth()), 0.7 * (light_bars[j].GetCenter().x - light_bars[i].GetCenter().x)), (light_bars[i].GetAngle() + light_bars[j].GetAngle()) / 2);

                CustomRect search_number_rect;
                if ((light_bars[i].GetWidth() + light_bars[j].GetWidth()) > 0.6 * (light_bars[j].GetCenter().x - light_bars[i].GetCenter().x)) {
                    search_number_rect = CustomRect((light_bars[i].GetCenter() + light_bars[j].GetCenter()) / 2,
                                                    cv::Size((light_bars[i].GetWidth() + light_bars[j].GetWidth()),
                                                             0.6 * (light_bars[j].GetCenter().x -light_bars[i].GetCenter().x)),
                                                    (light_bars[i].GetAngle() + light_bars[j].GetAngle()) / 2);
                } else {
                    search_number_rect = CustomRect((light_bars[i].GetCenter() + light_bars[j].GetCenter()) / 2,
                                                    cv::Size((light_bars[i].GetWidth() + light_bars[j].GetWidth()),
                                                             0.6 * (light_bars[j].GetCenter().x -light_bars[i].GetCenter().x)),
                                                    90 + (light_bars[i].GetAngle() + light_bars[j].GetAngle()) / 2);
                }
                cv::Rect search_rect = search_number_rect.GetRect();
                if (!RectSafety(search_rect)) //若矩形在图像外, 跳过, 否则取
                {
                    continue; //安全性
                }
#ifdef VIDEO_DEBUG  //灯条匹配的结果
                Debug::AddCustomRect("双灯条检测潜在装甲板区域", search_number_rect, cv::Scalar(255, 192, 203), 2);
                //Debug::AddText("双灯条检测潜在装甲板区域", std::to_string(j), search_rect.tl(), cv::Scalar(255, 192, 203), 2);
#endif // VIDEO_DEBUG
                ////-------------------------------------------

                cv::Mat image_numb = src(search_rect); //取感兴趣区域
                cv::Mat image_without_color;
                cv::extractChannel(image_numb, image_without_color, 1);

                int thre = RegionOustThreshold(image_without_color, image_without_color, 0);

                //				if((sum(image_without_color)[0] / 255) * 5 > (image_without_color.size().area() * 3))
                //				{
                //					continue;
                //				}
                //				if((sum(image_without_color)[0] / 255) * 5 < (image_without_color.size().area()))
                //				{
                //					continue;
                //				}

                erode(image_without_color, image_without_color, this->element2_, cv::Point(-1, -1), 2);
                dilate(image_without_color, image_without_color, this->element2_, cv::Point(-1, -1), 1);
//                    Debug::img=image_without_color.clone();
//                    Debug::ShowMat();

                std::vector<std::vector<cv::Point>> contours;
                findContours(image_without_color, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE,search_rect.tl()); //查找轮廓

                tdttoolkit::CustomRect Numb_rect;
                Numb_rect.SetSize(cv::Size(0, 0));
                tdttoolkit::CustomRect temp_numb_rect;

                if (light_bar.GetAngle() == 0) {light_bar.SetAngle(0.5 * (light_bars[i].GetAngle() + light_bars[j].GetAngle()));}
                std::sort(contours.begin(),contours.end(),[](std::vector<cv::Point> &a,std::vector<cv::Point> &b)->bool{return a.size()>b.size();});
                int time=0,last_contour=0;
                for (auto &contour : contours) //遍历轮廓  只遍历包含点最多的两个contour
                {
                    if(time==2) {break;}
                    time++;
#ifdef LIGHT_BAR_MATCH_DEBUG
                    std::cout << i << "-" << j << "contour.size():" << contour.size() << "\n";
#endif // LIGHT_BAR_MATCH_DEBUG
                    if (contour.size() < 10) //包含的点太少不要
                    {
                        continue; //点太少的不要
                    }
                    temp_numb_rect = tdttoolkit::CustomRect::minCustomRect(contour, light_bar.GetAngle());
//                    if (temp_numb_rect.GetRect().width > 1.3 * temp_numb_rect.GetRect().height) //筛掉横向区域
//                    {
//#ifdef LIGHT_BAR_MATCH_DEBUG
//                        std::cout << "横向区域!*****************************\n";
//#endif // LIGHT_BAR_MATCH_DEBUG
//                        continue;
//                    }
//#ifdef LIGHT_BAR_MATCH_DEBUG
//                    std::cout << temp_numb_rect.GetSize().area() / search_rect.area()<< "area()*****************************\n";
//#endif // LIGHT_BAR_MATCH_DEBUG
//                    if (temp_numb_rect.GetSize().area() < 0.1 * search_rect.area()) //面积太小不要
//                    {
//                        continue;
//                    }

                    //计算的两灯条中心距离数字贴纸中心的距离
                    float dist1 = tdttoolkit::CalcDistance(cv::Point(temp_numb_rect.GetCenter()),light_bars[i].GetCenter());
                    float dist2 = tdttoolkit::CalcDistance(cv::Point(temp_numb_rect.GetCenter()),light_bars[j].GetCenter());

                    //如果数字贴纸离灯条确定的中心太远, 筛掉
                    if (light_bars[i].GetWidth() > light_bars[j].GetWidth()) {
                        if (dist1 > 1.75 * dist2 || dist2 > 1.8 * dist1) {
#ifdef LIGHT_BAR_MATCH_DEBUG
                            std::cout<<"1.数字贴纸离灯条确定的中心太远!"<<(float)dist1/dist2<<"\n";
#endif // LIGHT_BAR_MATCH_DEBUG
                            continue;
                        }
                    } else {
                        if (dist1 > 1.8 * dist2 || dist2 > 1.75 * dist1) {
#ifdef LIGHT_BAR_MATCH_DEBUG
                            std::cout<<"2.数字贴纸离灯条确定的中心太远!"<<(float)dist1/dist2<<"\n";
#endif // LIGHT_BAR_MATCH_DEBUG
                            continue;
                        }
                    }
#ifdef LIGHT_BAR_MATCH_DEBUG
                    std::cout<<last_contour<<"*******************lastcontour\n";
#endif // LIGHT_BAR_MATCH_DEBUG
                    if(Numb_rect.GetSize().area()!=0&&contour.size()>last_contour/3)  {
                        std::vector<cv::Point> temp=Numb_rect.GetVertices();
                        contour.insert(contour.end(),temp.begin(),temp.end());
                        Numb_rect=CustomRect::minCustomRect(contour,light_bar.GetAngle());
                        std::vector<cv::Point> ().swap(temp);
                    }
                    else if (Numb_rect.GetSize().area() <temp_numb_rect.GetSize().area())
                        {Numb_rect = temp_numb_rect; last_contour=contour.size(); } //保存可能存在的数字区域进列表
                }
                //std::cout<<(float)Numb_rect.GetRect().width/Numb_rect.GetRect().height<<"****************************横向区域\n";
                if (Numb_rect.GetRect().width > 1.2*Numb_rect.GetRect().height) //筛掉横向区域
                {
#ifdef LIGHT_BAR_MATCH_DEBUG
                    std::cout << "识别为横向区域!*****************************\n";
#endif // LIGHT_BAR_MATCH_DEBUG
                    continue;
                }
#ifdef LIGHT_BAR_MATCH_DEBUG
                std::cout<<(float)Numb_rect.GetRect().area()/search_rect.area()<<"面积比！***************\n";
#endif // LIGHT_BAR_MATCH_DEBUG
                if(Numb_rect.GetRect().area()>1.2*search_rect.area()) {  //面积太大
                    continue;
                }
                if (Numb_rect.GetSize().area() == 0) {
#ifdef LIGHT_BAR_MATCH_DEBUG
                    std::cout<<"面积为0!\n";
#endif // LIGHT_BAR_MATCH_DEBUG
                    continue;
                }
#ifdef VIDEO_DEBUG
                //Debug::AddRotatedRect("双灯条检测潜在装甲板区域", temp_numb_rect.GetRotRect(), cv::Scalar(0, 0, 255), 2);
                Debug::AddCustomRect("双灯条检测潜在装甲板区域", Numb_rect, cv::Scalar(0, 0, 255), 2);
#endif // VIDEO_DEBUG
                //                else
                //                {
                //                    sort(Numb_rects.begin(), Numb_rects.end(), *
                //                            [](const tdttoolkit::CustomRect& a, const tdttoolkit::CustomRect& b) -> bool
                //                            {
                //                                return a.GetSize().area() > b.GetSize().area();
                //                            });//以旋转矩形面积从大到小排序
                //                }
                ArmorDetectInfo tmp_armor(light_bars[i], light_bars[j], Numb_rect, thre);
                CalaArmorRotRect(tmp_armor);
                light_bars[i].SetFindStatus(light_bars[i].GetFindStatus() == -1 ? 2 : 1); //-1表示此灯条寻找过左边
                light_bars[j].SetFindStatus(light_bars[j].GetFindStatus() == 1 ? 2 : -1); // 1表示寻找过右边
                armors_detected.push_back(tmp_armor);
                break;
            }
            double t2 = cv::getTickCount();
            std::cout << "[Get][DoubleDetect]func耗时" << (t2 - t1) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
        };
        double t6 = cv::getTickCount();
        for (uint64_t i = 0; i < (light_bars.size() - 1); ++i) //双层循环 遍历每一对矩形框
        {
            threads[i] = std::thread(func, std::ref(src), i, std::ref(armors_detected_vector[i]));
#ifdef DOUBLE_LIGHT_BAR_DEBUG
            TDT_DEBUG("{双灯条}  线程%d启动", i);
#endif // DOUBLE_LIGHT_BAR_DEBUG
        }
        std::cout << "++++++++++++耗时" << (cv::getTickCount() - t6) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
        double t7 = cv::getTickCount();
        for (uint64_t i = 0; i < (light_bars.size() - 1); ++i) {
            if (threads[i].joinable()) {
                threads[i].join();
            }
#ifdef DOUBLE_LIGHT_BAR_DEBUG
            TDT_DEBUG("{双灯条}  线程%d结束", i);
            TDT_DEBUG("{双灯条}  第%d个灯条有%d个成员", i, armors_detected_vector[i].size());
#endif // DOUBLE_LIGHT_BAR_DEBUG
            if (!armors_detected_vector[i].empty()) {
                output_armors.insert(output_armors.end(), armors_detected_vector[i].begin(), armors_detected_vector[i].end());
            }
            std::vector<ArmorDetectInfo>().swap(armors_detected_vector[i]);
        }
        std::cout << "***********耗时" << (cv::getTickCount() - t7) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
        std::vector<std::thread>().swap(threads);
        std::vector<std::vector<ArmorDetectInfo>>().swap(armors_detected_vector);
        double t3 = cv::getTickCount();
        std::cout << "[Get]DoubleDetect耗时" << (t3 - t0) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
    }*/

    void ArmorDetector::NumberStickerDetect(const cv::Mat &src, std::vector<ArmorDetectInfo> &output_armors) {
        std::vector<std::vector<ArmorDetectInfo>> armors_detected_vector(this->last_armors_info_.size());

        for (uint64_t i = 0; i < this->last_armors_info_.size(); ++i) {
            cv::Rect safe_rect;
            safe_rect = this->last_armors_info_[i].GetArmorRotRect().GetRect();
            safe_rect = RectEnlarge(safe_rect, cv::Size2f(1.1, 1.1));
            if (!RectSafety(safe_rect)) {
                return;
            }
            cv::Mat                             src_roi = src(safe_rect); // ROI为上一帧装甲板的周围扩大
            // safe_rect = RectEnlarge(safe_rect, cv::Size2f(1.2, 1));
            // cv::Mat                             all_roi = src(safe_rect);
            // cv::Scalar scalar = cv::sum(all_roi);
            // if(scalar[2 - enemy_color_] > scalar[enemy_color_])
            //     continue;
            std::vector<std::vector<cv::Point>> contours;
            //            std::vector<cv::Mat> split_mat;
            //            split(src_roi, split_mat);
            //            src_roi = split_mat[1];//消除灯条干扰0蓝1绿2红
            cv::Mat image_without_color;
            cv::extractChannel(src_roi, image_without_color, 2-enemy_color_);

            erode(image_without_color, image_without_color, this->element2_, cv::Point(-1, -1), 1);
            dilate(image_without_color, image_without_color, this->element2_, cv::Point(-1, -1), 2);

            cv::threshold(image_without_color, image_without_color, fmax(20, this->last_armors_info_[i].GetThreshold()), 255, cv::THRESH_BINARY); //根据上一帧装甲板计算出的阈值二值化图片

            findContours(image_without_color, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, safe_rect.tl()); //提取外轮廓

            for (auto &contour : contours) //遍历所有轮廓
            {
                if (contour.size() < 20) {
                    continue; //轮廓面积的绝对值
                }
                float                  angle    = this->last_armors_info_[i].GetNumberRotRect().GetAngle();
                tdttoolkit::CustomRect rot_rect = tdttoolkit::CustomRect::minCustomRect(contour, angle);
                if (rot_rect.GetSize().area() < this->last_armors_info_[i].GetNumberRotRect().GetSize().area() * 0.5) {
                    continue;
                }
                cv::Rect rect = rot_rect.GetRect();
                if (!RectSafety(rect)) {
                    continue;
                }
                if (rect != rot_rect.GetRect()) {
                    continue;
                }
#ifdef VIDEO_DEBUG
                Debug::AddRotatedRect("数字区域范围", rot_rect.GetRotRect(), cv::Scalar(255, 192, 203), 2);
#endif
                ArmorDetectInfo tmp(rot_rect, this->last_armors_info_[i].GetThreshold());
                CalaArmorRotRect(tmp);
                armors_detected_vector[i].push_back(tmp);
            }
        };

        for (uint64_t i = 0; i < this->last_armors_info_.size(); ++i) {
#ifdef NO_LIGHT_BAR_DEBUG
            TDT_DEBUG("{无灯条}  循环%d结束", i);
            TDT_DEBUG("{无灯条}  第%d个last_armor有%d个成员", i, armors_detected_vector[i].size());
#endif // NO_LIGHT_BAR_DEBUG
            if (!armors_detected_vector[i].empty()) {
                output_armors.insert(output_armors.end(), armors_detected_vector[i].begin(), armors_detected_vector[i].end());
            }
            std::vector<ArmorDetectInfo>().swap(armors_detected_vector[i]);
        }
        std::vector<std::vector<ArmorDetectInfo>>().swap(armors_detected_vector);
#ifdef LIGHT_BAR_MATCH_DEBUG
        std::cout << "调用无灯条识别，识别出装甲板：output_armors.size():" << output_armors.size() << "个！\n";
#endif // LIGHT_BAR_MATCH_DEBUG
    }

    bool ArmorDetector::IsLightBarMatched(const LightBarInfo &left_light_bar, const LightBarInfo &right_light_bar) {
        if (left_light_bar.GetCenter().x >= right_light_bar.GetCenter().x) //如果左灯条在右灯条右边, 或者两灯条x相等, 返回假
        {
#ifdef LIGHT_BAR_MATCH_DEBUG
            //            Debug::SetMessage("双灯条匹配", "灯条不匹配原因", "两灯条位置x错误");
            //            TDT_DEBUG("{灯条匹配}  两灯条位置x错误");
            std::cout << "双灯条匹配,灯条不匹配原因, 两灯条位置x错误\n";
#endif // LIGHT_BAR_MATCH_DEBUG
            return false;
        }

        if (fabs(left_light_bar.GetAngle() - right_light_bar.GetAngle()) > 15) //平行度不好, 返回假
        {
#ifdef LIGHT_BAR_MATCH_DEBUG
            //            Debug::SetMessage("双灯条匹配", "灯条不匹配原因", "两灯条平行度不好");
            //            TDT_DEBUG("{灯条匹配}  两灯条平行度不好");
            std::cout << "双灯条匹配,灯条不匹配原因, 两灯条平行度不好\n";
#endif // LIGHT_BAR_MATCH_DEBUG
            return false;
        }

        if (atan(fabs(left_light_bar.GetCenter().y - right_light_bar.GetCenter().y) / fabs(left_light_bar.GetCenter().x - right_light_bar.GetCenter().x)) > 0.4) //中心角度过大, 返回假
        {
#ifdef LIGHT_BAR_MATCH_DEBUG
            //            Debug::SetMessage("双灯条匹配", "灯条不匹配原因", "两灯条中心角度过大");
            //            TDT_DEBUG("{灯条匹配}  两灯条中心角度过大");
            std::cout << "双灯条匹配,灯条不匹配原因, 两灯条中心角度过大\n";
#endif // LIGHT_BAR_MATCH_DEBUG
            return false;
        }

//         if (fabs(left_light_bar.GetCenter().x - right_light_bar.GetCenter().x) > 9 * fmax(left_light_bar.GetWidth(), right_light_bar.GetWidth())) //横向太远, 返回假
//         {
// #ifdef LIGHT_BAR_MATCH_DEBUG
//             //            Debug::SetMessage("双灯条匹配", "灯条不匹配原因", "两灯条横向太远");
//             //            TDT_DEBUG("{灯条匹配}  两灯条横向太远");
//             std::cout << "双灯条匹配,灯条不匹配原因, 两灯条横向太远\n";
// #endif // LIGHT_BAR_MATCH_DEBUG
//             return false;
//         }

//         if (fabs(left_light_bar.GetCenter().x - right_light_bar.GetCenter().x) < 0.2 * fmax(left_light_bar.GetWidth(), right_light_bar.GetWidth())) //横向太近, 返回假
//         {
// #ifdef LIGHT_BAR_MATCH_DEBUG
//             //            Debug::SetMessage("双灯条匹配", "灯条不匹配原因", "两灯条横向太近");
//             //            TDT_DEBUG("{灯条匹配}  两灯条横向太近");
//             std::cout << "双灯条匹配,灯条不匹配原因, 两灯条横向太近\n";
// #endif // LIGHT_BAR_MATCH_DEBUG
//             return false;
//         }

        if (fabs(left_light_bar.GetWidth() - right_light_bar.GetWidth()) > 0.5 * fmax(left_light_bar.GetWidth(), right_light_bar.GetWidth())) //高度差诡异
        {
#ifdef LIGHT_BAR_MATCH_DEBUG
            //            Debug::SetMessage("双灯条匹配", "灯条不匹配原因", "两灯条高度差诡异");
            //            TDT_DEBUG("{灯条匹配}  两灯条高度差诡异");
            std::cout << "双灯条匹配,灯条不匹配原因, 两灯条高度差诡异\n";
#endif // LIGHT_BAR_MATCH_DEBUG
            return false;
        }

        float point_distance = tdttoolkit::CalcDistance(left_light_bar.GetCenter(), right_light_bar.GetCenter());

        if (point_distance > 6 * fmax(left_light_bar.GetWidth(), right_light_bar.GetWidth())) {
#ifdef LIGHT_BAR_MATCH_DEBUG
            //            Debug::SetMessage("双灯条匹配", "灯条不匹配原因", "两灯条中心距离太远");
            //            TDT_DEBUG("{灯条匹配}  两灯条中心距离太远");
            std::cout << "双灯条匹配,灯条不匹配原因, 两灯条中心距离太远\n";
#endif // LIGHT_BAR_MATCH_DEBUG
            return false;
        }
        if (point_distance < fmin(left_light_bar.GetWidth(), right_light_bar.GetWidth())*0.8) // NOLINT
        {
#ifdef LIGHT_BAR_MATCH_DEBUG
            //            Debug::SetMessage("双灯条匹配", "灯条不匹配原因", "两灯条中心距离太近");
            //            TDT_DEBUG("{灯条匹配}  两灯条中心距离太近");
            std::cout << "双灯条匹配,灯条不匹配原因,两灯条中心距离太近\n";
#endif // LIGHT_BAR_MATCH_DEBUG
            return false;
        }
//         if (fabs(left_light_bar.GetCenter().y - right_light_bar.GetCenter().y) > (left_light_bar.GetWidth() + right_light_bar.GetWidth()) / 2) {
// #ifdef LIGHT_BAR_MATCH_DEBUG
//             std::cout << "中心的y值差太大\n";
// #endif // LIGHT_BAR_MATCH_DEBUG
//             return false;
//         }
#ifdef LIGHT_BAR_MATCH_DEBUG
        std::cout << "双灯条匹配成功！！\n";
#endif // LIGHT_BAR_MATCH_DEBUG
        return true;
    }

    int ArmorDetector::RegionOustThreshold(const cv::Mat &input_image, cv::Mat &output_image, int lr) {
        cv::Mat  tmp;
        cv::Rect o_rect;
        switch (lr) {
        case -1:
            o_rect = cv::Rect(cv::Point(cvRound(0.5 * input_image.cols), cvRound(0.35 * input_image.rows)), cv::Point(cvRound(0.9 * input_image.cols), cvRound(0.65 * input_image.rows)));
            break;

        case 1:
            o_rect = cv::Rect(cv::Point(cvRound(0.1 * input_image.cols), cvRound(0.35 * input_image.rows)), cv::Point(cvRound(0.5 * input_image.cols), cvRound(0.65 * input_image.rows)));
            break;

        default:
            o_rect = cv::Rect(cv::Point(cvRound(0.3 * input_image.cols), cvRound(0.35 * input_image.rows)), cv::Point(cvRound(0.7 * input_image.cols), cvRound(0.65 * input_image.rows)));
            break;
        }
        int threshold = cvRound(cv::threshold(input_image(o_rect), tmp, 0, 255, cv::THRESH_OTSU));
        cv::threshold(input_image, output_image, threshold, 255, cv::THRESH_BINARY);
        return threshold;
    }

    void ArmorDetector::CalaArmorRotRect(ArmorDetectInfo &armor) {

        ////////////////////////////////////////////确定机器学习区域/////////////////////////////////////////////
        tdttoolkit::CustomRect roi;
        if (armor.HaveLeftBar() && armor.HaveRightBar()) {
            /*
                        roi = armor.GetNumberRotRect();
                        roi.SetSize(cv::Size(roi.GetSize().width * 1.1, roi.GetSize().height * 1.1));*/

            // std::cout<<"左边灯条角度："<<armor.GetLeftBarRect().GetAngle()<<" 右："<<armor.GetRightBarRect().GetAngle();
            float                  distance = tdttoolkit::CalcDistance(armor.GetLeftBarRect().GetCenter(), armor.GetRightBarRect().GetCenter()); //两个矩形框的距离
            tdttoolkit::CustomRect better_light_bar(cv::Point(0, 0), cv::Size(0, 0), 0);
            if (armor.GetLeftBarRect().GetAngle() != 90 && armor.GetRightBarRect().GetAngle() != 90) {
                better_light_bar = 0.5 * (armor.GetLeftBarRect().GetAngle() + armor.GetRightBarRect().GetAngle()) > 90 ? armor.GetLeftBarRect() : armor.GetRightBarRect();
            } else {
                better_light_bar = armor.GetLeftBarRect().GetWidth() < armor.GetRightBarRect().GetWidth() ? armor.GetLeftBarRect() : armor.GetRightBarRect();
            }
            cv::Size size;
            size.width       = fmax(better_light_bar.GetWidth() * 2.2, armor.GetNumberRotRect().GetWidth() * 1.1);
            size.height      = fmin(distance - 1.8 * better_light_bar.GetHeight(), size.width / 1.4);
            size.height      = fmax(size.height, armor.GetNumberRotRect().GetHeight() * 1.05);
            float     angle  = better_light_bar.GetAngle();
            cv::Point center = (armor.GetLeftBarRect().GetCenter() + armor.GetRightBarRect().GetCenter()) / 2;
            roi              = tdttoolkit::CustomRect(center, size, angle);
        } else if (armor.HaveLeftBar() || armor.HaveRightBar()) {
            tdttoolkit::CustomRect better_lightbar = armor.HaveRightBar() ? armor.GetRightBarRect() : armor.GetLeftBarRect();
            cv::Size               size;
            cv::Point              center     = armor.GetNumberRotRect().GetCenter();
            cv::Point              top_center = (better_lightbar.GetTl() + better_lightbar.GetTr()) / 2;

            //////////////////////////////////////////灯条间距////////////////////////////////////////////////
            int A = 0, B = 0, C = 0;
            A = top_center.y - better_lightbar.GetCenter().y;
            B = better_lightbar.GetCenter().x - top_center.x;
            C = top_center.x * better_lightbar.GetCenter().y - top_center.y * better_lightbar.GetCenter().x;
            //代入点到直线距离公式
            double distance = 2 * (fabs(A * center.x + B * center.y + C)) / (sqrt(A * A + B * B));
            ////////////////////////////////////////////////////////////////////////////////////////////////

            size.width  = fmax(cvRound(better_lightbar.GetWidth() * 2.2), cvRound(armor.GetNumberRotRect().GetHeight() * 1.1));
            size.height = fmin(cvRound(distance - 1.8 * better_lightbar.GetHeight()), cvRound(size.width / 1.4));
            size.height = fmax(size.height, cvRound(armor.GetNumberRotRect().GetHeight() * 1.05));
            float angle = better_lightbar.GetAngle();
            roi         = tdttoolkit::CustomRect(center, size, angle);
        } else {
            roi = armor.GetNumberRotRect();
            roi.SetSize(cv::Size(roi.GetSize().width * 1.1, roi.GetSize().height * 1.1));
        }
        ////////////////////////////////////////////////////////////////////////////////////////////////
        armor.SetArmorRotRect(roi);
    }

    void ArmorDetector::GetMlRoi(const ArmorDetectInfo &armor, const cv::Mat &src, cv::Mat &ml_roi) {
        tdttoolkit::CustomRect roi = armor.GetArmorRotRect();

        cv::Rect roi_mat     = roi.GetRect();
        cv::Rect roi_matcopy = cv::Rect(roi_mat);
        if (RectSafety(roi_matcopy)) { //取安全的矩形
            cv::Mat roiimg;
            if (roi_matcopy != roi_mat) {                         //如果取安全矩形对矩形进行过裁剪
                roiimg = cv::Mat::zeros(roi_mat.size(), CV_8UC3); //大小和原矩形一样 //NOLINT(hicpp-signed-bitwise)
                src(roi_matcopy).copyTo(roiimg(cv::Rect(roi_matcopy.tl() - roi_mat.tl(), roi_matcopy.size())));
            } else {
                roiimg = src(roi_mat);
            }
           
            float tlx = static_cast<float>(roi_mat.tl().x);
            float tly = static_cast<float>(roi_mat.tl().y);
            cv::Point2f tl = cv::Point2f(tlx, tly);
            cv::Point2f src[] = {roi.GetTl() - tl, roi.GetTr() - tl, roi.GetBr() - tl};
            cv::Point2f dst[] = {cv::Point2f(0, 0), cv::Point2f(28, 0), cv::Point2f(28, 28)};
            cv::Mat rotation_mat = cv::getAffineTransform(src, dst);
            warpAffine(roiimg, ml_roi, rotation_mat,cv::Size(28, 28));
            if (ml_roi.empty()) {
                ml_roi = cv::Mat::zeros(28, 28, CV_8UC3); // NOLINT(hicpp-signed-bitwise)
            }
        } else {
            ml_roi = cv::Mat::zeros(28, 28, CV_8UC3); // NOLINT(hicpp-signed-bitwise)
        }
        cvtColor(ml_roi, ml_roi, cv::COLOR_BGR2GRAY);
        // cv::imwrite(std::to_string(cv::getTickCount())+".png", ml_roi);
        //        Debug::img=ml_roi.clone();
        //        Debug::ShowMat();
    }

    bool ArmorDetector::IsArmorNearby(const ArmorDetectInfo &armor_a, const ArmorDetectInfo &armor_b) {
        if (fabs(armor_a.GetRect().x - armor_b.GetRect().x) * 3 < (armor_a.GetRect().width)) {
            if (fabs(armor_a.GetRect().y - armor_b.GetRect().y) * 3 < (armor_a.GetRect().height)) {
                return true;
            }
        }
        return false;
    }

    bool ArmorDetector::IsArmorSimilar(const ArmorDetectInfo &armor_a, const ArmorDetectInfo &armor_b) {
        if (fabs(armor_a.GetArmorRotRect().GetSize().area() - armor_b.GetArmorRotRect().GetSize().area()) < armor_a.GetArmorRotRect().GetSize().area() * 0.2) {
            return true;
        }
        return false;
    }

    void ArmorDetector::MatchWithLastArmor(const std::vector<ArmorDetectInfo> &input_armors, std::vector<ArmorDetectInfo> &output_armors) {
        std::vector<std::vector<ArmorDetectInfo>> armors_matched_vector(this->last_armors_info_.size());
        std::vector<std::thread>                  threads(this->last_armors_info_.size());
        std::vector<ArmorDetectInfo>              output_tmp_armors;
        auto                                      func = [this, &input_armors](const ArmorDetectInfo &last_armor, std::vector<ArmorDetectInfo> &armors_matched) -> void {
            for (auto const &armor : input_armors) {
                if (IsArmorNearby(last_armor, armor)) {
                    ArmorDetectInfo armor_tmp(armor);
                    armor_tmp.SetArmorType(last_armor.GetArmorType());
                    armors_matched.emplace_back(armor_tmp);
                }
            }
        };
        for (int i = 0; i < this->last_armors_info_.size(); i++) {
            threads[i] = std::thread(func, std::ref(this->last_armors_info_[i]), std::ref(armors_matched_vector[i]));
#ifdef ARMOR_DETECT_DEBUG
            std::cout << "线程" << i << "启动" << std::endl;
#endif
        }
        for (int i = 0; i < this->last_armors_info_.size(); ++i) {
            if (threads[i].joinable()) {
                threads[i].join();
            }
#ifdef ARMOR_DETECT_DEBUG
            std::cout << "线程" << i << "结束" << std::endl;
            std::cout << "第" << i << "个last_armor有" << armors_matched_vector[i].size() << "个成员" << std::endl;
#endif
            if (armors_matched_vector[i].size() == 1) {
                output_tmp_armors.emplace_back(armors_matched_vector[i][0]);
            }
            std::vector<ArmorDetectInfo>().swap(armors_matched_vector[i]);
        }
        std::vector<std::thread>().swap(threads);
        std::vector<std::vector<ArmorDetectInfo>>().swap(armors_matched_vector);
        if (output_tmp_armors.size() != this->last_armors_info_.size()) {
            std::vector<ArmorDetectInfo>().swap(output_tmp_armors);
        }
        output_tmp_armors.swap(output_armors);
        std::vector<ArmorDetectInfo>().swap(output_tmp_armors);
    }

    /*******************************************************************************************************************
     * 工具
     ******************************************************************************************************************/

    bool ArmorDetector::RectSafety(cv::Rect2i &rect) {
        rect = rect & this->src_area_;
        return !rect.empty();
    }

    cv::Rect2i ArmorDetector::RectEnlarge(const cv::Rect2i &rect, const cv::Size2i &gain) {
        cv::Rect2i output_rect;
        cv::Size   arg = cv::Size((gain.width - 1) * rect.size().width, (gain.height - 1) * rect.size().height);
        output_rect    = rect + arg;
        output_rect    = output_rect - cv::Point2i(arg.width / 2, arg.height / 2);
        RectSafety(output_rect);
        return output_rect;
    }

    void ArmorDetector::ArmorTransform(const tdtrobot::ArmorDetector::ArmorDetectInfo &armor_info, tdttoolkit::RobotArmor &armor) {
        /////////////////////////////////计算图像点////////////////////////////////////
        //从左到右依次是 左灯条{0上定点,1下顶点,2中心点} 数字贴纸{3左中心点,4右中心点,5上中心点,6下中心点,7中心点} 右灯条{8上定点,9下顶点,10中心点}
        std::vector<cv::Point2f> image_point_lists = std::vector<cv::Point2f>(11);

        // image_point_lists[3] = (armor_info.GetArmorRotRect().GetTl() + armor_info.GetArmorRotRect().GetBl()) / 2;
        // image_point_lists[4] = (armor_info.GetArmorRotRect().GetTr() + armor_info.GetArmorRotRect().GetBr()) / 2;
        // image_point_lists[5] = (armor_info.GetArmorRotRect().GetTl() + armor_info.GetArmorRotRect().GetTr()) / 2;
        // image_point_lists[6] = (armor_info.GetArmorRotRect().GetBl() + armor_info.GetArmorRotRect().GetBr()) / 2;
        image_point_lists[7] = armor_info.GetArmorRotRect().GetCenter();

        if (armor_info.HaveLeftBar() && armor_info.HaveRightBar()) {
            image_point_lists[0] = (armor_info.GetLeftBarRect().GetTl() + armor_info.GetLeftBarRect().GetTr()) / 2;
            image_point_lists[1] = (armor_info.GetLeftBarRect().GetBl() + armor_info.GetLeftBarRect().GetBr()) / 2;
            image_point_lists[2] = armor_info.GetLeftBarRect().GetCenter();

            image_point_lists[8]  = (armor_info.GetRightBarRect().GetTl() + armor_info.GetRightBarRect().GetTr()) / 2;
            image_point_lists[9]  = (armor_info.GetRightBarRect().GetBl() + armor_info.GetRightBarRect().GetBr()) / 2;
            image_point_lists[10] = armor_info.GetRightBarRect().GetCenter();
        } else if (armor_info.HaveLeftBar()) {
            image_point_lists[0] = (armor_info.GetLeftBarRect().GetTl() + armor_info.GetLeftBarRect().GetTr()) / 2;
            image_point_lists[1] = (armor_info.GetLeftBarRect().GetBl() + armor_info.GetLeftBarRect().GetBr()) / 2;
            image_point_lists[2] = armor_info.GetLeftBarRect().GetCenter();
        } else if (armor_info.HaveRightBar()) {
            image_point_lists[9]  = (armor_info.GetRightBarRect().GetBl() + armor_info.GetRightBarRect().GetBr()) / 2;
            image_point_lists[8]  = (armor_info.GetRightBarRect().GetTl() + armor_info.GetRightBarRect().GetTr()) / 2;
            image_point_lists[10] = armor_info.GetRightBarRect().GetCenter();
        }
        int believable;
        if (!armor_info.HaveLeftBar() && !armor_info.HaveRightBar())
            believable = 0;
        else if (armor_info.HaveLeftBar() && armor_info.HaveRightBar())
            believable = 1;
        else
            believable = armor_info.HaveLeftBar() ? 2 : 3;
        armor = tdttoolkit::RobotArmor(armor_info.GetArmorRotRect(), armor_info.GetArmorType(), image_point_lists, believable);
    }
} // namespace tdtrobot
