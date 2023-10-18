
#ifndef TDTVision_RM2021_ERERGY_DETECT_H
#define TDTVision_RM2021_ERERGY_DETECT_H

#include"tdtcommon.h"
#include "Debug.h"
#include"log.h"

namespace tdtbuff{

    class BuffDetector {

    public:
        BuffDetector();
        std::vector<tdttoolkit::BuffArmor> Get(cv::Mat &src);
        bool disturb = false;

        inline void SetTimes(bool first_times){first_times_=first_times;}
    private:
        cv::Ptr<cv::ml::SVM> svm = cv::ml::StatModel::load<cv::ml::SVM>("../../model/flowrect2.xml");
        //float svm_flow_water_sum = 0;
        std::vector<tdttoolkit::BuffArmor> history_armors;
        bool first_times_=false;
        int enemy_color_ = 0;
        int times;
        float last_radius = - 1;
        float last_armor_area = - 1;
        cv::Rect2i src_area_;
        cv::Rect bounding_rect_;
        tdttoolkit::CustomRect flow_rect_;
        tdttoolkit::CustomRect r_big_rect_;
        cv::Mat pnp_rvec_object_to_world_0;
        cv::Mat pnp_tvec_object_to_world_0;

        cv::Ptr<cv::ml::KNearest> knn_Model_r = cv::ml::StatModel::load<cv::ml::KNearest> ("../../model/knn(flowwater).xml");

        cv::Ptr<cv::ml::KNearest> knn_Model_flow_water = cv::ml::StatModel::load<cv::ml::KNearest> (
                "../../model/knn(flowwater).xml");

        cv::HOGDescriptor *flow_water_hog = new cv::HOGDescriptor( cv::Size (96, 28),
                                                                   cv::Size (32, 28),
                                                                   cv::Size (8, 7),
                                                                   cv::Size (16, 14), 9);

        cv::HOGDescriptor *r_hog = new cv::HOGDescriptor( cv::Size (28, 28),
                                                          cv::Size (14, 14),
                                                          cv::Size (7, 7),
                                                          cv::Size (7, 7), 9 );

/**
        * @name    KnnRCompute
        * @brief   KnnR 的计算
        */
        void KnnRCompute(cv::Mat &r_img,float &knn_r);
/**
        * @name    KnnRCompute
        * @brief   KnnR 的计算
        */
        void KnnFlowCompute(cv::Mat & armor_img,float & knn_flow_water);

        static void FlowWaterContour(cv::Mat &armor_img,cv::Rect &bounding_rect,float &small_contour,float &big_contour);

        /**
        * @name    GetFlowWaterRect
        * @brief   得到流水灯矩形
        */
        void GetFlowWaterRect(const cv::Point2f &circle_point,const tdttoolkit::CustomRect &custom_rect);

        /**
        * @brief   得到大R矩形
        * @param   contour_rect    输入轮廓
        * @param
        */
        void  GetRBigCustomRect(const tdttoolkit::CustomRect &contour_rect,bool order = true);

        /**
        * @brief   检测装甲版
        * @param   src    输入图像
        * @param   armors 装甲板vector,检测到到的所有armor
        */
        void Detect(std::vector<tdttoolkit::BuffArmor> &armors, const cv::Mat &src);




        void GetFlowArmor(std::vector<tdttoolkit::BuffArmor> &armors,std::vector<tdttoolkit::BuffArmor> &ret_armors);

        /**
       * @brief   找圆心
       * @param   Mat bin  入二值图,以后可能改为轮廓vector;
       * @param   RotatedRec  寻找区域
       */
        void DetectorCircle( tdttoolkit::BuffArmor &buff_armor,
                                    const std::vector<std::vector<cv::Point>> &contours,
                                    cv::Mat &src);

        /**
        * @brief   判断是不是流水灯
        * @param   src    输入原图
        * @param   buff_armor 输入的装甲版
        * @return  EnergyBuffType //能量机关种类 1是流水灯
        */
        tdttoolkit::BuffType FlowWaterLight( tdttoolkit::BuffArmor &buff_armor,
                                             const cv::Mat &src,
                                             const cv::Point2f &circle_point,
                                             float &rotate_angle);


        void ArmorWorldCenter( tdttoolkit::BuffArmor &buff_armor, cv::Point3f &armor_world_center);

        void GetPoint2D( tdttoolkit::BuffArmor &buff_armor, std::vector<cv::Point2f>&point2D);

        void GetPoint3D( tdttoolkit::BuffArmor &buff_armor, std::vector<cv::Point3f>&point3D);

        void SimplePnpResolve(const std::vector<cv::Point2f>&point2D,
                          const std::vector<cv::Point3f>&point3D,
                          tdttoolkit::BuffArmor &buff_armor,
                          cv::Point3f &armor_world_center);

    };

} //namespace tdtbuff
#endif //TDTVision_RM2021_ERERGY_DETECT_H