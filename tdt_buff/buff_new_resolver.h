//
// Created by tdt on 2021/4/10.
//

#ifndef TDTVISION_RM2021_BUFF_NEW_RESOLVER_H
#define TDTVISION_RM2021_BUFF_NEW_RESOLVER_H
#include "buff_detector.h"
#include "tdtcamera.h"
#include "tdtcommon.h"

namespace tdtnewbuff{


    class NEWBuffResolver {

    public:

        /**
        * @brief    构造函数
        **/
        explicit NEWBuffResolver(tdtcamera::Camera *camera);


        /**
         * @brief   大神符目标解算函数对外接口
         * @param   buff_armor  buff_armor,size为5,第0个是流水的,后边依次是其他的四个大神符装甲板,按顺时针顺序
         *          用多个装甲板去pnp结算可以大大提高结果精准度
         * @return  BuffTarget  输出BuffTarget类的目标
         **/

        tdttoolkit::Buff NEWBuffResolve( std::vector<tdttoolkit::BuffArmor> &buff_armor,
                                      tdttoolkit::ReceiveMessage &receive_message );

    private:

        /**
          * @brief  直接得到物体在世界坐标系下的位姿
          * @param  &rvec  相机坐标系到物体坐标系的旋转向量 将其更正为 世界坐标系到物体坐标系的旋转向量
          * @param  &tvec  相机坐标系到物体坐标系的平移向量 将其更正为 世界坐标系到物体坐标系的平移向量
          * @param  shootPlatform  云台
          **/
        void UnifyCoordinate( cv::Mat &rvec,
                              cv::Mat &tvec,
                              tdttoolkit::ShootPlatform shootPlatform,
                              bool to_world = true );
        /**
          * @brief  求得能量机关中心R的旋转和平移矩阵
          * @param  &buff_armors  装甲板
          * @param  &receive_message 当前云台位姿
          **/
        void GetPrimaryParam(tdttoolkit::BuffArmor &buff_armor,tdttoolkit::ReceiveMessage &receive_message);
        /**
         * @brief  通过反映射纠正欧拉角，得到世界相对相机的平移和旋转矩阵
         * @param  &buff_armors  装甲板
         * @param  &receive_message 当前云台位姿
         **/
        void Angle_Correct(tdttoolkit::BuffArmor &buff_armor,tdttoolkit::ReceiveMessage &receive_message);

        /**
         * @brief  通过反映射纠正距离，得到世界相对物体的平移和旋转矩阵
         * @param  &buff_armor  装甲板
         * @param  &receive_message 当前云台位姿
         **/
        void Distance_Correct(tdttoolkit::BuffArmor &buff_armor,tdttoolkit::ReceiveMessage &receive_message);
        cv::Mat camera_matrix_;
        cv::Mat distortion_matrix_;
        cv::Mat rvec_object_to_world_;
        cv::Mat tvec_object_to_world_;
        float pix_grade_=100;
    };

}

#endif // TDTVISION_RM2021_BUFF_NEW_RESOLVER_H
