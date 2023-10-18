#ifndef TDTVision_RM2021_ARMOR_RESOLVER_H
#define TDTVision_RM2021_ARMOR_RESOLVER_H

#include "tdtcamera.h"
#include "tdtcommon.h"

namespace tdtrobot {

    class ArmorResolver {

    public:
        /**
         * @name    构造函数
         * @brief   读取相机内参
         */
        ArmorResolver();

        /**
         * @name    Resolve
         * @brief   对传入的每个RobotArmor进行PNP解算并统一到世界坐标系中描述坐标和角度, 返回封装的ResolvedArmor集合
         * @param   armors 识别部分获取的RobotArmor集合
         * @param   shoot_platform 云台当前状态
         * @return  vector<ResolvedArmor> 解算完毕重新封装的armors
         */
        std::vector<tdttoolkit::ResolvedArmor> Resolve(std ::vector<tdttoolkit ::RobotArmor> &armors, tdttoolkit ::ShootPlatform shoot_platform);

        // tdttoolkit::ResolvedArmor PreciseResolve(tdttoolkit::ResolvedArmor &resolvedArmor,
        //                             tdttoolkit :: ShootPlatform shoot_platform,
        //                             tdttoolkit::ResolvedArmor &last_resolved_armor,
        //                             cv::Mat src,cv::Mat &last_src);

        /**
         * @brief  对解算点进行投影
         * @return  图像上的点
         */
        cv::Point2d ProjectArmor(const tdttoolkit::ShootPlatform &shootPlatform, const tdttoolkit::ResolvedArmor &resolvedArmor, cv::Point3f worldPoint = cv::Point3f(0, 0, 0));

        /**
         * @name    WorldToPixel
         * @brief   世界坐标系重投影到像素坐标系
         * @author  严俊涵
         *
         * @param   [in] shootPlatform 包含yaw与pitch的射击平台数据
         * @param   [in] worldPoint 需要重投影的点
         * @return  重投影结果
         */
        cv::Point2d WorldToPixel(const tdttoolkit::ShootPlatform &shootPlatform, cv::Point3f worldPoint = cv::Point3f(0, 0, 0));

        /**
         * @name    GetFOV
         * @brief   获得相机FOV视场角
         * @author  严俊涵
         *
         * @return  x与y上的视场角
         * @note    [important] 本函数利用了标定参数，并假设相机为MV-CA016-10UC,即假设图像分辨率为1440*1080
         */
        std::pair<float, float> GetFOV();

    private:
        /**
         * @name    UnifyCoordinate
         * @brief   将坐标描述以及角度描述统一到世界坐标系下
         * @param   pnp_rvec PNP解算获得的旋转向量
         * @param   pnp_tvec PNP解算获得的平移向量
         * @param   shootPlatform 云台当前状态
         * @param   TVecObjInWorld 本函数得出的平移向量
         * @param   RVecObjToWorld 本函数得出的旋转向量
         * @param   euler_world_to_object 世界系->物体系  Z-X-Y欧拉角
         * @param   polar_in_world 世界系->物体系  Z-X-Y欧拉角
         *
         * @齐次变换           P(物体系原点|世界系) = R(相机系 -> 世界系) * P(物体系原点|相机系) + P(相机系原点|世界系)
         * @旋转矩阵的转换关系   R(物体系 -> 相机系) * R(相机系 ->　世界系) = R(物体系 -> 世界系)
         * @注释              P(p|B): B为上角标, 表示点p在坐标系B中的坐标
         *                   R(A -> B): A为下角标, B为上角标, 表示从坐标系A的姿态旋转至坐标系B的姿态的旋转矩阵
         */
        void UnifyCoordinate(tdttoolkit ::RobotType robot_type, cv ::Mat &pnp_rvec, cv ::Mat &pnp_tvec, tdttoolkit ::ShootPlatform shootPlatform, cv::Mat TVecObjInWorld, cv::Mat RVecObjToWorld, tdttoolkit ::Polar3f &polar_in_world, cv::Vec3f &euler_world_to_object);

        tdttoolkit ::RobotType robot_type_;
        cv::Mat                camera_matrix_;
        cv::Mat                dist_coeffs_;

        tdttoolkit ::Polar3f polar_in_camera_; //物体系原点在相机系中的极坐标
        cv::Mat              pnp_rvec_;
        cv::Mat              pnp_tvec_;
        // cv::Mat                RVecObjToWorld_;                                                                         //物体系到世界系的旋转向量
        // cv::Mat                TVecObjInWorld_;                                                                         //物体系原点在世界系中的三维坐标  P(物体系原点|世界系)
        cv::Vec3f TvecCameraInWorld_ = tdttoolkit::MatToPoint3f((cv::Mat_<float>(3, 1) << 1, -3.93, 11.9)); //世界坐标系在相机坐标系下坐标取反   /相机到转轴的位置补偿,不同车不一样,理解为世界坐标系原点在相机坐标系下的位置,并方向取反
    };

} // namespace tdtrobot

#endif // TDTVision_RM2021_ARMOR_RESOLVER_H
