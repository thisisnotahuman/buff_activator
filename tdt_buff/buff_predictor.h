

#ifndef TDTVision_RM2021_ENERGY_PREDICT_H
#define TDTVision_RM2021_ENERGY_PREDICT_H

#include"tdtcommon.h"
#include "Debug.h"
#include <fstream>
namespace tdtbuff {

    /**
     * @class   BuffPredcitor
     * @brief
     */
    class BuffPredictor {

    public:
        BuffPredictor();
        /**
         * @brief    计算要输出的信息(包括重力补偿 角度结算 开火控制)
         * @param    shootplatform  云台
         * @param    message 对message成员变量进行赋值
         * @return   void
         */
        void BuffPredict(tdttoolkit::ReceiveMessage &receive_message,tdtusart::Send_Struct_t &send_message);



        /**
         * @name    SetNewTarget
         * @brief   如果流程式输入，则使用下面这个部分
         */
        void SetNewTarget(tdttoolkit::Buff &target);
        /**
         * @name    BuffCommandBuild
         * @brief
         */

        void BuffCommandBuild(
                               bool buff_type,tdttoolkit::ReceiveMessage recevice_message,tdtusart::Send_Struct_t &send_message );

    private:

        /**
         * @name    Predict
         * @brief   通过        void JudgeIsFinal();
预测时间计算大概击打位置,推算就出diff_pitch,得到settime(提前静态瞄准时间)
         */
        void Predict(tdttoolkit::ReceiveMessage &receive_message);

        /**
         * @name    JudgeTypeAndDirection
         * @brief   判断能量机关类型和方向
         *          大符和小符，顺时针和逆时针的判断
         *          这个等待大符速度的范围下来后，是要进行更改的，需要考虑数量和转速时间，以考虑是否转过一圈
         */
        void JudgeType();
        void JudgeDirection();
        /**
         * @name    LittlePredictTime2Point
         * @brief   小能量机关的预测和方向
         */
        cv::Point3f LittlePredictTime2Point(float &predict_time,tdttoolkit::ReceiveMessage &reciveMessage,bool & is_final);

        //大能量机关的预测
        cv::Point3f BigPredictTime2Point(float &predict_time,tdttoolkit::ReceiveMessage &receive_message,bool & is_final);


        /**
         *
         * @name 开火判断
         * @param world_point
         * @param predict_angle
         * @param buff_type
         * @param recevice_message
         */

        void DetaConvert( const double angle,
                          const double deta_x,
                          const double deta_y,
                          double &deta_radial,
                          double &deta_tangent );

        std::vector<cv::Point2f> angle_pre;
        std::vector<tdttoolkit::Buff> history_targets_;     //储存历史的BuffTarget;
        std::vector<float> buff_speeds_;
        std::vector<float> buff_angle_speed;
        std::vector<float> A_max_vector;
        std::vector<float> A_min_vector;

        //预测点
        int inter=0;
        cv::Point3f predict_point_;
        float predict_time_ = 600;
        float t2 = 0;
        float my_pitch = 0;
        float my_yaw = 0;
        int contain = 0;
        bool start = false;
        int final = 0 ;
        int is_final_ = 0;
        float predict_angle_=0;
        int interval_ = 0;          //间隔，为计算速度时，间隔数量
        bool is_first_ = true;
        float last_beat_time_;
        float big_buff_time_;
        bool is_pass_ = false;
        bool clockwise_ = true; //true顺时针 fasle逆时针
        bool buff_type_ = false; //true大符,false小符
        tdttoolkit::KalmanFilter KF;



    };

}


#endif //TDTVision_RM2021_ENERGY_PREDICT_H