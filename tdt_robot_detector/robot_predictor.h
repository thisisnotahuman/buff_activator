#ifndef TDTVision_RM2021_ROBOT_PREDICTOR_H
#define TDTVision_RM2021_ROBOT_PREDICTOR_H

#ifdef VIDEO_DEBUG
#include "Debug.h"
#endif
#include "ModelGenerator.h"
#include "armor_resolver.h"
#include "tdtcommon.h"
#include <fstream>
namespace tdtrobot {
    class RobotRadius {
        // 本类保存轴长数据, 采用类桶排序算法, 精度为int. 初始化需要半径的上下限.
        // 每测得一个半径数据应当Insert一次, GetRaidus返回被Insert次数最多的半径.
    public:
        RobotRadius() {
            min_ = 15, max_ = 35, cnt_ = 0;
            for (int i = 0; i < 30; i++)
                bucket_[i] = 0;
        }
        RobotRadius(int min, int max) {
            min_ = min, max_ = max, cnt_ = 0;
            for (int i = 0; i < 30; i++)
                bucket_[i] = 0;
        }
        void Insert(double r) {
            if (r < min_ || max_ < r)
                return;
            bucket_[(int)round(r) - min_]++;
            cnt_++;
        }
        int GetRadius() {
            int tot = 0;
            for (int i = 0; i < 30; i++, tot += bucket_[i])
                if (tot + bucket_[i] >= cnt_ / 2)
                    return i + min_;
        }
        int GetCnt() { return cnt_; }

    private:
        // 长短轴半径r均应在[min_, max_]之间. 桶排序数组bucket_大小不得小于max_ - min_
        int min_, max_, bucket_[30], cnt_;
    };

    class RobotPredictor {
    public:
        /**************************************************************************
         * @name            RobotPredictor
         * @brief           初始化类
         * @author          王声溢, 1421503785@qq.com
         ***************************************************************************/
        explicit RobotPredictor();

        /**************************************************************************
         * @name            Init
         * @brief           用传入的robot初始化类
         * @author          王声溢, 1421503785@qq.com
         ***************************************************************************/
        void Init(tdttoolkit::Robot robot);

        /**************************************************************************
         * @name            Update
         * @brief           更新当前帧装甲板信息.
         * @author          王声溢, 1421503785@qq.com
         *
         * @param           [in] resolved_armors 解算后的装甲板
         * @param           [in] time_stamp 时间戳, 旋转中心
         * @note            判断是否是新目标. 如果目标连续, 就把封装好的目标放入history_后面.
         ***************************************************************************/
        void Update(std::vector<tdttoolkit::ResolvedArmor> &resolved_armors, uint64_t const &time_stamp);

    private:
        /**************************************************************************
         * @name            CalcArmorsTag
         * @brief           为每个装甲板分配唯一tag(0,1,2,3). 保持tag的旋转不变性.
         * @author          王声溢, 1421503785@qq.com
         *
         * @note           经过本函数计算后,可使用如Robot.GetResolvedArmors()[0].GetTag()的方法获取装甲板tag信息.
         *
         * @note            本函数计算history_.back()中可见的装甲板tag值. 尝试将现存装甲板与上一帧中存在的装甲板连接并继承tag, 然后给新出现的装甲板分配编号
         ***************************************************************************/
        void CalcArmorsTag();

        /**************************************************************************
         * @name            MeasureAxisYaw
         * @brief           对双装甲板和单装甲板应用不同方法计算旋转中心位置
         * @author          王声溢, 1421503785@qq.com
         *
         * @note            [out] Point3f类型的旋转中心,结果保存在axis_point这个vector中.
         * @note            本函数只相信双灯条解算的装甲板.
         * @note            有两个装甲板时计算两装甲板法向量最近点得到轴点x, z坐标. y坐标使用两装甲板y轴平均值.
         * @note            车的轴长信息来源于本函数在双装甲板时测得的长短轴半径. 半径信息存入robotRadius[RobotType][长/短轴]中
         * @note            单装甲板时轴心根据长短轴半径估测. 没有半径数据时不可计算. 没有tag_to_radius_index_信息时使用平均半径
         ***************************************************************************/
        void MeasureAxisYaw(const tdttoolkit::ReceiveMessage &receiveMessage);

        /**************************************************************************
         * @name            CalcVelocity
         * @brief           计算装甲板在视野中出现时间内的平均角速度.
         * @author          王声溢, 1421503785@qq.com
         *
         *
         * @return           返回平均角速度，速度单位为 角度/秒. 顺时针为正, 逆时针为负.
         *
         * @note            只可在装甲板刚消失时调用, 以保证获得的是完整1/4圈的平均速度.
         * @note            调用时销毁未出现装甲板的数据. 每一帧必须调用一次. 调用位置在CalcArmorsTag之后.
         ***************************************************************************/
        double CalcVelocity();

        /**************************************************************************
         * @name            GetStatus
         * @brief           获取当前机器人运动状态
         * @author          严俊涵，王声溢1421503785@qq.com
         *
         * @return          0: 无法判断 1: 平移 2: 小陀螺 3: 平移+小陀螺 4: 摇摆 5: 平移+摇摆 >5: error
         ***************************************************************************/
        int GetStatus();

        /**************************************************************************
         * @name            AxisLinearRegression
         * @brief           获取回归直线方程预测的轴心点
         * @author          严俊涵, 359360997@qq.com
         *
         * @return          预测值的坐标点
         ***************************************************************************/
        cv::Point3f AxisLinearRegression();

        cv::Point3f ShockAbsorber();

        cv::Point3f ShockAbsorber_liner();

        /**************************************************************************
         * @name            Cmp
         * @brief           比较2个Point.x大小
         * @author          黄海
         * @param           [in] 2个Point
         * @note            如果cmp返回结果为假, 那么sort函数就会将他们互换位置；
         * @note            如果cmp返回结果为真，就会保持原来位置不变。
         ***************************************************************************/
        static bool Cmp(cv::Point2f a, cv::Point2f b);

        /**************************************************************************
         * @name            PolyFit
         * @brief           通过矩阵运算来获取拟合结果
         * @author          黄海
         * @param           [in] n 所选拟合方程的阶数
         * @note            不需要了解函数实现，用就完事了
         ***************************************************************************/
        cv::Mat PolyFit(int n);

        /**************************************************************************
         * @name            RegressionEnd
         * @return          返回回归得到曲线的最后点
         * @author          黄海
         ***************************************************************************/
        cv::Point3f RegressionEnd(cv::Mat M, int n);

        /**************************************************************************
         * @name            CalcOnTimeVelocity
         * @author          严俊涵
         *
         * @return          返回实时旋转速度
         * @note            本函数实时更新数值，区别于CalcVelocity在装甲板消失时更新数值
         * @note            本函数用于摇摆运算，区别于CalcVelocity用于小陀螺判断
         ***************************************************************************/
        double CalcOnTimeVelocity();

        /**************************************************************************
         * @name            armorDecision
         * @brief           击打装甲板决策
         * @author          严俊涵
         *
         * @param           [in] resolved_armors_ 传入需要决策的装甲板
         * @return          返回决策的装甲板id
         ***************************************************************************/
        int armorDecision(std::vector<tdttoolkit::ResolvedArmor> resolved_armors_);

        /**************************************************************************
         * @name            DistanceCompensation
         * @brief           对轴心到装甲板的距离进行补偿
         * @author          严俊涵
         *
         * @param           [in] axis_point_ 需要补偿的轴心
         ***************************************************************************/
        void DistanceCompensation(cv::Point3f &axis_point);

        /**************************************************************************
         * @name            AxisFilter
         * @brief           对轴心进行速度滤波并预测
         * @author          严俊涵
         *
         * @param           [in] axis_point 轴心
         * @param           [in] bulletspeed 弹速
         * @return          速度滤波并预测后轴心
         ***************************************************************************/
        cv::Point3f AxisFilter(cv::Point3f axis_point, float bulletspeed);

        /**************************************************************************
         * @name            GetStatusPlus
         * @brief           获取当前机器人在小陀螺以及摇摆时的具体情况
         * @author          严俊涵
         *
         * @param           [in] query 查询id 默认0返回当前状态，1返回摇摆中心点,2返回摇摆过程中出现离轴心最近的装甲板tag
         * @return          在查询id为0时返回：2.0 普通小陀螺,2.1 平移小陀螺,2.2 偏心小陀螺,4.0 未知摇摆,4.1 单装甲板小幅度摇摆,4.2 中心不在装甲板上的1->2->1摇摆，4.3 2->1->2->3->2摇摆,4.4 转动幅度过大的摇摆
         * @note            小陀螺部分暂未完成
         * @note            查询id=1:若无中心装甲板返回-1,返回中心装甲板时会+0.1f避免浮点数精度误差
         * @note            查询id=2:若无结果返回-1,返回装甲板tag时会+0.1f避免浮点数精度误差
         ***************************************************************************/
        float GetStatusPlus(int query = 0);

        /**************************************************************************
         * @name            FollowModeShoot
         * @brief           跟随模式射击
         *
         * @param           [in] receiveMessage 下位机传上来的信息
         * @param           [in] sendStruct 传给下位机的信息
         ***************************************************************************/
        void FollowModeShoot(const tdttoolkit::ReceiveMessage &receiveMessage, tdtusart::Send_Struct_t &sendStruct);

        /**************************************************************************
         * @name            GyroModeShoot
         * @brief           小陀螺射击
         *
         * @param           [in] receiveMessage 下位机传上来的信息
         * @param           [in] sendStruct 传给下位机的信息
         ***************************************************************************/
        void GyroModeShoot(const tdttoolkit::ReceiveMessage &receiveMessage, tdtusart::Send_Struct_t &sendStruct);

        /**************************************************************************
         * @name            SwingModeShoot
         * @brief           摇摆射击
         *
         * @param           [in] receiveMessage 下位机传上来的信息
         * @param           [in] sendStruct 传给下位机的信息
         ***************************************************************************/
        void SwingModeShoot(const tdttoolkit::ReceiveMessage &receiveMessage, tdtusart::Send_Struct_t &sendStruct);

        /**************************************************************************
         * @name            SwingModeShoot2
         * @brief           摇摆射击
         *
         * @param           [in] receiveMessage 下位机传上来的信息
         * @param           [in] sendStruct 传给下位机的信息
         ***************************************************************************/
        void SwingModeShoot2(const tdttoolkit::ReceiveMessage &receiveMessage, tdtusart::Send_Struct_t &sendStruct);

        /**************************************************************************
         * @name            GetKalmanProcessNoise
         * @brief           由帧率获取卡尔曼滤波过程噪声
         * @author          黄海
         *
         * @param           [in] orginNoise 帧数换算之前的视频DEBUG过程噪声
         * @return          卡尔曼滤波过程噪声
         * @note            90.738 = 0.05*42.6*42.6 (42.6是红色多目标.avi的平均帧率，0.05是之前一直使用的玄学参数）
         ***************************************************************************/
        double GetKalmanProcessNoise(float orginNoise = 0.02);

        /**************************************************************************
         * @name            GyroFireTime
         * @brief           计算小陀螺模式下每次可开火时间
         * @author          黄海
         *
         * @param           d   机器人装甲板长度
         * @param           R   机器人长短轴平均值
         * @param           w   机器人角速度
         * @return          t 可开火时间
         * @note            t = 2*arctan(d/2R)/w*0.8(实际情况与建模有差异，所以乘以0.8,可根据上车情况调整0.8）
         * @note            看不懂为什么这么算找黄海
         ***************************************************************************/
        double GyroFireTime();

        /**************************************************************************
         * @name            SpecialMode
         * @brief           用于中期视频等特殊情况
         * @author          严俊涵
         *
         * @param           [in] receiveMessage 下位机传上来的信息
         * @param           [in] sendStruct 传给下位机的信息
         ***************************************************************************/
        void SpecialMode(const tdttoolkit::ReceiveMessage &receiveMessage, tdtusart::Send_Struct_t &sendStruct);

    public:
        void FireCommand(const tdttoolkit::ReceiveMessage &receiveMessage, tdtusart::Send_Struct_t &sendStruct);

    private:
        std::vector<cv::Point2f>                    in;                          //减震时把需要帧的轴心传入
        int                                         tag_to_radius_index_[4];     // tag对应轴的长度. 0: 长轴, 1: 短轴.
        std::vector<cv::Point3f>                    armor_pos_history_[4];       // 以tag分类. 装甲板中心在世界坐标系下的位置.
        std::vector<double>                         armor_angle_history_[4];     // 以tag分类. 装甲板法向量相对世界坐标系z轴的角度. 丢弃y轴数据.
        bool                                        isTaged[4];                  // 以tag分类.当前Tag是否与前一帧连上
        std::vector<double>                         armor_timeStamp_history_[4]; //装甲板的历史存在时间
        std::vector<double>                         shock_timeStamp_history_;    //每次摇摆用时的历史时间
        std::vector<std::pair<cv::Point3f, double>> axis_point_;                 // 轴点在世界坐标系下的位置, 时间戳
        std::vector<tdttoolkit::Robot>              history_;                    // 当前robot的历史信息记录
        std::vector<int>                            tag_change_history_;         // 新出现的装甲板相对于旧装甲的tag_diff被保存下来.
        int                                         rotation_duration_;
        double                                      yawLinearSpeed_;       // Robot可见装甲板在世界系中的yaw轴线速度，单位：米/秒
        double                                      armorAngles_[4];       // 以tag分类，通过当前装甲板与世界系z轴的夹角预估下一个装甲板的夹角
        tdttoolkit::Polar3f                         currentPolar_[4];      // 以tag分类，装甲板当前的极坐标
        tdttoolkit::Polar3f                         predictPolar_[4];      // 以tag分类，装甲板预测点的极坐标
        cv::Point3f                                 currentPoint_[4];      // 以tag分类，装甲板当前的世界坐标
        cv::Point3f                                 predictPoint_[4];      // 以tag分类，装甲板预测点的世界坐标
        tdttoolkit::KalmanFilter                    axisFilter_[3];        // 轴点的三维坐标速度卡尔曼滤波器
        tdttoolkit::KalmanFilter                    yawLinearFilter_[4];   // 四个装甲板在世界系中yaw的瞬时线速度
        tdttoolkit::KalmanFilter                    pitchLinearFilter_[4]; // 四个装甲板在世界系中pitch的瞬时线速度
        int                                         imm_num = 0;           // 0的时候初始化，>0时更新状态
        tdtimm::ModelGenerator                      armor_model[4];        // 四个装甲板分别的分布式模型
        tdtimm::ModelGenerator                      axis_model;            // 轴心的分布式模型
        Eigen::Matrix<double, 1, 6>                 state_vector      = Eigen::Matrix<double, 1, 6>::Zero(6);
        Eigen::Vector4d                             update_vector     = Eigen::Vector4d::Zero(4); // 更新IMM用的状态向量
        double                                      velocity_time_out = 2;                        //判断小陀螺的装甲板超时时间
        bool                                        gyroCommand       = 0;

    public:
        bool                     isFire_;       //这一帧是否开火
        tdtrobot::ArmorResolver *armorResolver; //用于开火决策中决策点的重投影，main中赋值后可以忽略
        double                   kalman_fps;    //用来算跟随模式下卡尔曼滤波过程噪声　声明为public是因为需要在main.cpp里获取帧率
        double                   fire_delay_;   //相机拍摄时间到发射子弹的时间差,单位：s
    };

} // namespace tdtrobot

#endif // TDTVision_RM2021_ROBOT_PREDICTOR_H
