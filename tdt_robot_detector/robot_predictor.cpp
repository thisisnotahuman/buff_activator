#ifdef O3ENABLE
#pragma GCC optimize(3, "Ofast", "inline")
#endif
// #define AXIS_DEBUG
#include "robot_predictor.h"

// #define SPECIALMODE
// #define PREDICT_DEBUG_OUTPUT

using namespace std;
using namespace cv;
using namespace tdttoolkit;

namespace tdtrobot {
    RobotRadius robotRadius[6][2]; // 存储场上5个有轴心的机器人半径数据. [0]无作用.

#ifdef AXIS_DEBUG
    ofstream fout("./axispoint918-1.txt", ios::out);  //输出预测点
    ofstream fout1("./axispoint918-2.txt", ios::out); //输出测量点
    float    dis_real_pred = 0;
#endif

    RobotPredictor::RobotPredictor() {
        std::vector<int>().swap(tag_change_history_);
        std::vector<std::pair<cv::Point3f, double>>().swap(axis_point_);
        std::vector<tdttoolkit::Robot>().swap(history_);
        state_vector    = Eigen::Matrix<double, 1, 6>::Zero(6);
        state_vector(4) = 10; // a = 10m/s^2
        state_vector(5) = 10;
        isFire_         = 0;
        for (int i = 0; i < 4; i++) {
            tag_to_radius_index_[i] = -1;
            std::vector<double>().swap(armor_angle_history_[i]);
            std::vector<double>().swap(armor_timeStamp_history_[i]);
            std::vector<cv::Point3f>().swap(armor_pos_history_[i]);
            armor_model[i].imm_model = armor_model[i].generateIMMModel(Time::GetTimeNow() / 1000000.0f, state_vector); ///
            // std::cout << "robot_predict.cpp 32" << std::endl;
        }
        // axis_model.imm_model = axis_model.generateIMMModel(Time::GetTimeNow() / 1000000.0f, state_vector); ///

        for (int i = 0; i < 3; ++i) {
            axisFilter_[i] = tdttoolkit::KalmanFilter(0, true);
            axisFilter_[i].SetQ(0.1);
            axisFilter_[i].SetR(0.1);
            axisFilter_[i].Estimate(0, 0);
        }

        for (int i = 0; i < 4; ++i) {
            yawLinearFilter_[i]   = tdttoolkit::KalmanFilter(100, true);
            pitchLinearFilter_[i] = tdttoolkit::KalmanFilter(0, true);
            yawLinearFilter_[i].SetQ(0.1);
            yawLinearFilter_[i].SetR(0.1);
            pitchLinearFilter_[i].SetQ(0.1);
            pitchLinearFilter_[i].SetR(0.1);
            predictPolar_[i] = Polar3f(-1, -1, -1);
            currentPolar_[i] = Polar3f(-1, -1, -1);
            yawLinearFilter_[i].Estimate(0, 0);
            pitchLinearFilter_[i].Estimate(0, 0);
        }

        // ! debug
        robotRadius[4][0].Insert(25);
        robotRadius[4][1].Insert(20);
        // ! debug
    }

    void RobotPredictor::Init(tdttoolkit::Robot robot) {
        RobotPredictor();
        for (int i = 0; i < robot.GetResolvedArmors().size(); i++)
            robot.SetArmorTag(i, i);
        history_.emplace_back(robot);
    }

    void RobotPredictor::Update(vector<ResolvedArmor> &resolved_armors, uint64_t const &time_stamp) {

        RobotType robot_type = resolved_armors[0].GetRobotType();
        Robot     robot(resolved_armors, robot_type, time_stamp); // 封装当前帧robot
        if (history_.empty())
            Init(robot);
        else {
            // 类型相同, 时间差小于100ms
            bool robot_consistency = (history_.back().GetRobotType() == robot.GetRobotType());
            bool time_consistency  = ((robot.GetTimeStamp() - history_.back().GetTimeStamp()) < 100000); // 100ms以内
            // ! Warning: debug: time_consistency off
            time_consistency = true;
            // ! Warning: debug: time_consistency off

            if (robot_consistency && time_consistency) {
                history_.push_back(robot);
            } else
                Init(robot);

            if (history_.size() > 50)
                history_.erase(history_.begin());
        }
    }

    void RobotPredictor::CalcArmorsTag() {

        int tag_difference_to_last = 0;
        if (history_.size() < 2) {
        } else {
            tdttoolkit::Robot &currentRobot = *(history_.end() - 1);
            tdttoolkit::Robot &lastRobot    = *(history_.end() - 2);
            // 当前帧最后一个被连接(识别到与上一帧某一装甲板有关联)的装甲板的下标 以及连接计数.
            int linked_armor_index = -1, linkcnt = 0;

            // 遍历上一帧中的armors
            for (int last_index = 0; last_index < lastRobot.GetResolvedArmors().size(); ++last_index) {
                // 遍历当前帧中的armors
                for (int current_index = 0; current_index < currentRobot.GetResolvedArmors().size(); ++current_index) {
                    // 判断为同一个armor：距离不远
                    const CustomRect &last_armor_rect    = lastRobot.GetResolvedArmors()[last_index].GetStickerRect();
                    const CustomRect &current_armor_rect = currentRobot.GetResolvedArmors()[current_index].GetStickerRect();
                    float             center_distance    = CalcDistance(last_armor_rect.GetCenter(), current_armor_rect.GetCenter());
                    // std::cout<<"tag debug:"<<", "<<center_distance<<", "<<float(last_armor_rect.GetRect().height)<<std::endl;
                    if (center_distance < float(last_armor_rect.GetRect().height)) {
                        // 记录连接情况
                        linked_armor_index = current_index;
                        linkcnt++;
                        // 连接成功,即时继承连接上的装甲板的tag
                        currentRobot.SetArmorTag(current_index, lastRobot.GetResolvedArmors()[last_index].GetTag());
                        // cout << "link success: armor " << current_index << " linked to tag " << lastRobot.GetResolvedArmors()[last_index].GetTag() << endl;
                    }
                }
            }

            if (linked_armor_index != -1) {
                if (currentRobot.GetResolvedArmors().size() > 1 && linkcnt < 2) {
                    // 进入这段代码的只有一种情况:当前帧检测到了两个装甲板,且只有一个被判定为新装甲板.
                    // 由于这种情况下装甲板下标只有0和1,所以新装甲板编号一定是已被连接的装甲板的隔壁
                    int new_armor_index = !linked_armor_index;
                    // cout << "new_armor_index:" << new_armor_index << endl;
                    const CustomRect &last_armor_rect = lastRobot.GetResolvedArmors()[0].GetStickerRect();
                    // new armor
                    const CustomRect &current_armor_rect = currentRobot.GetResolvedArmors()[new_armor_index].GetStickerRect();
                    // 目标逆时针旋转,设置-1,顺时针旋转设置1
                    tag_difference_to_last = last_armor_rect.GetCenter().x < current_armor_rect.GetCenter().x ? 1 : -1;
                    currentRobot.SetArmorTag(new_armor_index, currentRobot.GetResolvedArmors()[linked_armor_index].GetTag() + tag_difference_to_last);

                    if (tag_change_history_.size() >= 5)
                        tag_change_history_.erase(tag_change_history_.begin());
                    tag_change_history_.emplace_back(tag_difference_to_last);
                }
            } else {
                // 从左到右重新分配装甲板编号
                Init(currentRobot);
            }
        }

        // 更新armor_pos_history_
        tdttoolkit::Robot &currentRobot = history_.back();
        for (int i = 0; i < 4; ++i)
            currentRobot.SetArmorStatus(i, false);

        for (auto &armor : currentRobot.GetResolvedArmors())
            if (armor.GetTag() != -1) {
                currentRobot.SetArmorStatus(armor.GetTag(), true);
                armor_pos_history_[armor.GetTag()].emplace_back(MatToPoint3f(armor.GetPositionInWorld()));
                armor_timeStamp_history_[armor.GetTag()].emplace_back(currentRobot.GetTimeStamp());
            }
    }

    void RobotPredictor::MeasureAxisYaw(const ReceiveMessage &receiveMessage) {

        bool         isBelievable[2] = {false, false};
        Robot const &robot           = history_.back();
        static float axis_y          = 2.3333333f; //赋值用于查看axis_y是否被更新
        // 装甲板数量异常
        if (robot.GetResolvedArmors().empty() || robot.GetResolvedArmors().size() > 2)
            goto exitMeasureAxisYaw;

        for (int i = 0; i < robot.GetResolvedArmors().size(); i++)
            // 必须是双灯条此装甲板解算结果才可信
            isBelievable[i] = robot.GetResolvedArmors()[i].GetBelievable() == 1;

        // 没有可信装甲板 或者 (只有一块装甲板 同时 该编号的机器人没有半径数据)
        if ((!isBelievable[0] && !isBelievable[1]) || ((isBelievable[0] xor isBelievable[1]) && robotRadius[history_.back().GetRobotType()][0].GetCnt() == 0)) {
        exitMeasureAxisYaw:
            if (!axis_point_.empty()) {
                // float x                  = axisFilter_[0].Estimate(axis_point_.back().first.x, false);
                // float y                  = axisFilter_[1].Estimate(axis_point_.back().first.y, false);
                // float z                  = axisFilter_[2].Estimate(axis_point_.back().first.z, false);
                // axis_point_.back().first = {x, y, z};
            }
            return; // 没有一块装甲板可信
        }

        // 有两块可信装甲板
        if (isBelievable[0] && isBelievable[1]) {
#ifdef PREDICT_DEBUG_OUTPUT
            std::cout << "-------------Measure TEST1-------------\n";
#endif

            cv::Mat                    tVec1     = robot.GetResolvedArmors()[0].GetTVecInWorld();
            cv::Mat                    rVec1     = robot.GetResolvedArmors()[0].GetRVecToWorld();
            cv::Mat                    tVec2     = robot.GetResolvedArmors()[1].GetTVecInWorld();
            cv::Mat                    rVec2     = robot.GetResolvedArmors()[1].GetRVecToWorld();
            std::pair<Point3f, double> axisPoint = make_pair(Point3f(), armor_timeStamp_history_[robot.GetResolvedArmors()[0].GetTag()].back());
            double                     angle, distance;
            // auto GetTheClosestPointOfTwoLines =
            //     [&angle, &distance](Mat tVec1, Mat rVec1, Mat tVec2, Mat rVec2, Point3f &axisPoint) {

            Mat rMat1, rMat2, z = (Mat_<float>(3, 1) << 0, 0, 1);
            Rodrigues(rVec1, rMat1);
            Rodrigues(rVec2, rMat2);

            Point3f a[2], b[2];
            a[0] = MatToPoint3f(tVec1);
            a[1] = MatToPoint3f(rMat1 * z + tVec1);
            b[0] = MatToPoint3f(tVec2);
            b[1] = MatToPoint3f(rMat2 * z + tVec2);

            Point3f d1          = a[1] - a[0];
            Point3f d2          = b[1] - b[0];
            Point3f e           = b[0] - a[0];
            Point3f cross_d1_d2 = d1.cross(d2);
            float   t1          = e.cross(d2).dot(cross_d1_d2);
            float   t2          = e.cross(d1).dot(cross_d1_d2);
            float   dd_2        = pow(norm(cross_d1_d2), 2);

            //两直线上的最近点
            Point3f PonA      = a[0] + (a[1] - a[0]) * t1 / dd_2;
            Point3f PonB      = b[0] + (b[1] - b[0]) * t2 / dd_2;
            axisPoint.first   = (PonA + PonB) / 2;
            axisPoint.first.y = 0;

            d1 = DropYAxis(d1), d2 = DropYAxis(d2);
            //两直线之间角度
            angle = 45.0 / atan(1.0) * acos(d1.dot(d2) / (norm(d1) * norm(d2)));
            //两直线之间最近距离
            distance = norm(PonB - PonA);
            // std::cout << "Normal line of two armor - angle: " << angle << ", distance: " << distance << std::endl;

            // };
            // GetTheClosestPointOfTwoLines(tvec1, rvec1, tvec2, rvec2, axisPoint);

            // 计算并更新长短轴长度
            double norm1 = norm(DropYAxis(MatToPoint3f(tVec1)) - axisPoint.first);
            double norm2 = norm(DropYAxis(MatToPoint3f(tVec2)) - axisPoint.first);
            robotRadius[history_.back().GetRobotType()][0].Insert(max(norm1, norm2));
            robotRadius[history_.back().GetRobotType()][1].Insert(min(norm1, norm2));

            // 为装甲板添加对应半径下标
            int tag = robot.GetResolvedArmors()[0].GetTag();
            if (norm1 > norm2) {
                tag_to_radius_index_[tag % 4] = tag_to_radius_index_[(tag + 2) % 4] = 0;
                tag_to_radius_index_[(tag + 1) % 4] = tag_to_radius_index_[(tag + 3) % 4] = 1;
            } else {
                tag_to_radius_index_[tag % 4] = tag_to_radius_index_[(tag + 2) % 4] = 1;
                tag_to_radius_index_[(tag + 1) % 4] = tag_to_radius_index_[(tag + 3) % 4] = 0;
            }

            // 计算装甲板法向量抛弃y轴后与世界坐标系向量(0, 0, 1)的夹角. 在x-z平面上, z轴为0度, 顺时针为正, 逆时针为负.
            armor_angle_history_[robot.GetResolvedArmors()[0].GetTag()].emplace_back((Point3f(0, 0, 1).cross(d1).y > 0 ? 1 : -1) * 45.0 / atan(1.0) * acos(Point3f(0, 0, 1).dot(d1) / (norm(Point3f(0, 0, 1)) * norm(d1))));
            armor_angle_history_[robot.GetResolvedArmors()[1].GetTag()].emplace_back((Point3f(0, 0, 1).cross(d2).y > 0 ? 1 : -1) * 45.0 / atan(1.0) * acos(Point3f(0, 0, 1).dot(d2) / (norm(Point3f(0, 0, 1)) * norm(d2))));

            axisPoint.first.y = (robot.GetResolvedArmors()[0].GetPositionInWorld().at<float>(1) + robot.GetResolvedArmors()[1].GetPositionInWorld().at<float>(1)) / 2;
            axis_point_.emplace_back(axisPoint);
            axis_y = axisPoint.first.y;
            if (axis_point_.size() >= 2 && (GetStatus() == 2 || GetStatus() == 3 || GetStatus() == 4 || GetStatus() == 5))
                axis_point_.back().first = 0.3 * axis_point_.back().first + 0.7 * (*(axis_point_.end() - 2)).first;

            /*  // 卡尔曼滤波: 旋转中心
             for (int i = 0; i < 3; ++i) {
                 axisFilter_[i].SetQ(0.15);
                 axisFilter_[i].SetR(0.2);
             }
             if (!axis_point_.empty()) {
                 float x            = axisFilter_[0].Estimate(axis_point_.back().first.x, true);
                 float y            = axisFilter_[1].Estimate(axis_point_.back().first.y, true);
                 float z            = axisFilter_[2].Estimate(axis_point_.back().first.z, true);
                 axis_point_.back().first = {x, y, z};
             }  */

        } else { // 只有一块装甲板可信
#ifdef PREDICT_DEBUG_OUTPUT
            std::cout << "-------------Measure TEST2-------------\n";
#endif

            double  currentRobotRadius[2] = {(double)robotRadius[history_.back().GetRobotType()][0].GetRadius(), (double)robotRadius[history_.back().GetRobotType()][1].GetRadius()};
            int     index                 = isBelievable[0] ? 0 : 1;
            cv::Mat tVec                  = robot.GetResolvedArmors()[index].GetTVecInWorld();
            cv::Mat rVec                  = robot.GetResolvedArmors()[index].GetRVecToWorld();

            cv::Mat rMat, z = (Mat_<float>(3, 1) << 0, 0, 1);
            Rodrigues(rVec, rMat);
            Point3f armorToCenter = DropYAxis(MatToPoint3f(rMat * z));
            // 获取当前装甲板对应半径信息. 无对应信息则使用长短轴半径平均值
            double radius;
            if (tag_to_radius_index_[robot.GetResolvedArmors()[index].GetTag()] == -1) {
                radius = (currentRobotRadius[0] + currentRobotRadius[1]) / 2;
            } else
                radius = currentRobotRadius[tag_to_radius_index_[robot.GetResolvedArmors()[index].GetTag()]];
            armorToCenter *= radius / norm(armorToCenter);
            std::pair<Point3f, double> axisPoint = make_pair(DropYAxis(armorToCenter + MatToPoint3f(tVec)), armor_timeStamp_history_[robot.GetResolvedArmors()[index].GetTag()].back());

            // 计算装甲板法向量抛弃y轴后与世界坐标系向量(0, 0, 1)的夹角. 在x-z平面上, z轴为0度, 顺时针为正, 逆时针为负.
            armor_angle_history_[robot.GetResolvedArmors()[index].GetTag()].emplace_back((Point3f(0, 0, 1).cross(armorToCenter).y > 0 ? 1 : -1) * 45.0 / atan(1.0) * acos(Point3f(0, 0, 1).dot(armorToCenter) / (norm(Point3f(0, 0, 1)) * norm(armorToCenter))));

            // 使用当前装甲板高度作为轴点高度.
            axisPoint.first.y = robot.GetResolvedArmors()[index].GetPositionInWorld().at<float>(1);
            if (axis_y != 2.3333333f)
                axisPoint.first.y = axis_y;
            axis_point_.emplace_back(axisPoint);

            if (axis_point_.size() >= 2 && (GetStatus() == 2 || GetStatus() == 3 || GetStatus() == 4 || GetStatus() == 5))
                axis_point_.back().first = 0.3 * axis_point_.back().first + 0.7 * (*(axis_point_.end() - 2)).first;

            /* // 卡尔曼滤波: 旋转中心
            for (int i = 0; i < 3; ++i) {
                axisFilter_[i].SetQ(0.2);
                axisFilter_[i].SetR(0.45);
            }
            if (!axis_point_.empty()) {
                float x            = axisFilter_[0].Estimate(axis_point_.back().first.x, true);
                float y            = axisFilter_[1].Estimate(axis_point_.back().first.y, true);
                float z            = axisFilter_[2].Estimate(axis_point_.back().first.z, true);
                axis_point_.back().first = {x, y, z};
            } */
        }

        // // 三轴限幅
        // if(axis_point_.size() > 0)
        // {
        //     const auto& armors = history_.back().GetResolvedArmors();
        //     if(armors.size() == 1){
        //         double xdiffPow = pow((axis_point_.back().first.x - armors[0].GetPositionInWorld().at<float>(0)), 2);
        //         double ydiffPow = pow((axis_point_.back().first.y - armors[0].GetPositionInWorld().at<float>(1)), 2);
        //         double zdiffPow = pow((axis_point_.back().first.z - armors[0].GetPositionInWorld().at<float>(2)), 2);
        //         std::cout << "[限域]单装甲板: " << xdiffPow << ", " << ydiffPow << ", " << zdiffPow << std::endl;
        //         if(xdiffPow > 144 || ydiffPow > 10 || zdiffPow > 400)    // 12 * 12 || 3 * 3 * 2 || 20 * 20
        //             axis_point_.pop_back();
        //     }else if(armors.size() == 2){
        //         double xdiffPow = pow((axis_point_.back().first.x - armors[0].GetPositionInWorld().at<float>(0)), 2) + pow((axis_point_.back().first.x - armors[1].GetPositionInWorld().at<float>(0)), 2);
        //         double ydiffPow = pow((axis_point_.back().first.y - armors[0].GetPositionInWorld().at<float>(1)), 2) + pow((axis_point_.back().first.x - armors[1].GetPositionInWorld().at<float>(1)), 2);
        //         double zdiffPow = pow((axis_point_.back().first.z - armors[0].GetPositionInWorld().at<float>(2)), 2) + pow((axis_point_.back().first.z - armors[1].GetPositionInWorld().at<float>(2)), 2);
        //         std::cout << "[限域]双装甲板: " << xdiffPow << ", " << ydiffPow << ", " << zdiffPow << std::endl;
        //         if(xdiffPow > 722 || ydiffPow > 20 || zdiffPow > 800)  // 19 * 19 * 2 || 3 * 3 * 2 || 20 * 20 * 2
        //             axis_point_.pop_back();
        //     }
        //     if(RectangularToPolar(axis_point_.back().first).distance > 1000)
        //         axis_point_.pop_back();
        // }

        // // xz邻帧限幅
        // if (axis_point_.size() >= 2) {
        //     double xdiff = pow(fabs((*(axis_point_.end() - 1)).first.x) - fabs((*(axis_point_.end() - 2)).first.x), 2);
        //     double zdiff = pow(fabs((*(axis_point_.end() - 1)).first.z) - fabs((*(axis_point_.end() - 2)).first.z), 2);
        //     double diff  = xdiff + zdiff;
        //     if (diff > 109) // zdiff <= 10, xdiff <= 3
        //         axis_point_.back().first = (*(axis_point_.end() - 2)).first;
        // }

        // // x多帧限幅
        // float errorSq = 0;
        // int   max     = (axis_point_.size() < 6 && axis_point_.size() > 0) ? axis_point_.size() : (axis_point_.size() > 6 ? 6 : -1);

        // if (max != -1)
        //     for (int i = 0; i < max; ++i)
        //         errorSq += pow(fabs((*(axis_point_.end() - 1)).first.x) - fabs((*(axis_point_.end() - 1 - i)).first.x), 2);

        // if (errorSq > 45) // 3^2 * 5
        //     (*(axis_point_.end() - 1)).first.x = (*(axis_point_.end() - 2)).first.x;
    }

    double RobotPredictor::CalcVelocity() {
        // 清空未出现装甲板的armor_pos_history_
        static double spinSpeed = 0;
        int           calcCnt   = 0;
        double        v         = 0;
        for (int i = 0; i < 4; i++)
            if (!history_.back().GetArmorStatus(i)) {
                if (!armor_angle_history_[i].empty() && !armor_timeStamp_history_[i].empty()) {
                    double deltaAngle = armor_angle_history_[i].back() - armor_angle_history_[i].front();
                    double deltaTime  = (armor_timeStamp_history_[i].back() - armor_timeStamp_history_[i].front()) / 1000000.0f;
#ifdef PREDICT_DEBUG_OUTPUT
                    std::cout << "deltaAngle = " << deltaAngle << ", deltaTime = " << deltaTime << std::endl;
#endif
                    if (deltaTime > velocity_time_out)
                        spinSpeed = 0, calcCnt = -1;
                    else if (abs(deltaAngle) > 60)
                        v = deltaAngle / deltaTime, calcCnt++, rotation_duration_ = armor_angle_history_[i].size() * 2;
                }
                std::vector<double>().swap(armor_angle_history_[i]);
                std::vector<double>().swap(armor_timeStamp_history_[i]);
                std::vector<cv::Point3f>().swap(armor_pos_history_[i]);
            } else if (!armor_timeStamp_history_[i].empty()) {
                double deltaTime = (armor_timeStamp_history_[i].back() - armor_timeStamp_history_[i].front()) / 1000000.0f;
                if (deltaTime > velocity_time_out)
                    spinSpeed = 0, calcCnt = -1;
            }
        spinSpeed = calcCnt == 1 ? v : spinSpeed;
        // std::cout << "spinSpeed=" << spinSpeed << endl;
        return spinSpeed;
    }

    double RobotPredictor::CalcOnTimeVelocity() {
        static float  last_time = 0;
        static double spinspeed = 0;
        int           maxid     = -1;
        for (int i = 0; i < 4; i++)
            if (history_.back().GetArmorStatus(i)) {
                if (armor_angle_history_[i].size() >= 16 && armor_timeStamp_history_[i].size() >= 16) {
                    if (history_.back().GetResolvedArmors()[i].GetStickerRect().GetSize().area() > maxid == -1 ? 0 : history_.back().GetResolvedArmors()[maxid].GetStickerRect().GetSize().area())
                        maxid = i;
                }
            };
        if (maxid != -1) {
            double angle_diff = *(armor_angle_history_[maxid].end() - 1) - *(armor_angle_history_[maxid].end() - 16);
            double time_diff  = (*(armor_timeStamp_history_[maxid].end() - 1) - *(armor_timeStamp_history_[maxid].end() - 16)) / 1000000.0f;
#ifdef PREDICT_DEBUG_OUTPUT
            cout << "amgle_diff:" << angle_diff << " time_diff:" << time_diff << endl;
#endif

            if (time_diff != 0) {
                spinspeed = angle_diff / time_diff, rotation_duration_ = armor_angle_history_[maxid].size() * 2;
                last_time = history_.back().GetTimeStamp() / 1000000.0f;
            } else if (history_.back().GetTimeStamp() / 1000000.0f - last_time > 2)
                spinspeed = 0;
        } else if (history_.back().GetTimeStamp() / 1000000.0f - last_time > 2)
            spinspeed = 0;
        // std::cout << "OnTime spinSpeed = " << spinspeed << "°" << std::endl;

        return spinspeed;
    }

    int RobotPredictor::GetStatus() {
        static double lastTime  = 0;
        static double lastSpeed = 0;
        static int    times     = 0;
        static int    lastRobot = 0;
        int           statusSum = 0;
#ifdef PREDICT_DEBUG_OUTPUT
        cout << "lastTime:" << lastTime << " times:" << times << endl;
#endif
        if (axis_point_.size() > 15 && norm(DropYAxis((*(axis_point_.end() - 1)).first - (*(axis_point_.end() - 15)).first)) > 10)
            statusSum += 1;
        int status_switch = statusSum;//切换目标时只会有跟随和静止，不会把小陀螺和摇摆从上一个目标带过去
        // if (CalcOnTimeVelocity() * lastSpeed < 0 && abs(CalcOnTimeVelocity()) > 60 && abs(CalcVelocity()) < 540) {
        //     times++;
        //     lastTime = history_.back().GetTimeStamp();
        //     shock_timeStamp_history_.push_back(lastTime);
        // } else if (history_.back().GetTimeStamp() - lastTime > 2 * 1000000.0) {
        //     times = 0;
        //     std::vector<double>().swap(shock_timeStamp_history_);
        // }
        //        if(axis_point_.size() > 5 && norm(DropYAxis((*(axis_point_.end() - 1)).first - (*(axis_point_.end() - 15)).first))  <= 10){
        //            Point3f mean_point = {0};
        //            for(int i=1; i<=5; i++){
        //                mean_point += (*(axis_point_.end() - 1)).first;
        //            }
        //            mean_point /= 5;
        //            float max_diff = {0};
        //            for(int i=1; i<=5; i++){
        //                float temp_diff = {0};
        //                temp_diff += (*(axis_point_.end() - i)).first.x - mean_point.x;
        //                temp_diff += (*(axis_point_.end() - i)).first.y - mean_point.y;
        //                temp_diff += (*(axis_point_.end() - i)).first.z - mean_point.z;
        //                if(temp_diff > max_diff)
        //                    max_diff = temp_diff;
        //            }
        //            cout << "max_diff:" <<max_diff << " norm:"<<norm(DropYAxis((*(axis_point_.end() - 1)).first - (*(axis_point_.end() - 15)).first)) <<endl;
        //            //armor_pos_history_->back();
        //            //armor_pos_history_->back()
        //        }
        // if (times >= 3)
        //     statusSum += 4;
        //else {
        static double last_speed = CalcVelocity();
        static int    time       = 0;
        if (abs(CalcVelocity()) > 90 && CalcVelocity() != last_speed)
            time++;
        else if (abs(CalcVelocity()) < 10)
            time = 0;
        last_speed = CalcVelocity();
        if (time >= 4)
            statusSum += 2;
        //}
        // }
        lastSpeed = CalcOnTimeVelocity();
        if(history_.back().GetRobotType() != lastRobot){//检测到切换目标
            statusSum = status_switch;
            time = 0;//仅仅是切换状态还不够，如果不清空time，下一帧仍可能是小陀螺
            //std::cout << "status_switch:"<<status_switch<<std::endl;
        }
        //std::cout << "status:"<<statusSum<<std::endl;
        lastRobot = history_.back().GetRobotType();
#ifdef VIDEO_DEBUG
        Debug::SetMessage("装甲板预测", "机器人状态", statusSum);
#endif
        return statusSum;
    }

    float RobotPredictor::GetStatusPlus(int query) {
        int status = GetStatus();
        if (status == 2 || status == 3) {
            return 2.0;
        }
        if (status == 4 || status == 5) {
            static bool                  appear[4]    = {0, 0, 0, 0};
            static int                   nums         = 0;
            static double                last_time[4] = {0, 0, 0, 0};
            static double                start_time   = 0;
            static std::pair<int, float> max_angle_id = {-1, 90.0f};
            if (query == 1) {
                if (nums == 1) {
                    for (int i = 0; i < 4; i++) {
                        if (appear[i])
                            return (float)i + 0.1f;
                    }
                };
                if (nums == 3) {
                    for (int i = 0; i < 4; i++) {
                        if (appear[i] && appear[(i + 1) % 4] && appear[(i + 2) % 4])
                            return (float)((i + 1) % 4) + 0.1f;
                    }
                }
                return -1.0f;
            }
            if (query == 2) {
                return (float)max_angle_id.first + 0.1f;
            }
            if (query == 0) {
                for (int i = 0; i < history_.back().GetResolvedArmors().size(); i++) {
                    if (!appear[history_.back().GetResolvedArmors()[i].GetTag()]) {
                        if (!nums)
                            start_time = history_.back().GetTimeStamp();
                        nums++;
                        appear[history_.back().GetResolvedArmors()[i].GetTag()] = 1;
                    }
                    last_time[history_.back().GetResolvedArmors()[i].GetTag()] = history_.back().GetTimeStamp();
                    if (armor_angle_history_[history_.back().GetResolvedArmors()[i].GetTag()].back() < max_angle_id.second) {
                        max_angle_id.first  = history_.back().GetResolvedArmors()[i].GetTag();
                        max_angle_id.second = armor_angle_history_[history_.back().GetResolvedArmors()[i].GetTag()].back();
                    }
                }
                for (int i = 0; i < 4; i++) {
                    if (((history_.back().GetTimeStamp() - last_time[i]) / 1000000.0f > 4) && appear[i]) {
                        appear[i] = 0;
                        nums--;
                        if (!nums)
                            start_time = 0;
                        if (i == max_angle_id.first) {
                            max_angle_id = {-1, 90.0f};
                        }
                    }
                }
                if ((history_.back().GetTimeStamp() - start_time) / 1000000.0f > 2) {
                    if (nums == 1)
                        return 4.1;
                    if (nums == 2)
                        return 4.2;
                    if (nums == 3)
                        return 4.3;
                    if (nums == 4)
                        return 4.4;
                }
                return 4.0;
            }
        }
    }

    cv::Point3f RobotPredictor::AxisLinearRegression() {
        cv::Point3f per_point = {0, 0, 0};
        float       per_time  = 0;
        int         cnt;
        for (cnt = 0; cnt < axis_point_.size() && cnt < 15; cnt++) {
            per_point += (axis_point_.end() - cnt - 1)->first;
            per_time += (axis_point_.end() - cnt - 1)->second;
        }
        per_point /= cnt;
        per_time /= cnt;
        cv::Point3f b1 = {0, 0, 0};
        float       b2 = 0;
        for (int i = 0; i < axis_point_.size() && i < 15; i++) {
            b1 += (axis_point_.end() - i - 1)->first * (axis_point_.end() - i - 1)->second - cnt * per_point * per_time;
            b2 += (axis_point_.end() - i - 1)->second * (axis_point_.end() - i - 1)->second - cnt * per_time * per_time;
        }
        cv::Point3f b = b1 / b2, a = per_point - per_time * b; //此处a与b为回归直线方程y=bx+a中a与b
        cv::Point3f predict_point = (axis_point_.end() - 1)->second * b + a;
#ifdef AXIS_DEBUG
        fout << "axis_point," << predict_point.x << "," << predict_point.z << ",?" << std::endl;
#endif
        return predict_point;
    }

    cv::Point3f RobotPredictor::ShockAbsorber_liner() { //线性回归计算旋转中心
        Point3f axp = Point3f(0, 0, 0);
        // int     cnt, state = GetStatus();
        // if (state == 0 || state == 1)
        //     for (cnt = 1; cnt <= 15 && axis_point_.size() - cnt >= 0; cnt++)
        //         axp += (*(axis_point_.end() - cnt)).first;
        // if ((state == 2 || state == 3) && axis_point_.size() >= 2)
        //     for (cnt = 1; cnt <= rotation_duration_ && axis_point_.size() - cnt >= 0; cnt++)
        //         axp += (*(axis_point_.end() - cnt)).first;
        // axp /= cnt;
        // if (state != 2)/
        //     axp += ((*(axis_point_.end() - 1)).first - (*(axis_point_.end() - cnt)).first) / 2;
        int cnt, state = GetStatus();
        if (state == 0 || state == 1) {
            axp = AxisLinearRegression() / 2 + (*(axis_point_.end() - 1)).first / 2;
        }

        if (state == 2 || state == 3 || state == 4 || state == 5) {
            axp         = AxisLinearRegression() / 2 + (*(axis_point_.end() - 1)).first / 2;
            float per_y = 0;
            for (cnt = 0; cnt < axis_point_.size() && cnt < 15; cnt++) {
                per_y += (axis_point_.end() - cnt - 1)->first.y;
            }
            per_y /= cnt;
            axp.y = per_y;
        }
#ifdef AXIS_DEBUG
        bool debug_error = 0;
        fout << "axis_point," << axp.x << "," << axp.z << ",!" << std::endl;
        fout1 << "axis_point," << (axis_point_.end() - 1)->first.x << "," << (axis_point_.end() - 1)->first.z << ",!" << std::endl;
#endif
        return axp;
    }

    Mat RobotPredictor::PolyFit(int n) {
        int size = in.size();
        //所求未知数个数
        int x_num = n + 1; //拟合方程阶数
        //构造矩阵U和Y
        Mat mat_u(size, x_num, CV_64F);
        Mat mat_y(size, 1, CV_64F);

        for (int i = 0; i < mat_u.rows; ++i)
            for (int j = 0; j < mat_u.cols; ++j) {
                mat_u.at<double>(i, j) = pow(in[i].x, j);
            }

        for (int i = 0; i < mat_y.rows; ++i) {
            mat_y.at<double>(i, 0) = in[i].y;
        }

        //矩阵运算，获得系数矩阵K
        Mat mat_k(x_num, 1, CV_64F);
        mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;
        return mat_k;
    }

    Point3f RobotPredictor::RegressionEnd(Mat M, int n) {
        Point3f End;
        float   real_x = 0;
        //取最后一个
        real_x = (*(axis_point_.end() - 1)).first.x;

        End.x = real_x;
        End.y = 0;

        for (int j = 0; j < n + 1; ++j) {
            End.z += M.at<double>(j, 0) * pow(real_x, j);
        }
#ifdef AXIS_DEBUG
        fout << "axis_point," << End.x << "," << End.z << ",?" << std::endl;
#endif
        return End;
    }

    bool RobotPredictor::Cmp(cv::Point2f a, cv::Point2f b) { return a.x < b.x; }

    //减震器
    cv::Point3f RobotPredictor::ShockAbsorber() {
        Point3f axp(0, 0, 0);
        Point2f p;
        float   y = 0;
        int     cnt;

        for (cnt = 1; cnt <= rotation_duration_ && axis_point_.size() - cnt >= 0; cnt++) {
            p.x = (*(axis_point_.end() - cnt)).first.x;
            p.y = (*(axis_point_.end() - cnt)).first.z;
            in.push_back(p);
            y += (*(axis_point_.end() - cnt)).first.y;
        }

        sort(begin(in), end(in), Cmp);
        int n = 2; //多項式擬合的階數
        Mat M = PolyFit(n);
        // axp = RegressionMid(M, n);
        axp = RegressionEnd(M, n);

        y /= cnt;
        // axp.y = y;
        std::vector<cv::Point2f>().swap(in);
        axp   = axp * 0.7 + (*(axis_point_.end() - 1)).first * 0.3;
        axp.y = (*(axis_point_.end() - 1)).first.y;
#ifdef AXIS_DEBUG
        bool debug_error = 0;
        fout << "axis_point," << axp.x << "," << axp.z << ",!" << std::endl;
        fout1 << "axis_point," << (axis_point_.end() - 1)->first.x << "," << (axis_point_.end() - 1)->first.z << ",?" << std::endl;
#endif
        return axp;
    }

    int RobotPredictor::armorDecision(std::vector<tdttoolkit::ResolvedArmor> armors) {
        static int lastDecesion = -1;
        int        Points[armors.size()];
        memset(Points, 0, sizeof(Points));
        for (int i = 0; i < armors.size(); i++) {
            if (isTaged[armors[i].GetTag()]) {
                if (armors[i].GetBelievable() == 1)
                    Points[i] += 500;                                           //可信装甲板得分
                Points[i] += armors[i].GetStickerRect().GetSize().area() / 300; //面积得分
                Points[i] += armors[i].GetPolar().distance / 10;                //距离得分
                if (armors[i].GetTag() == lastDecesion)
                    Points[i] += 50; //上次击打装甲板得分
            }
        };
        int ret = 0;
        for (int i = 0; i < armors.size(); i++)
            ret = Points[ret] > Points[i] ? ret : i;
        lastDecesion = armors[ret].GetTag();
        return ret;
    }

    void RobotPredictor::DistanceCompensation(cv::Point3f &axis_point) {
        float axis_distance  = sqrt(axis_point.x * axis_point.x + axis_point.z * axis_point.z);                                                                               // 轴心到世界坐标系原点大小                                                                                                                                //轴心到世界坐标系原点距离大小
        float armor_distance = axis_distance - (robotRadius[history_.back().GetRobotType()][0].GetRadius() + robotRadius[history_.back().GetRobotType()][1].GetRadius()) / 2; //装甲板中心平均值到世界坐标系原点大小
        float y              = axis_point.y;
        axis_point           = axis_point / axis_distance * armor_distance;
        axis_point.y         = y;
    };

    Point3f RobotPredictor::AxisFilter(cv::Point3f axis_point, float bulletspeed) {
        cv::Point3f        predict_axis;
        cv::Point3f        Filter_speed;
        static cv::Point3f last_axis = axis_point;
        cv::Point3f        diff      = axis_point - last_axis;
        last_axis                    = axis_point;
        if (axis_point_.size() < 2) {
            axisFilter_[0].Estimate(axis_point.x, false);
            axisFilter_[1].Estimate(axis_point.y, false);
            axisFilter_[2].Estimate(axis_point.z, false);
            return axis_point;
        }
        float time_diff = (axis_point_.end() - 1)->second - (axis_point_.end() - 2)->second;
        time_diff /= 1000.0f;
        cv::Point3f speed = diff / time_diff;
        float       qX = GetKalmanProcessNoise(0.1), qY = 0.001, qZ = qX;
        float       rX = 0.3 * time_diff, rY = 2 * time_diff, rZ = rX;
        // qZ = qX = qY = GetKalmanProcessNoise();
        // GetKalmanProcessNoise =        good!   <0.5
#ifdef PREDICT_DEBUG_OUTPUT
        cout << "qX:" << qX << " rX:" << rX << " rY:" << rY << endl;
#endif
        axisFilter_[0].SetQ(qX);
        axisFilter_[0].SetR(rX);
        Filter_speed.x = axisFilter_[0].Estimate(speed.x, true);
        axisFilter_[1].SetQ(qY);
        axisFilter_[1].SetR(rY);
        Filter_speed.y = axisFilter_[1].Estimate(speed.y, true);
        axisFilter_[2].SetQ(qZ);
        axisFilter_[2].SetR(rZ);
        Filter_speed.z = axisFilter_[2].Estimate(speed.z, true);
        // std::cout << "AxisFilter speed=" << speed << std::endl;
        float predictTime = (RectangularToPolar(axis_point).distance / bulletspeed + fire_delay_) * 1000;
        predict_axis      = axis_point + predictTime * Filter_speed;
        predict_axis.y -= 20;
        return predict_axis;
    }

    double RobotPredictor::GetKalmanProcessNoise(float orginNoise) { return orginNoise * 42.6 * 42.6 / (kalman_fps * kalman_fps); }

    double RobotPredictor::GyroFireTime(){
        double  currentRobotRadius[2] = {(double)robotRadius[history_.back().GetRobotType()][0].GetRadius(), (double)robotRadius[history_.back().GetRobotType()][1].GetRadius()};
        double usedRadius = (currentRobotRadius[0] + currentRobotRadius[1]) / 2.0;//将长短轴平均值当作半径来算
        double armorRadius = 0;
        double angularVelocity = CalcVelocity();
        if(history_.back().GetRobotType() == 1){//目标为英雄时
            armorRadius = 0.220;//官方给的是0.23m,减去灯条部分所以小了一点 0.23->0.22
        }
        else {
            armorRadius = 0.125;//同上理    0.135->0.125
        }
        angularVelocity = angularVelocity == 0 ? 1 : angularVelocity;
        //
        double FireTime = fabs(2.0*atan2(0.5*armorRadius, usedRadius/100.0)/CV_PI*180.0 / angularVelocity*1000)*0.8;//单位：ms
        std::cout<<"GYRO::y："<<0.5*armorRadius<<std::endl;
        std::cout<<"GYRO::x："<<usedRadius/100.0<<std::endl;
        std::cout<<"GYRO::atan2："<<atan2(0.5*armorRadius, usedRadius/100.0)/CV_PI*180.0<<std::endl;
        std::cout<<"GYRO::angularVelocity："<<angularVelocity<<std::endl;
        std::cout<<"GYRO::FireTime1:"<<FireTime<<std::endl;
        FireTime = min(64.0,FireTime);
        FireTime = max(6.0,FireTime);//返回开火时间最小为默认值6ms,最大为无限开火最大时间64ms
        std::cout<<"GYRO::FireTime2:"<<FireTime<<std::endl;
        return FireTime;
    }

    void RobotPredictor::FollowModeShoot(const tdttoolkit::ReceiveMessage &receiveMessage, tdtusart::Send_Struct_t &sendStruct) {
        Robot &currentRobot = *(history_.end() - 1);
        Robot &lastRobot    = (history_.size() > 1) ? (*(history_.end() - 2)) : (*(history_.end() - 1));
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "Status: 跟随模式" << std::endl;
#endif

        static tdttoolkit::ShootPlatform      lastPlatform = receiveMessage.shoot_platform;
        static deque<std::pair<float, float>> pfHistory;

        float timeGap = float(currentRobot.GetTimeStamp() - lastRobot.GetTimeStamp()) / 1000000.0f;
        timeGap       = timeGap == 0 ? 1 : timeGap;
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "timeGap: " << timeGap << " s" << std::endl;
        std::cout << "yawGap = " << receiveMessage.shoot_platform.yaw - lastPlatform.yaw << " rad" << std::endl;
#endif
        float pfYawSpeed   = (receiveMessage.shoot_platform.yaw - lastPlatform.yaw) / timeGap;
        float pfPitchSpeed = (receiveMessage.shoot_platform.pitch - lastPlatform.pitch) / timeGap;
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "GSMS: pfYawSpeed = " << pfYawSpeed << std::endl;
#endif

        pfHistory.emplace_back(std::make_pair(pfYawSpeed, pfPitchSpeed));
        if (pfHistory.size() > 5)
            pfHistory.pop_front();

        float pfYawAcc = 0.0f, pfPitchAcc = 0.0f;

        if (pfHistory.size() > 1)
            for (int i = 0; i < pfHistory.size() - 1; ++i) {
                pfYawAcc += pow(pfHistory[i + 1].first - pfHistory[i].first, 2);
                pfPitchAcc += pow(pfHistory[i + 1].second - pfHistory[i].second, 2);
            }

        // std::cout << "Follow: " << "pfYawSpeed = " << pfYawSpeed << ", pfYawAcc = " << pfYawAcc << std::endl;

        static tdttoolkit::ReceiveMessage lastMessage     = receiveMessage;
        float                             usart_yaw_speed = receiveMessage.shoot_platform.yaw - lastMessage.shoot_platform.yaw;
        float                             time_diff       = history_.back().GetTimeStamp() - (history_.end() - 2)->GetTimeStamp();
        time_diff                                         = time_diff == 0 ? 1 : time_diff / 1e3;
        usart_yaw_speed /= time_diff;
        usart_yaw_speed *= 1e3;
        static std::deque<float> last_speed;
        last_speed.push_back(usart_yaw_speed);
        if (last_speed.size() > 5)
            last_speed.pop_front();
        float usart_yaw_as = 0;
        for (int i = 0; i < last_speed.size(); i++) {
            usart_yaw_as += (usart_yaw_speed - last_speed[i]) * (usart_yaw_speed - last_speed[i]);
        }
        float qY, rY, qP, rP;
        float a_rY=0.2,b_rY=0.1,c_rY=1.2;
//        LoadParam::ReadTheParam("rY里加的常数",a_rY);
//        LoadParam::ReadTheParam("rY里角速度平方的系数",b_rY);
//        LoadParam::ReadTheParam("rY里角速度方差的系数",c_rY);
        LoadParam::ReadTheParam("Constants_in_rY",a_rY);
        LoadParam::ReadTheParam("Squared_in_rY",b_rY);
        LoadParam::ReadTheParam("Variance_in_rY",c_rY);
        rY = a_rY + fabs(usart_yaw_speed * usart_yaw_speed) * b_rY + fabs(usart_yaw_as) * c_rY;
        std::cout << "kalman speed=" << usart_yaw_speed << " as=" << usart_yaw_as << " time_diff=" << time_diff << std::endl;
        std::cout << "qY:" << qY << " qP:" << qP << endl;
        rP          = rY;
        lastMessage = receiveMessage;
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝" << endl;
        std::cout << "跟随模式过程噪声:" << endl;
        std::cout << "qY:" << qY << " qP:" << qP << endl;
        std::cout << "注意：看qY qP时别看最后一次输出，从倒数第二帧看起，神秘原因最后一帧帧率会很低，所以qY qP会偏大" << endl;
        std::cout << "＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝" << endl;
#endif
        float constants_qY;
        LoadParam::ReadTheParam("qY里time_diff的系数",constants_qY);
        qY = constants_qY * time_diff;
        // qY=0.02;
        qP = qY / 10;
#ifdef PREDICT_DEBUG_OUTPUT
        cout << "qY:" << qY << " rY:" << rY << endl;
#endif

#ifdef VIDEO_DEBUG
        Debug::SetMessage("卡尔曼滤波", "跟随模式Q　&　R", cv::Vec2f(qY, rY));
#endif
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "yawQandR: " << pfYawSpeed / CV_PI * 180 << ", " << pfYawAcc << ", " << qY << ", " << rY << std::endl;
#endif
        lastPlatform              = receiveMessage.shoot_platform;
        const auto &currentArmors = currentRobot.GetResolvedArmors();
        const auto &lastArmors    = lastRobot.GetResolvedArmors();
        float       yawLinearSpeed[4];
        float       yawStatePost[4];
        for (int i = 0; i < currentArmors.size(); ++i) {
            int  j, tag = currentArmors[i].GetTag();
            bool flag = false;
            for (int n = 0; n < lastArmors.size(); ++n)
                if (lastArmors[n].GetTag() == tag)
                    j = n, flag = true; // 两帧之间有连接成功的装甲板，编号为tag
                                        // TODO: --------------------------------------------EKF_IMM-------------------------------------------------------------
                                        /*
                                         if (flag) {
                                             Polar3f currentPolar = currentArmors[i].GetPolar();
                                             Polar3f lastPolar    = lastArmors[j].GetPolar();
                                             float   radius = -1, yawDiff = CalcAngleDifference(currentPolar.yaw, lastPolar.yaw);
                                             float   pitchDiff      = CalcAngleDifference(currentPolar.pitch, lastPolar.pitch);
                                             float   yawLinearSpeed = yawDiff * (currentPolar.distance * 10) / ((currentRobot.GetTimeStamp() - lastRobot.GetTimeStamp()) / 1000.0f);   //单位 m/s
                                             float   pitchSpeed     = pitchDiff * (currentPolar.distance * 10) / ((currentRobot.GetTimeStamp() - lastRobot.GetTimeStamp()) / 1000.0f); //单位 m/s
                                             if (((currentRobot.GetTimeStamp() - lastRobot.GetTimeStamp()) / 1000.0f) == 0)                                                            //防止分母为0导致 nan(这个bug我找了一天，吐了！）
                                                 yawLinearSpeed = pitchSpeed = 0;
                                             isTaged[tag] = true;
                            
                                             //                yawLinearFilter_[tag].SetQ(qY);
                                             //                yawLinearFilter_[tag].SetR(rY);
                                             //                pitchLinearFilter_[tag].SetQ(qP);
                                             //                pitchLinearFilter_[tag].SetR(rP);
                            
                                             if (tag_to_radius_index_[tag] == 0)
                                                 radius = robotRadius[currentRobot.GetRobotType()][0].GetRadius();
                                             else if (tag_to_radius_index_[tag] == 1)
                                                 radius = robotRadius[currentRobot.GetRobotType()][1].GetRadius();
                                             else
                                                 radius = 25;
                            
                                             cv::Point3f currentPoint = PolarToRectangular(currentPolar);
                                             cv::Point3f change       = {0};
                                             update_vector(0) = currentPolar.yaw, update_vector(1) = currentPolar.pitch;
                                             if (history_.size() > 1) {
                                                 update_vector(2) = yawLinearSpeed, update_vector(3) = pitchSpeed;
                                             } else {
                                                 update_vector(2) = 0, update_vector(3) = 0;
                                             }
                            
                                             /// 对视频进行观看时装甲板预测与实际不准主要是因为fire_delay_的开火延迟（预测时间如果过久会导致预测不准，勉强可以理解）
                                             double predictTime = (currentPolar.distance / (receiveMessage.shoot_platform.bulletspeed) + fire_delay_); // 单位 s
                            
                                             double t_update = cv::getTickCount();
                                             armor_model[tag].imm_model->updateOnce((currentRobot.GetTimeStamp() / 1000000.0f + predictTime), &update_vector);
                             #ifdef PREDICT_DEBUG_OUTPUT
                                             std::cout << "updateOnce1耗时:" << (cv::getTickCount() - t_update) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
                             #endif
                            
                                             float yawStatePost = history_.size() == 1 ? 0 : armor_model[tag].imm_model->x()(2); // 单位 m/s
                             #ifdef VIDEO_DEBUG
                                             Debug::SetMessage("装甲板预测", "yaw线速度", yawStatePost);
                             #endif
                                             float pitchStatePost = history_.size() == 1 ? 0 : armor_model[tag].imm_model->x()(3); // 单位 m/s
                                             currentPolar_[tag]   = currentPolar;
                            
                                             // Polar3f predictPolar(currentPolar.distance, currentPolar.yaw + predictTime * yawStatePost * 100 / currentPolar.distance, currentPolar.pitch + predictTime * pitchStatePost * 100 / currentPolar.distance);
                                             //Polar3f predictPolar(currentPolar.distance, armor_model[tag].imm_model->x()(0), armor_model[tag].imm_model->x()(1));
                                             Polar3f predictPolar(currentPolar.distance, currentPolar.yaw + predictTime * armor_model[tag].imm_model->x()(2) * 100 / currentPolar.distance,
                                             currentPolar.pitch + predictTime * armor_model[tag].imm_model->x()(3) * 100 / currentPolar.distance );
                            
                                             cv::Point3f predictPoint     = PolarToRectangular(predictPolar);
                                             cv::Point2d currentProjPoint = armorResolver->WorldToPixel(receiveMessage.shoot_platform, currentPoint);
                                             cv::Point2d predictProjPoint = armorResolver->WorldToPixel(receiveMessage.shoot_platform, predictPoint);
                             #ifdef VIDEO_DEBUG
                                             Debug::AddPoint("跟随模式装甲板中心.x", "实际.x", Point2f(Time::GetTimeNow() / 1000000.0f, currentProjPoint.x));
                                             Debug::AddPoint("跟随模式装甲板中心.y", "实际.y", Point2f(Time::GetTimeNow() / 1000000.0f, currentProjPoint.y));
                                             Debug::AddPoint("跟随模式装甲板中心.x", "预测.x", Point2f(Time::GetTimeNow() / 1000000.0f + predictTime, predictProjPoint.x)); //
                                             Debug::AddPoint("跟随模式装甲板中心.y", "预测.y", Point2f(Time::GetTimeNow() / 1000000.0f + predictTime, predictProjPoint.y)); //
                             #endif
                                             predictPolar_[tag] = predictPolar;
                            
                                         } else {
                                             Polar3f currentPolar = currentArmors[i].GetPolar();
                                             /// 这还能算出负数？
                             #ifdef PREDICT_DEBUG_OUTPUT
                                             cout << "currentRobot.GetTimeStamp994:" << currentRobot.GetTimeStamp() / 1000.0f << endl;
                                             cout << "armor_modelp994:" << armor_model[tag].imm_model->stamp() * 1000.0f << endl;
                                             cout << "predict.cpp994:" << currentRobot.GetTimeStamp() / 1000.0f - armor_model[tag].imm_model->stamp() * 1000.0f << endl;
                             #endif
                                             if (abs(currentRobot.GetTimeStamp() / 1000.0f - armor_model[tag].imm_model->stamp() * 1000.0f) < 50) { //前面的单位是us, 后面的单位是s
                                                 Polar3f predictPolar = predictPolar_[tag];
                                                 float   predictTime  = (currentPolar.distance / (receiveMessage.bulletspeed) + fire_delay_);
                            
                                                 update_vector(0) = currentPolar.yaw, update_vector(1) = currentPolar.pitch;
                                                 double t_update = cv::getTickCount();
                                                 armor_model[tag].imm_model->updateOnce((currentRobot.GetTimeStamp() / 1000000.0f + predictTime), &update_vector);
                             #ifdef PREDICT_DEBUG_OUTPUT
                                                 std::cout << "updateOnce2耗时:" << (cv::getTickCount() - t_update) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
                             #endif
                                                 predictPolar.yaw += armor_model[tag].imm_model->x()(0);
                                                 predictPolar.pitch += armor_model[tag].imm_model->x()(1);
                                                 predictPolar_[tag] = predictPolar;
                                             } else {
                                                 //                    state_vector = Eigen::VectorXd::Zero(6);
                                                 //                    state_vector(4) = 10, state_vector(5) = 10;
                                                 // armor_model[tag].imm_model = armor_model[tag].generateIMMModel(currentRobot.GetTimeStamp() / 1000000.0f, Eigen::VectorXd::Zero(4));// 置零
                                                 // update_vector = Eigen::VectorXd::Zero(4);
                                                 update_vector(2) = 0, update_vector(3) = 0;
                                                 currentPolar_[tag]   = currentArmors[i].GetPolar();
                                                 float   predictTime  = (currentPolar.distance / (receiveMessage.bulletspeed) + fire_delay_);
                                                 Polar3f predictPolar = currentPolar_[tag];
                                                 double  t_update     = cv::getTickCount();
                                                 armor_model[tag].imm_model->updateOnce(currentRobot.GetTimeStamp() / 1000000.0f + predictTime, &update_vector); // 置零
                             #ifdef PREDICT_DEBUG_OUTPUT
                                                 std::cout << "updateOnce3耗时:" << (cv::getTickCount() - t_update) / cv::getTickFrequency() * 1000 << "ms" << std::endl;
                             #endif
                                                 predictPolar_[tag].yaw += armor_model[tag].imm_model->x()(0);
                                                 predictPolar_[tag].pitch += armor_model[tag].imm_model->x()(1);
                                             }
                                         }
                             */
            // TODO: --------------------------------------------EKF_IMM-------------------------------------------------------------

            if (flag) {
                Polar3f currentPolar = currentArmors[i].GetPolar();
                Polar3f lastPolar    = lastArmors[j].GetPolar();
                float   radius = -1, yawDiff = CalcAngleDifference(currentPolar.yaw, lastPolar.yaw);
                float   pitchDiff                         = CalcAngleDifference(currentPolar.pitch, lastPolar.pitch);
                yawLinearSpeed[currentArmors[i].GetTag()] = yawDiff * (currentPolar.distance / 100) / (time_diff / 1000.0f);   //单位 m/s
                float pitchSpeed                          = pitchDiff * (currentPolar.distance / 100) / (time_diff / 1000.0f); //单位 m/s
                isTaged[tag]                              = true;

                yawLinearFilter_[tag].SetQ(qY);
                yawLinearFilter_[tag].SetR(rY);
                pitchLinearFilter_[tag].SetQ(qP);
                pitchLinearFilter_[tag].SetR(rP);

                std::cout << "GSMS:1 yawDiff = " << yawDiff << ", distance = " << currentPolar.distance << ", yawLinearSpeed = " << yawLinearSpeed << ", timeGap = " << (currentRobot.GetTimeStamp() - lastRobot.GetTimeStamp()) / 1000000.0f << std::endl;

                if (tag_to_radius_index_[tag] == 0)
                    radius = robotRadius[currentRobot.GetRobotType()][0].GetRadius();
                else if (tag_to_radius_index_[tag] == 1)
                    radius = robotRadius[currentRobot.GetRobotType()][1].GetRadius();
                else
                    radius = 25;

                // float linearSpinSpeed = spinSpeed / 180 * CV_PI * radius / 100; //单位 m/s

                // if (yawDiff > 0)
                //     yawLinearSpeed_ = 1.0 * yawLinearSpeed - 0.6 * linearSpinSpeed; // 单位 m/s
                // else
                //     yawLinearSpeed_ = 0.8 * yawLinearSpeed - 1.0 * linearSpinSpeed;

                std::cout << "GSMS:2 yawFilter[" << tag << "].statePre = " << yawLinearFilter_[tag].statePre.at<float>(0) << ", measurement = " << yawLinearSpeed << ", statePost = " << yawLinearFilter_[tag].statePost.at<float>(0) << std::endl;
                yawStatePost[tag] = history_.size() == 1 ? yawLinearFilter_[tag].Estimate(yawLinearSpeed[tag], false) : yawLinearFilter_[tag].Estimate(yawLinearSpeed[tag], true); // 单位 m/s
#ifdef VIDEO_DEBUG
                Debug::SetMessage("装甲板预测", "yaw线速度", yawStatePost[tag]);
#endif
                float pitchStatePost = history_.size() == 1 ? pitchLinearFilter_[tag].Estimate(pitchSpeed, false) : pitchLinearFilter_[tag].Estimate(pitchSpeed, true); // 单位 m/s
                currentPolar_[tag]   = currentPolar;
                /// 100是从cm / (m/s) -> m / (m/s)所需的转换倍数　fire_delay_是开火延迟（能否实时更新？）
                /// 对视频进行观看时装甲板预测与实际不准主要是因为fire_delay_的开火延迟（预测时间如果过久会导致预测不准，勉强可以理解）
                double predictTime = (currentPolar.distance / (receiveMessage.shoot_platform.bulletspeed) + fire_delay_); // 单位 s

                Polar3f predictPolar(currentPolar.distance, currentPolar.yaw + predictTime * yawStatePost[tag] * 100 / currentPolar.distance, currentPolar.pitch + predictTime * pitchStatePost * 100 / currentPolar.distance);

                // Polar3f predictPolar(currentPolar.distance, currentPolar.yaw + predictTime * yawStatePost * 100 / currentPolar.distance, currentPolar.pitch);

                // if(yawStatePost < -0.1 && yawStatePost > -0.2)
                //     predictPolar.yaw -= 0.015;
                // else if(yawStatePost < -0.2)
                //     predictPolar.yaw -= 0.04;

                // if (yawStatePost < -0.2)
                //     predictPolar.yaw -= 0.035;
                // else if(yawStatePost >= -0.2 && yawStatePost < -0.1)
                //     predictPolar.yaw -= 0.025;
                // else if (yawStatePost >= 0.1 && yawStatePost < 0.2)
                //     predictPolar.yaw += 0.01;
                // else if(yawStatePost >= 0.2)
                //     predictPolar.yaw += 0.02;

                cv::Point3f currentPoint     = PolarToRectangular(currentPolar);
                cv::Point3f predictPoint     = PolarToRectangular(predictPolar);
                cv::Point2d currentProjPoint = armorResolver->WorldToPixel(receiveMessage.shoot_platform, currentPoint);
                cv::Point2d predictProjPoint = armorResolver->WorldToPixel(receiveMessage.shoot_platform, predictPoint);
#ifdef VIDEO_DEBUG
                Debug::AddPoint("跟随模式装甲板中心.x", "实际.x", Point2f(Time::GetTimeNow() / 1000000.0f, currentProjPoint.x));
                Debug::AddPoint("跟随模式装甲板中心.y", "实际.y", Point2f(Time::GetTimeNow() / 1000000.0f, currentProjPoint.y));
                Debug::AddPoint("跟随模式装甲板中心.x", "预测.x", Point2f(Time::GetTimeNow() / 1000000.0f + predictTime, predictProjPoint.x)); //+ predictTime
                Debug::AddPoint("跟随模式装甲板中心.y", "预测.y", Point2f(Time::GetTimeNow() / 1000000.0f + predictTime, predictProjPoint.y)); //
#endif
                predictPolar_[tag] = predictPolar;

            } else {
                Polar3f currentPolar = currentArmors[i].GetPolar();
                cout << "predict.cpp1094" << currentRobot.GetTimeStamp() / 1000.0f - yawLinearFilter_[tag].GetLastTime() / 1.0f << endl;
                if (currentRobot.GetTimeStamp() / 1000.0f - yawLinearFilter_[tag].GetLastTime() / 1.0f < 50) {
                    Polar3f predictPolar = predictPolar_[tag];
                    float   predictTime  = (currentPolar.distance / (receiveMessage.bulletspeed) + fire_delay_);
                    predictPolar.yaw += predictTime * yawLinearFilter_[tag].statePost.at<float>(0) * 100 / currentPolar_[tag].distance;
                    predictPolar.pitch += predictTime * pitchLinearFilter_[tag].statePost.at<float>(0) * 100 / currentPolar_[tag].distance;
                    predictPolar_[tag] = predictPolar;
                } else {
                    yawLinearFilter_[tag].Estimate(0, false); // 置零
                    currentPolar_[tag]   = currentArmors[i].GetPolar();
                    float   predictTime  = (currentPolar.distance / (receiveMessage.bulletspeed) + fire_delay_);
                    Polar3f predictPolar = currentPolar_[tag];
                    predictPolar_[tag].yaw += predictTime * yawLinearFilter_[tag].statePost.at<float>(0) * 100 / currentPolar_[tag].distance;
                    predictPolar_[tag].pitch += predictTime * pitchLinearFilter_[tag].statePost.at<float>(0) * 100 / currentPolar_[tag].distance;
                }
            }
            //*/
        }

        // for(int i = 0; i < 4; ++i)
        // {
        //     std::cout << "currentPolar[" << i << "] = " << currentPolar_[i] << std::endl;
        //     std::cout << "predictPolar[" << i << "] = " << predictPolar_[i] << std::endl;
        // }

        static int lastBeatTag = -1;
        int        beatTag     = currentArmors[armorDecision(currentArmors)].GetTag();
        double     spinSpeed   = CalcVelocity();
        if (fabs(spinSpeed) > 20) {
            if (spinSpeed > 0) // 顺时针旋转
                beatTag = (currentArmors.size() == 2) ? currentArmors[1].GetTag() : currentArmors[0].GetTag();
            else
                beatTag = currentArmors[0].GetTag();
        }
#ifdef VIDEO_DEBUG
        Debug::AddPoint("跟随模式速度", "滤波前", Point2f(history_.back().GetTimeStamp(), (float)yawLinearSpeed[beatTag]));
        Debug::AddPoint("跟随模式速度", "滤波后", Point2f(history_.back().GetTimeStamp(), (float)yawStatePost[beatTag]));
#endif
        Polar3f orientedPolar;
        if (beatTag != -1) {
            orientedPolar = predictPolar_[beatTag];
            lastBeatTag   = beatTag;
        } else {
            sendStruct.beat  = false;
            sendStruct.pitch = receiveMessage.shoot_platform.pitch / CV_PI * 180; //当前角度
            sendStruct.yaw   = receiveMessage.shoot_platform.yaw / CV_PI * 180;   //当前角度
#ifdef PREDICT_DEBUG_OUTPUT
            std::cout << "SendMessage1: " << sendStruct.yaw << ", " << sendStruct.pitch << "（角度）\n";
#endif
        }
        cv::Point3f orientedPoint = PolarToRectangular(orientedPolar);
#ifdef VIDEO_DEBUG
        cv::Point2d orientedProjPoint = armorResolver->WorldToPixel(receiveMessage.shoot_platform, orientedPoint);
        Debug::SetMessage("装甲板预测", "跟随预测点反投影坐标", cv::Vec2d(orientedProjPoint));
        Debug::SetMessage("装甲板预测", "跟随预测点世界坐标", cv::Vec3f(orientedPoint));
        Debug::SetMessage("装甲板预测", "跟随预测点极坐标", cv::Vec3f(orientedPolar.distance, orientedPolar.yaw, orientedPolar.pitch));
        Debug::AddCircle("跟随模式预测点", orientedProjPoint, 6, cv::Scalar(0, 0, 255));
#endif
        float x, y;
        y                   = orientedPoint.y;
        x                   = sqrt(orientedPoint.x * orientedPoint.x + orientedPoint.z * orientedPoint.z);
        cv::Vec2f exitAngle = ParabolaSolve(cv::Point2f(x, y), receiveMessage.bulletspeed);
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "exitAngle: " << x << ", " << y << ", " << receiveMessage.bulletspeed << ", " << exitAngle << std::endl;
#endif
        float pitch = fabs(exitAngle[0] - receiveMessage.shoot_platform.pitch) < fabs(exitAngle[1] - receiveMessage.shoot_platform.pitch) ? exitAngle[0] : exitAngle[1];

        // if (orientedPolar.distance < 500 && orientedPolar.distance > 300)
        //     pitch -= 0.02;
        // else if (orientedPolar.distance <= 300 && orientedPolar.distance > 150)
        //     pitch -= 0.01;
        // pitch+=0.025;
        // pitch+=0.01;
        float yaw_offset = 0.01,pitch_offset = 0.0584;//TODO:跟随模式偏置
        LoadParam::ReadTheParam("Follow_Yaw_offset",yaw_offset);
        LoadParam::ReadTheParam("Follow_Pitch_offset",pitch_offset);
        orientedPolar.yaw += yaw_offset;
        pitch = pitch + pitch_offset - 0.038 * log(receiveMessage.bulletspeed / 100);


        sendStruct.pitch = (pitch) / CV_PI * 180.0f;
        sendStruct.yaw   = (orientedPolar.yaw) / CV_PI * 180.0f;
        //过圈处理
        // if (receiveMessage.shoot_platform.yaw > 0) {
        float recv_yaw         = receiveMessage.shoot_platform.yaw / CV_PI * 180;
        float platformPosition = recv_yaw - (int)(recv_yaw / 360.0f) * 360.0f;                       //电控云台位置
        float visionPosition   = platformPosition > 180 ? platformPosition - 360 : platformPosition; //电控位置在视觉坐标系位置
        float diff             = sendStruct.yaw - visionPosition;
        diff                   = diff > 180 ? diff - 360 : diff < -180 ? diff + 360 : diff;
        sendStruct.yaw         = recv_yaw + diff;
        // }
        // sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) + 0.5)) * 360.0f;
        // else if (receiveMessage.shoot_platform.yaw < 0) {
        //     float platformPosition = receiveMessage.shoot_platform.yaw - (int)(receiveMessage.shoot_platform.yaw / 360) * 360.0f; //电控云台位置
        //     float visionPosition   = platformPosition < -180 ? platformPosition + 360 : platformPosition;                         //视觉位置
        //     sendStruct.yaw += receiveMessage.shoot_platform.yaw - visionPosition;
        // }
        // sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) - 0.5)) * 360.0f;
#ifdef VIDEO_DEBUG
        // cv::Point2d compensation(CalcProjectPoint(receiveMessage.shoot_platform, PolarToRectangular(Polar3f(orientedPolar.distance, sendStruct.yaw, sendStruct.pitch))));
        cv::Point3f compPoint    = PolarToRectangular(Polar3f(orientedPolar.distance, orientedPolar.yaw, pitch));
        cv::Point2d compensation = armorResolver->WorldToPixel(receiveMessage.shoot_platform, compPoint);
        Debug::AddCircle("跟随模式预测点重力补偿", compensation, 6, cv::Scalar(125, 125, 255));
#endif

        float deltaX, deltaY, deltaX_limit, deltaY_limit;
        deltaX   = tan(orientedPolar.yaw - receiveMessage.shoot_platform.yaw) * x;
        deltaY   = tan(pitch - receiveMessage.shoot_platform.pitch) * x;
        int type = (int)currentRobot.GetRobotType();
        LoadParam::ReadTheParam("deltaX_limit",deltaX_limit);
        LoadParam::ReadTheParam("deltaY_limit",deltaY_limit);
        sendStruct.beat = fabs(deltaX) < deltaX_limit && fabs(deltaY) < deltaY_limit;
        isFire_         = sendStruct.beat;
#ifdef VIDEO_DEBUG
        if (isFire_) {
            Debug::AddCircle("跟随模式预测点重力补偿", compensation, 6, cv::Scalar(0, 255, 0));
            Debug::AddRotatedRect("开火高亮", RotatedRect(compensation, Size2f(20, 20), 45), cv::Scalar(0, 255, 0));
        }
#endif
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "PYMS: deltaX = " << deltaX << ", deltaY = " << deltaY << std::endl;
        std::cout << "PYMS: beat = " << bool(sendStruct.beat) << std::endl;
#endif
    }

    void RobotPredictor::GyroModeShoot(const tdttoolkit::ReceiveMessage &receiveMessage, tdtusart::Send_Struct_t &sendStruct) {
        sendStruct.beat = false;
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "Status: 小陀螺模式" << std::endl;
#endif
        if (axis_point_.size() < 2) {
            sendStruct.beat  = false;
            sendStruct.pitch = receiveMessage.shoot_platform.pitch / CV_PI * 180; //当前角度
            sendStruct.yaw   = receiveMessage.shoot_platform.yaw / CV_PI * 180;   //当前角度
            return;
        }

        cv::Point3f currentAxisPoint = ShockAbsorber_liner();
        if (RectangularToPolar(currentAxisPoint).distance > 1000) {
            sendStruct.beat  = false;
            sendStruct.pitch = receiveMessage.shoot_platform.pitch / CV_PI * 180; //当前角度
            sendStruct.yaw   = receiveMessage.shoot_platform.yaw / CV_PI * 180;   //当前角度
            return;
        }
        Polar3f axisPolar          = RectangularToPolar(currentAxisPoint);
        Point3f predict_axis       = AxisFilter(currentAxisPoint, receiveMessage.shoot_platform.bulletspeed);
        float   predictTime_rotate = (RectangularToPolar(currentAxisPoint).distance / (receiveMessage.shoot_platform.bulletspeed) + fire_delay_);
#ifdef VIDEO_DEBUG
        cv::Point2d projRotCenter    = armorResolver->WorldToPixel(receiveMessage.shoot_platform, currentAxisPoint);
        cv::Point2d projPreRotCenter = armorResolver->WorldToPixel(receiveMessage.shoot_platform, predict_axis);
        Debug::SetMessage("装甲板预测", "旋转中心世界直角坐标", cv::Vec3f(currentAxisPoint));
        Debug::SetMessage("装甲板预测", "旋转中心世界极坐标", cv::Vec3f(axisPolar.distance, axisPolar.yaw, axisPolar.pitch));
        Debug::SetMessage("装甲板预测", "旋转中心反投影坐标", cv::Vec2d(projRotCenter));

        Debug::AddCircle("旋转中心", projRotCenter, 6, cv::Scalar(255, 255, 255));
        Debug::SetMessage("装甲板预测", "旋转中心跟随预测点直角坐标", cv::Vec3f(predict_axis));
        Debug::AddCircle("旋转中心预测点", projPreRotCenter, 6, cv::Scalar(0, 0, 255));

        Debug::AddPoint("小陀螺模式轴心.x", "实际.x", Point2f(Time::GetTimeNow() / 1000000.0f, projRotCenter.x));
        Debug::AddPoint("小陀螺模式轴心.y", "实际.y", Point2f(Time::GetTimeNow() / 1000000.0f, projRotCenter.y));
        Debug::AddPoint("小陀螺模式轴心.x", "预测.x", Point2f(Time::GetTimeNow() / 1000000.0f + predictTime_rotate, projPreRotCenter.x)); //
        Debug::AddPoint("小陀螺模式轴心.y", "预测.y", Point2f(Time::GetTimeNow() / 1000000.0f + predictTime_rotate, projPreRotCenter.y));
#endif

        DistanceCompensation(predict_axis);
#ifdef VIDEO_DEBUG
        cv::Point2d projPreComRotCenter = armorResolver->WorldToPixel(receiveMessage.shoot_platform, predict_axis);
        Debug::SetMessage("装甲板预测", "旋转中心跟随预测点距离补偿点直角坐标", cv::Vec3f(predict_axis));
        Debug::AddCircle("旋转中心预测距离补偿点", projPreComRotCenter, 6, cv::Scalar(125, 125, 255));
#endif
        Polar3f orientedPolar = RectangularToPolar(predict_axis);
        float   x, y;
        y                   = currentAxisPoint.y;
        x                   = sqrt(currentAxisPoint.x * currentAxisPoint.x + currentAxisPoint.z * currentAxisPoint.z);
        cv::Vec2f exitAngle = ParabolaSolve(cv::Point2f(x, y), receiveMessage.bulletspeed);
        float     pitch     = fabs(exitAngle[0] - receiveMessage.shoot_platform.pitch) < fabs(exitAngle[1] - receiveMessage.shoot_platform.pitch) ? exitAngle[0] : exitAngle[1];

        // if(orientedPolar.distance < 500 && orientedPolar.distance > 250)
        //     pitch += 0.005;
        // else if(orientedPolar.distance <= 250)
        //     pitch += 0.015;
        float yaw_offset = 0,pitch_offset = 0.0744;//TODO:小陀螺模式偏置
        LoadParam::ReadTheParam("Gyro_Yaw_offset",yaw_offset);
        LoadParam::ReadTheParam("Gyro_Pitch_offset",pitch_offset);
        orientedPolar.yaw += yaw_offset;
        pitch = pitch + pitch_offset - 0.038 * log(receiveMessage.bulletspeed / 100);

        bool   flag         = false;
        float  deltaX       = tan(orientedPolar.yaw - receiveMessage.shoot_platform.yaw) * x;
        float  deltaY       = tan(pitch - receiveMessage.shoot_platform.pitch) * x;
        Robot &currentRobot = *(history_.end() - 1);
        Robot &lastRobot    = (history_.size() > 1) ? (*(history_.end() - 2)) : (*(history_.end() - 1));
        int    type         = (int)currentRobot.GetRobotType();

        if (orientedPolar.distance < 270)
            flag = fabs(deltaX) < 11 && fabs(deltaY) < 7;
        else if (orientedPolar.distance >= 270 && orientedPolar.distance < 350)
            flag = fabs(deltaX) < 13 && fabs(deltaY) < 8;
        else
            flag = fabs(deltaX) < 17 && fabs(deltaY) < 15;

        sendStruct.pitch = (pitch) / CV_PI * 180.0f;
        sendStruct.yaw   = (orientedPolar.yaw) / CV_PI * 180.0f;
        //过圈处理
        float recv_yaw         = receiveMessage.shoot_platform.yaw / CV_PI * 180;
        float platformPosition = recv_yaw - (int)(recv_yaw / 360.0f) * 360.0f;                       //电控云台位置
        float visionPosition   = platformPosition > 180 ? platformPosition - 360 : platformPosition; //电控位置在视觉坐标系位置
        float diff             = sendStruct.yaw - visionPosition;
        diff                   = diff > 180 ? diff - 360 : diff < -180 ? diff + 360 : diff;
        sendStruct.yaw         = recv_yaw + diff;

        // sendStruct.pitch = (pitch) / CV_PI * 180.0f;
        // sendStruct.yaw   = (orientedPolar.yaw - 0.01f) / CV_PI * 180.0f;
        // if (receiveMessage.shoot_platform.yaw > 0)
        //     sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) + 0.5)) * 360.0f;
        // else if (receiveMessage.shoot_platform.yaw < 0)
        //     sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) - 0.5)) * 360.0f;

        float oppositeStamp = -1.0f;
        int   tagToRadius   = 0;
        for (int i = history_.size() - 1; i > -1; --i) {
            float yawDiff = fabs(axisPolar.yaw - history_[i].GetResolvedArmors()[0].GetPolar().yaw);
            if (fabs(yawDiff * history_[i].GetResolvedArmors()[0].GetPolar().distance) < 1.5) {
                tagToRadius   = tag_to_radius_index_[history_[i].GetResolvedArmors()[0].GetTag()];
                oppositeStamp = history_[i].GetTimeStamp();
                break;
            }
        }
        if (oppositeStamp == -1.0f)
            return;

            // if (tagToRadius == 0)
            //     sendStruct.pitch += 0.02f;
            // else if (tagToRadius == 1)
            //     sendStruct.pitch += 0.01f;
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "XTL:5 tagToRadius = " << tagToRadius << std::endl;
#endif
        double predictTime = 0.0f;
        double spinSpeed   = CalcVelocity(); // 角度每秒，顺正逆负
        if (spinSpeed > 0)
            predictTime = (orientedPolar.distance / receiveMessage.shoot_platform.bulletspeed + fire_delay_) * 1000000; //单位: us
        else if (spinSpeed < 0)
            predictTime = (orientedPolar.distance / receiveMessage.shoot_platform.bulletspeed + fire_delay_) * 1000000; //单位: us

        double quarter  = fabs(90 / spinSpeed) * 1000000;
        int    k        = (double(Time::GetTimeNow() - oppositeStamp) + predictTime + 0.5 * quarter) / quarter;
        double interval = double(Time::GetTimeNow() - oppositeStamp) + predictTime - k * quarter;
        if (flag) {
            for (int i = 0; i < 4; i++) {//TODO 小陀螺开多发的话时间范围应该调大点
                if (interval - i * quarter >= -5000 && interval - i * quarter < 5000) //不仅仅预测是否能打到下一个装甲板,而是预测接下来的四个装甲板
                {
                    sendStruct.beat = true;
                    double FireTime = GyroFireTime();
                    sendStruct.unLimitedFireTime = FireTime;
                    break;
                }
            }
        }
        isFire_ = sendStruct.beat;
#ifdef VIDEO_DEBUG
        if (isFire_) {
            Debug::AddCircle("旋转中心预测距离补偿点", projPreComRotCenter, 6, cv::Scalar(0, 255, 0));
            Debug::AddRotatedRect("开火高亮", RotatedRect(projPreComRotCenter, Size2f(20, 20), 45), cv::Scalar(0, 255, 0));
        }
#endif
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "XTL:0 TimeNow = " << Time::GetTimeNow() << ", oppositeStamp = " << oppositeStamp << ", predictTime = " << predictTime << ", quarter = " << quarter << std::endl;
        std::cout << "XTL:1 former = " << float(Time::GetTimeNow() - oppositeStamp) + predictTime << ", latter = " << quarter << std::endl;
        std::cout << "XTL:2 interval = " << interval << std::endl;
        std::cout << "XTL:3 flag = " << bool(flag) << std::endl;
        std::cout << "XTL:4 deltaX = " << deltaX << ", deltaY = " << deltaY << std::endl;
#endif
    }

    void RobotPredictor::SwingModeShoot(const tdttoolkit::ReceiveMessage &receiveMessage, tdtusart::Send_Struct_t &sendStruct) {
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "摇摆模式" << std::endl;
#endif
        double spinSpeed = CalcOnTimeVelocity();
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "摇摆模式实时转速" << spinSpeed << std::endl;
#endif
        sendStruct.beat = 0;
        if (axis_point_.size() < 2) {
            sendStruct.beat  = false;
            sendStruct.pitch = receiveMessage.shoot_platform.pitch / CV_PI * 180; //当前角度
            sendStruct.yaw   = receiveMessage.shoot_platform.yaw / CV_PI * 180;   //当前角度
            return;
        }

        cv::Point3f currentAxisPoint = ShockAbsorber_liner();
        if (RectangularToPolar(currentAxisPoint).distance > 1000) {
            sendStruct.beat  = false;
            sendStruct.pitch = receiveMessage.shoot_platform.pitch / CV_PI * 180; //当前角度
            sendStruct.yaw   = receiveMessage.shoot_platform.yaw / CV_PI * 180;   //当前角度
            return;
        }
        DistanceCompensation(currentAxisPoint);
        Polar3f axisPolar = RectangularToPolar(currentAxisPoint);
#ifdef VIDEO_DEBUG
        cv::Point2d projRotCenter = armorResolver->WorldToPixel(receiveMessage.shoot_platform, currentAxisPoint);
        Debug::SetMessage("装甲板预测", "旋转中心世界直角坐标", cv::Vec3f(currentAxisPoint));
        Debug::SetMessage("装甲板预测", "旋转中心世界极坐标", cv::Vec3f(axisPolar.distance, axisPolar.yaw, axisPolar.pitch));
        Debug::SetMessage("装甲板预测", "旋转中心反投影坐标", cv::Vec2d(projRotCenter));
        Debug::AddCircle("旋转中心", projRotCenter, 6, cv::Scalar(0, 255, 255));
#endif
        float      statusplus        = GetStatusPlus();
        static int status2_shoot_tag = -1;
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "StatusPlus=" << statusplus << std::endl;
#endif
        if (statusplus > 4.15f && statusplus < 4.25f) { //偶数装甲板摇摆
            if (status2_shoot_tag == -1)
                status2_shoot_tag = (int)GetStatusPlus(2);
            if (status2_shoot_tag == -1) {
                sendStruct.beat  = false;
                sendStruct.pitch = receiveMessage.shoot_platform.pitch / CV_PI * 180; //当前角度
                sendStruct.yaw   = receiveMessage.shoot_platform.yaw / CV_PI * 180;   //当前角度
                return;
            }
            if (!history_.back().GetArmorStatus(status2_shoot_tag)) {
                sendStruct.beat  = false;
                sendStruct.pitch = receiveMessage.shoot_platform.pitch / CV_PI * 180; //当前角度
                sendStruct.yaw   = receiveMessage.shoot_platform.yaw / CV_PI * 180;   //当前角度
                return;
            }
            ResolvedArmor target;
            for (int i = 0; i < history_.back().GetResolvedArmors().size(); i++) {
                if (history_.back().GetResolvedArmors()[i].GetTag() == status2_shoot_tag)
                    target = history_.back().GetResolvedArmors()[i];
            }
            static Polar3f shootplace = target.GetPolar();
            static bool    down       = (armor_angle_history_[status2_shoot_tag].back() - *(armor_angle_history_[status2_shoot_tag].end() - 2)) < 0; //目标装甲板的法线角度是否在减小
            static double  time_diff  = 0;                                                                                                           //一个周期中目标装甲板射击点出现时距离周期起点的时间
            if (down && (armor_angle_history_[status2_shoot_tag].back() - *(armor_angle_history_[status2_shoot_tag].end() - 2)) > 0) {
                shootplace = target.GetPolar();
                time_diff  = history_.back().GetTimeStamp() - shock_timeStamp_history_.back();
            }
            down                = (armor_angle_history_[status2_shoot_tag].back() - *(armor_angle_history_[status2_shoot_tag].end() - 2)) < 0;
            cv::Vec2f exitAngle = ParabolaSolve(cv::Point2f(shootplace.distance, currentAxisPoint.y), receiveMessage.bulletspeed);
            float     pitch     = fabs(exitAngle[0] - receiveMessage.shoot_platform.pitch) < fabs(exitAngle[1] - receiveMessage.shoot_platform.pitch) ? exitAngle[0] : exitAngle[1];
            float     deltaX    = tan(shootplace.yaw - receiveMessage.shoot_platform.yaw) * shootplace.distance;
            float     deltaY    = tan(pitch - receiveMessage.shoot_platform.pitch) * shootplace.distance;
            bool      flag      = 0; //枪口是否靠近射击点
            if (shootplace.distance < 270)
                flag = fabs(deltaX) < 11 && fabs(deltaY) < 7;
            else if (shootplace.distance >= 270 && shootplace.distance < 350)
                flag = fabs(deltaX) < 13 && fabs(deltaY) < 8;
            else
                flag = fabs(deltaX) < 17 && fabs(deltaY) < 15;
            sendStruct.pitch = (pitch) / CV_PI * 180.0f;
            sendStruct.yaw   = (shootplace.yaw - 0.01f) / CV_PI * 180.0f;
            if (receiveMessage.shoot_platform.yaw > 0)
                sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) + 0.5)) * 360.0f;
            else if (receiveMessage.shoot_platform.yaw < 0)
                sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) - 0.5)) * 360.0f;
            double predictTime = 0;
            if (spinSpeed > 0)
                predictTime = (shootplace.distance / receiveMessage.shoot_platform.bulletspeed + 0.13) * 1000000; //单位: us
            else if (spinSpeed < 0)
                predictTime = (shootplace.distance / receiveMessage.shoot_platform.bulletspeed + 0.10) * 1000000; //单位: us
            double cycle = shock_timeStamp_history_.back() - *(shock_timeStamp_history_.end() - 2);
            int    delay = (double)(history_.back().GetTimeStamp() + predictTime - shock_timeStamp_history_.back() + cycle / 2) / cycle;
            double diff  = delay % 2 ? history_.back().GetTimeStamp() + predictTime - shock_timeStamp_history_.back() - time_diff - delay * cycle : history_.back().GetTimeStamp() + predictTime - shock_timeStamp_history_.back() - (cycle - time_diff) - delay * cycle;
            if (diff < 10000 && diff > -44000) {
                sendStruct.beat = flag;
            }
            isFire_ = sendStruct.beat;
#ifdef VIDEO_DEBUG
            if (isFire_) {
                Debug::AddCircle("旋转中心", projRotCenter, 6, cv::Scalar(0, 255, 0));
                Debug::AddRotatedRect("开火高亮", RotatedRect(projRotCenter, Size2f(20, 20), 45), cv::Scalar(0, 255, 0));
            }
#endif
#ifdef PREDICT_DEBUG_OUTPUT
            std::cout << "XTL:0 TimeNow = " << Time::GetTimeNow() << ", predictTime = " << predictTime << std::endl;
            std::cout << "XTL:1 Cycle Time Deviation = " << time_diff << std::endl;
            std::cout << "XTL:2 timeDiff = " << history_.back().GetTimeStamp() + predictTime - shock_timeStamp_history_.back() - delay * time_diff << std::endl;
            std::cout << "XTL:3 flag = " << bool(flag) << std::endl;
            std::cout << "XTL:4 deltaX = " << deltaX << ", deltaY = " << deltaY << std::endl;
#endif
        } else { //清空偶数装甲板数摇摆状态
            status2_shoot_tag = -1;
        }
        if (statusplus > 3.95f && statusplus < 4.05f) { //未知摇摆
            FollowModeShoot(receiveMessage, sendStruct);
        }
        if ((statusplus > 4.05f && statusplus < 4.15f) || (statusplus > 4.25f && statusplus < 4.35f)) { //奇数装甲板数摇摆

            static double              time_diff; //一个周期中目标装甲板出现时距离周期起点的时间
            int                        target_tag     = (int)GetStatusPlus(1);
            int                        id_to_tag      = -1;
            std::vector<ResolvedArmor> resolvedArmors = history_.back().GetResolvedArmors();
            for (int i = 0; i < resolvedArmors.size(); i++) {
                if (resolvedArmors[i].GetTag() == target_tag)
                    id_to_tag = i;
            }
            if (id_to_tag == -1) {
                sendStruct.beat  = false;
                sendStruct.pitch = receiveMessage.shoot_platform.pitch / CV_PI * 180; //当前角度
                sendStruct.yaw   = receiveMessage.shoot_platform.yaw / CV_PI * 180;   //当前角度
                return;
            }
            float         yawdiff    = fabs(resolvedArmors[id_to_tag].GetPolar().yaw - axisPolar.yaw);
            static bool   enter_oppo = 0;
            static double per_time   = 0;
            static int    times      = 0;
            if (yawdiff * resolvedArmors[id_to_tag].GetPolar().distance < 1.5) {
                enter_oppo = 1;
                per_time += history_.back().GetTimeStamp();
                times++;
            } else if (enter_oppo) {
                per_time /= times;
                time_diff  = per_time - shock_timeStamp_history_.back();
                per_time   = 0;
                times      = 0;
                enter_oppo = 0;
            }
            cv::Vec2f exitAngle = ParabolaSolve(cv::Point2f(axisPolar.distance, currentAxisPoint.y), receiveMessage.bulletspeed);
            float     pitch     = fabs(exitAngle[0] - receiveMessage.shoot_platform.pitch) < fabs(exitAngle[1] - receiveMessage.shoot_platform.pitch) ? exitAngle[0] : exitAngle[1];
            float     deltaX    = tan(axisPolar.yaw - receiveMessage.shoot_platform.yaw) * axisPolar.distance;
            float     deltaY    = tan(pitch - receiveMessage.shoot_platform.pitch) * axisPolar.distance;
            bool      flag      = 0; //枪口是否靠近射击点
            if (axisPolar.distance < 270)
                flag = fabs(deltaX) < 11 && fabs(deltaY) < 7;
            else if (axisPolar.distance >= 270 && axisPolar.distance < 350)
                flag = fabs(deltaX) < 13 && fabs(deltaY) < 8;
            else
                flag = fabs(deltaX) < 17 && fabs(deltaY) < 15;
            sendStruct.pitch = (pitch) / CV_PI * 180.0f;
            sendStruct.yaw   = (axisPolar.yaw - 0.01f) / CV_PI * 180.0f;
            if (receiveMessage.shoot_platform.yaw > 0)
                sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) + 0.5)) * 360.0f;
            else if (receiveMessage.shoot_platform.yaw < 0)
                sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) - 0.5)) * 360.0f;
            double predictTime = 0;
            if (spinSpeed > 0)
                predictTime = (axisPolar.distance / receiveMessage.shoot_platform.bulletspeed + 0.13) * 1000000; //单位: us
            else if (spinSpeed < 0)
                predictTime = (axisPolar.distance / receiveMessage.shoot_platform.bulletspeed + 0.10) * 1000000; //单位: us
            double cycle = shock_timeStamp_history_.back() - *(shock_timeStamp_history_.end() - 2);
            int    delay = (double)(history_.back().GetTimeStamp() + predictTime - shock_timeStamp_history_.back() + cycle / 2) / cycle;
            double diff  = delay % 2 ? history_.back().GetTimeStamp() + predictTime - shock_timeStamp_history_.back() - time_diff - delay * cycle : history_.back().GetTimeStamp() + predictTime - shock_timeStamp_history_.back() - (cycle - time_diff) - delay * cycle;
            if (diff < 10000 && diff > -44000) {
                sendStruct.beat = flag;
            }
            isFire_ = sendStruct.beat;
#ifdef VIDEO_DEBUG
            if (isFire_) {
                Debug::AddCircle("旋转中心", projRotCenter, 6, cv::Scalar(0, 255, 0));
                Debug::AddRotatedRect("开火高亮", RotatedRect(projRotCenter, Size2f(20, 20), 45), cv::Scalar(0, 255, 0));
            }
#endif
            // Debug::AddRect("旋转中心",Rect())
#ifdef PREDICT_DEBUG_OUTPUT
            std::cout << "XTL:0 TimeNow = " << Time::GetTimeNow() << ", predictTime = " << predictTime << std::endl;
            std::cout << "XTL:1 Cycle Time Deviation = " << time_diff << std::endl;
            std::cout << "XTL:2 timeDiff = " << history_.back().GetTimeStamp() + predictTime - shock_timeStamp_history_.back() - delay * time_diff << std::endl;
            std::cout << "XTL:3 flag = " << bool(flag) << std::endl;
            std::cout << "XTL:4 deltaX = " << deltaX << ", deltaY = " << deltaY << std::endl;
#endif
        }
        if (statusplus > 4.35f && statusplus < 4.45f) { //旋转一周摇摆
            GyroModeShoot(receiveMessage, sendStruct);
            return;
        }
        // orientedPolar       = axisPolar;
        // cv::Vec2f exitAngle = ParabolaSolve(cv::Point2f(orientedPolar.distance, currentAxisPoint.y), receiveMessage.bulletspeed);
        // float     pitch     = fabs(exitAngle[0] - receiveMessage.shoot_platform.pitch) < fabs(exitAngle[1] - receiveMessage.shoot_platform.pitch) ? exitAngle[0] : exitAngle[1];
        // float     deltaX    = tan(orientedPolar.yaw - receiveMessage.shoot_platform.yaw) * orientedPolar.distance;
        // float     deltaY    = tan(pitch - receiveMessage.shoot_platform.pitch) * orientedPolar.distance;
        // bool      flag      = 0; //枪口是否靠近射击点
        // if (orientedPolar.distance < 270)
        //     flag = fabs(deltaX) < 11 && fabs(deltaY) < 7;
        // else if (orientedPolar.distance >= 270 && orientedPolar.distance < 350)
        //     flag = fabs(deltaX) < 13 && fabs(deltaY) < 8;
        // else
        //     flag = fabs(deltaX) < 17 && fabs(deltaY) < 15;
        // sendStruct.pitch = (pitch) / CV_PI * 180.0f;
        // sendStruct.yaw   = (orientedPolar.yaw - 0.01f) / CV_PI * 180.0f;
        // if (receiveMessage.shoot_platform.yaw > 0)
        //     sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) + 0.5)) * 360.0f;
        // else if (receiveMessage.shoot_platform.yaw < 0)
        //     sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) - 0.5)) * 360.0f;
        // double predictTime = 0;
        // if (spinSpeed > 0)
        //     predictTime = (orientedPolar.distance / receiveMessage.shoot_platform.bulletspeed + 0.13) * 1000000; //单位: us
        // else if (spinSpeed < 0)
        //     predictTime = (orientedPolar.distance / receiveMessage.shoot_platform.bulletspeed + 0.10) * 1000000; //单位: us
        // double shootTime = shock_timeStamp_history_.back() + (*(shock_timeStamp_history_.end() - 1) - *(shock_timeStamp_history_.end() - 2)) / 2;
        // double timeDiff  = Time::GetTimeNow() + predictTime - shootTime;
        // sendStruct.beat  = (timeDiff >= -44000 && timeDiff < 10000) ? (flag ? true : false) : false;
        // isFire_          = sendStruct.beat;
        // std::cout << "XTL:0 TimeNow = " << Time::GetTimeNow() << ", predictTime = " << predictTime << std::endl;
        // std::cout << "XTL:2 timeDiff = " << timeDiff << std::endl;
        // std::cout << "XTL:3 flag = " << bool(flag) << std::endl;
        // std::cout << "XTL:4 deltaX = " << deltaX << ", deltaY = " << deltaY << std::endl;
    }

    void RobotPredictor::SwingModeShoot2(const tdttoolkit::ReceiveMessage &receiveMessage, tdtusart::Send_Struct_t &sendStruct) {
        Robot &currentRobot = *(history_.end() - 1);
        Robot &lastRobot    = (history_.size() > 1) ? (*(history_.end() - 2)) : (*(history_.end() - 1));
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "Status: 摇摆模式" << std::endl;
#endif

        static tdttoolkit::ShootPlatform      lastPlatform = receiveMessage.shoot_platform;
        static deque<std::pair<float, float>> pfHistory;

        float timeGap = float(currentRobot.GetTimeStamp() - lastRobot.GetTimeStamp()) / 1000000.0f;
        timeGap       = timeGap == 0 ? 1 : timeGap;
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "timeGap: " << timeGap << " s" << std::endl;
        std::cout << "yawGap = " << receiveMessage.shoot_platform.yaw - lastPlatform.yaw << " rad" << std::endl;
#endif
        float pfYawSpeed   = (receiveMessage.shoot_platform.yaw - lastPlatform.yaw) / timeGap;
        float pfPitchSpeed = (receiveMessage.shoot_platform.pitch - lastPlatform.pitch) / timeGap;
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "GSMS: pfYawSpeed = " << pfYawSpeed << std::endl;
#endif
        pfHistory.emplace_back(std::make_pair(pfYawSpeed, pfPitchSpeed));
        if (pfHistory.size() > 5)
            pfHistory.pop_front();

        float pfYawAcc = 0.0f, pfPitchAcc = 0.0f;

        if (pfHistory.size() > 1)
            for (int i = 0; i < pfHistory.size() - 1; ++i) {
                pfYawAcc += pow(pfHistory[i + 1].first - pfHistory[i].first, 2);
                pfPitchAcc += pow(pfHistory[i + 1].second - pfHistory[i].second, 2);
            }

        float qY, rY, qP, rP;
        qY = GetKalmanProcessNoise();
        qP = qY / kalman_fps; /// 调车时感觉qP与qY相同时小车竖直有变化是相对水平容易抖，所以调小了一点，但还待调整

#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝" << endl;
        std::cout << "摇摆模式过程噪声:" << endl;
        std::cout << "qY:" << qY << " qP:" << qP << endl;
        std::cout << "注意：看qY qP时别看最后一次输出，从倒数第二帧看起，神秘原因最后一帧帧率会很低，所以qY qP会偏大" << endl;
        std::cout << "＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝＝" << endl;
#endif
        rY = 0.02 * (currentRobot.GetTimeStamp() - lastRobot.GetTimeStamp()) / 1000.0f;
        rP = rY;

#ifdef VIDEO_DEBUG
        Debug::SetMessage("卡尔曼滤波", "跟随模式Q　&　R", cv::Vec2f(qY, rY));
#endif
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "yawQandR: " << pfYawSpeed / CV_PI * 180 << ", " << pfYawAcc << ", " << qY << ", " << rY << std::endl;
#endif
        lastPlatform              = receiveMessage.shoot_platform;
        const auto &currentArmors = currentRobot.GetResolvedArmors();
        const auto &lastArmors    = lastRobot.GetResolvedArmors();
        for (int i = 0; i < currentArmors.size(); ++i) {
            int  j, tag = currentArmors[i].GetTag();
            bool flag = false;
            for (int n = 0; n < lastArmors.size(); ++n)
                if (lastArmors[n].GetTag() == tag)
                    j = n, flag = true; // 两帧之间有连接成功的装甲板，编号为tag

            if (flag) {
                Polar3f currentPolar = currentArmors[i].GetPolar();
                Polar3f lastPolar    = lastArmors[j].GetPolar();
                float   radius = -1, yawDiff = CalcAngleDifference(currentPolar.yaw, lastPolar.yaw);
                float   pitchDiff      = CalcAngleDifference(currentPolar.pitch, lastPolar.pitch);
                float   yawLinearSpeed = yawDiff * (currentPolar.distance * 10) / ((currentRobot.GetTimeStamp() - lastRobot.GetTimeStamp()) / 1000.0f);   //单位 m/s
                float   pitchSpeed     = pitchDiff * (currentPolar.distance * 10) / ((currentRobot.GetTimeStamp() - lastRobot.GetTimeStamp()) / 1000.0f); //单位 m/s
                isTaged[tag]           = true;

                yawLinearFilter_[tag].SetQ(qY);
                yawLinearFilter_[tag].SetR(rY);
                pitchLinearFilter_[tag].SetQ(qP);
                pitchLinearFilter_[tag].SetR(rP);
#ifdef PREDICT_DEBUG_OUTPUT
                std::cout << "GSMS:1 yawDiff = " << yawDiff << ", distance = " << currentPolar.distance << ", yawLinearSpeed = " << yawLinearSpeed << ", timeGap = " << (currentRobot.GetTimeStamp() - lastRobot.GetTimeStamp()) / 1000000.0f << std::endl;
#endif
                if (tag_to_radius_index_[tag] == 0)
                    radius = robotRadius[currentRobot.GetRobotType()][0].GetRadius();
                else if (tag_to_radius_index_[tag] == 1)
                    radius = robotRadius[currentRobot.GetRobotType()][1].GetRadius();
                else
                    radius = 25;
#ifdef PREDICT_DEBUG_OUTPUT
                std::cout << "GSMS:2 yawFilter[" << tag << "].statePre = " << yawLinearFilter_[tag].statePre.at<float>(0) << ", measurement = " << yawLinearSpeed << ", statePost = " << yawLinearFilter_[tag].statePost.at<float>(0) << std::endl;
#endif
                float yawStatePost = history_.size() == 1 ? yawLinearFilter_[tag].Estimate(yawLinearSpeed, false) : yawLinearFilter_[tag].Estimate(yawLinearSpeed, true); // 单位 m/s
#ifdef VIDEO_DEBUG
                Debug::SetMessage("装甲板预测", "yaw线速度", yawStatePost);
#endif
                float pitchStatePost = history_.size() == 1 ? pitchLinearFilter_[tag].Estimate(pitchSpeed, false) : pitchLinearFilter_[tag].Estimate(pitchSpeed, true); // 单位 m/s
                currentPolar_[tag]   = currentPolar;
                /// 100是从cm / (m/s) -> m / (m/s)所需的转换倍数　fire_delay_是开火延迟（能否实时更新？）
                /// 对视频进行观看时装甲板预测与实际不准主要是因为fire_delay_的开火延迟（预测时间如果过久会导致预测不准，勉强可以理解）
                double predictTime = (currentPolar.distance / (receiveMessage.shoot_platform.bulletspeed) + fire_delay_); // 单位 s

                double swing_time = 2 * shock_timeStamp_history_.back() - *(shock_timeStamp_history_.end() - 2); //摇摆模式预测转向时间
                while (swing_time < history_.back().GetTimeStamp())
                    swing_time += shock_timeStamp_history_.back() - *(shock_timeStamp_history_.end() - 2);
                //进行摇摆模式时间补偿
                swing_time /= 1000000.0f;
                double predictTimeCompensate = predictTime;
                bool   f                     = 1;
                while (history_.back().GetTimeStamp() / 1000000.0f + predictTimeCompensate > swing_time) {
                    predictTimeCompensate = predictTimeCompensate + f * (-1) * 2 * (history_.back().GetTimeStamp() / 1000000.0f + predictTimeCompensate - swing_time);
                    f                     = !f;
                }

                Polar3f predictPolar(currentPolar.distance, currentPolar.yaw + predictTimeCompensate * yawStatePost * 100 / currentPolar.distance, currentPolar.pitch + predictTime * pitchStatePost * 100 / currentPolar.distance);

#ifdef VIDEO_DEBUG
                cv::Point3f currentPoint     = PolarToRectangular(currentPolar);
                cv::Point3f predictPoint     = PolarToRectangular(predictPolar);
                cv::Point2d currentProjPoint = armorResolver->WorldToPixel(receiveMessage.shoot_platform, currentPoint);
                cv::Point2d predictProjPoint = armorResolver->WorldToPixel(receiveMessage.shoot_platform, predictPoint);
                Debug::AddPoint("跟随模式装甲板中心.x", "实际.x", Point2f(Time::GetTimeNow() / 1000000.0f, currentProjPoint.x));
                Debug::AddPoint("跟随模式装甲板中心.y", "实际.y", Point2f(Time::GetTimeNow() / 1000000.0f, currentProjPoint.y));
                Debug::AddPoint("跟随模式装甲板中心.x", "预测.x", Point2f(Time::GetTimeNow() / 1000000.0f + predictTime, predictProjPoint.x)); //+ predictTime
                Debug::AddPoint("跟随模式装甲板中心.y", "预测.y", Point2f(Time::GetTimeNow() / 1000000.0f + predictTime, predictProjPoint.y)); //
#endif
                predictPolar_[tag] = predictPolar;

            } else {
                Polar3f currentPolar = currentArmors[i].GetPolar();
                if (currentRobot.GetTimeStamp() / 1000.0f - yawLinearFilter_[tag].GetLastTime() / 1.0f < 50) {
                    Polar3f predictPolar = predictPolar_[tag];
                    float   predictTime  = (currentPolar.distance / (receiveMessage.bulletspeed) + fire_delay_);
                    predictPolar.yaw += predictTime * yawLinearFilter_[tag].statePost.at<float>(0) * 100 / currentPolar_[tag].distance;
                    predictPolar.pitch += predictTime * pitchLinearFilter_[tag].statePost.at<float>(0) * 100 / currentPolar_[tag].distance;
                    predictPolar_[tag] = predictPolar;
                } else {
                    yawLinearFilter_[tag].Estimate(0, false); // 置零
                    currentPolar_[tag]   = currentArmors[i].GetPolar();
                    float   predictTime  = (currentPolar.distance / (receiveMessage.bulletspeed) + fire_delay_);
                    Polar3f predictPolar = currentPolar_[tag];
                    predictPolar_[tag].yaw += predictTime * yawLinearFilter_[tag].statePost.at<float>(0) * 100 / currentPolar_[tag].distance;
                    predictPolar_[tag].pitch += predictTime * pitchLinearFilter_[tag].statePost.at<float>(0) * 100 / currentPolar_[tag].distance;
                }
            }
        }
        Polar3f orientedPolar;
        float   statusplus = GetStatusPlus();
        if (statusplus > 4.15f && statusplus < 4.25f) {
            static int    lastBeatTag = armorDecision(history_.back().GetResolvedArmors());
            static double lastTagTime = history_.back().GetTimeStamp();
            if (history_.back().GetArmorStatus(lastBeatTag)) {
                lastTagTime = history_.back().GetTimeStamp();
            } else if (history_.back().GetTimeStamp() - lastTagTime > 2 * 1000000.0f) {
                lastBeatTag = armorDecision(history_.back().GetResolvedArmors());
                lastTagTime = history_.back().GetTimeStamp();
            } else {
                return;
            }
            orientedPolar = predictPolar_[lastBeatTag];
        } else {
            int    beatTag   = currentArmors[armorDecision(currentArmors)].GetTag();
            double spinSpeed = CalcVelocity();
            if (fabs(spinSpeed) > 20) {
                if (spinSpeed > 0) // 顺时针旋转
                    beatTag = (currentArmors.size() == 2) ? currentArmors[1].GetTag() : currentArmors[0].GetTag();
                else
                    beatTag = currentArmors[0].GetTag();
            }
            if (beatTag != -1) {
                orientedPolar = predictPolar_[beatTag];
            } else {
                return;
            }
        }

        cv::Point3f orientedPoint = PolarToRectangular(orientedPolar);
#ifdef VIDEO_DEBUG
        cv::Point2d orientedProjPoint = armorResolver->WorldToPixel(receiveMessage.shoot_platform, orientedPoint);
        Debug::SetMessage("装甲板预测", "跟随预测点反投影坐标", cv::Vec2d(orientedProjPoint));
        Debug::SetMessage("装甲板预测", "跟随预测点世界坐标", cv::Vec3f(orientedPoint));
        Debug::SetMessage("装甲板预测", "跟随预测点极坐标", cv::Vec3f(orientedPolar.distance, orientedPolar.yaw, orientedPolar.pitch));
        Debug::AddCircle("跟随模式预测点", orientedProjPoint, 6, cv::Scalar(0, 0, 255));
#endif
        float x, y;
        y                   = orientedPoint.y;
        x                   = sqrt(orientedPoint.x * orientedPoint.x + orientedPoint.z * orientedPoint.z);
        cv::Vec2f exitAngle = ParabolaSolve(cv::Point2f(x, y), receiveMessage.bulletspeed);
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "exitAngle: " << x << ", " << y << ", " << receiveMessage.bulletspeed << ", " << exitAngle << std::endl;
#endif
        float pitch = fabs(exitAngle[0] - receiveMessage.shoot_platform.pitch) < fabs(exitAngle[1] - receiveMessage.shoot_platform.pitch) ? exitAngle[0] : exitAngle[1];

        if (orientedPolar.distance < 500 && orientedPolar.distance > 300)
            pitch -= 0.02;
        else if (orientedPolar.distance <= 300 && orientedPolar.distance > 150)
            pitch -= 0.01;

        sendStruct.pitch = (pitch) / CV_PI * 180.0f;
        sendStruct.yaw   = (orientedPolar.yaw) / CV_PI * 180.0f;
        if (receiveMessage.shoot_platform.yaw > 0)
            sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) + 0.5)) * 360.0f;
        else if (receiveMessage.shoot_platform.yaw < 0)
            sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) - 0.5)) * 360.0f;

#ifdef VIDEO_DEBUG
        cv::Point3f compPoint    = PolarToRectangular(Polar3f(orientedPolar.distance, orientedPolar.yaw, pitch));
        cv::Point2d compensation = armorResolver->WorldToPixel(receiveMessage.shoot_platform, compPoint);
        Debug::AddCircle("跟随模式预测点重力补偿", compensation, 6, cv::Scalar(125, 125, 255));
#endif

        float deltaX, deltaY;
        deltaX   = tan(orientedPolar.yaw - receiveMessage.shoot_platform.yaw) * x;
        deltaY   = tan(pitch - receiveMessage.shoot_platform.pitch) * x;
        int type = (int)currentRobot.GetRobotType();

        sendStruct.beat = fabs(deltaX) < 10 && fabs(deltaY) < 8;
        isFire_         = sendStruct.beat;
#ifdef VIDEO_DEBUG
        if (isFire_) {
            Debug::AddCircle("跟随模式预测点重力补偿", compensation, 6, cv::Scalar(0, 255, 0));
            Debug::AddRotatedRect("开火高亮", RotatedRect(compensation, Size2f(20, 20), 45), cv::Scalar(0, 255, 0));
        }
#endif
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "PYMS: deltaX = " << deltaX << ", deltaY = " << deltaY << std::endl;
        std::cout << "PYMS: beat = " << bool(sendStruct.beat) << std::endl;
#endif
    }

    void RobotPredictor::SpecialMode(const tdttoolkit::ReceiveMessage &receiveMessage, tdtusart::Send_Struct_t &sendStruct) {
        Robot &currentRobot = *(history_.end() - 1);
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "Status: 特殊模式" << std::endl;
#endif
        int    beatTag       = armorDecision(currentRobot.GetResolvedArmors());
        auto   currentArmors = currentRobot.GetResolvedArmors();
        double spinSpeed     = CalcVelocity();
        if (fabs(spinSpeed) > 20) {
            if (spinSpeed > 0) // 顺时针旋转
                beatTag = (currentArmors.size() == 2) ? currentArmors[1].GetTag() : currentArmors[0].GetTag();
            else
                beatTag = currentArmors[0].GetTag();
        }
        Polar3f     orientedPolar = currentRobot.GetResolvedArmors()[beatTag].GetPolar();
        cv::Point3f orientedPoint = PolarToRectangular(orientedPolar);
#ifdef VIDEO_DEBUG
        cv::Point2d orientedProjPoint = armorResolver->WorldToPixel(receiveMessage.shoot_platform, orientedPoint);
        Debug::SetMessage("装甲板预测", "跟随预测点反投影坐标", cv::Vec2d(orientedProjPoint));
        Debug::SetMessage("装甲板预测", "跟随预测点世界坐标", cv::Vec3f(orientedPoint));
        Debug::SetMessage("装甲板预测", "跟随预测点极坐标", cv::Vec3f(orientedPolar.distance, orientedPolar.yaw, orientedPolar.pitch));
        Debug::AddCircle("跟随模式预测点", orientedProjPoint, 12, cv::Scalar(0, 0, 255));
#endif
        float deltaX, deltaY;
        float x  = orientedPolar.distance;
        deltaX   = tan(orientedPolar.yaw - receiveMessage.shoot_platform.yaw) * x;
        deltaY   = tan(orientedPolar.pitch - receiveMessage.shoot_platform.pitch) * x;
        int type = (int)currentRobot.GetRobotType();

        sendStruct.pitch = (orientedPolar.pitch) / CV_PI * 180.0f;
        sendStruct.yaw   = (orientedPolar.yaw) / CV_PI * 180.0f;
        if (receiveMessage.shoot_platform.yaw > 0)
            sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) + 0.5)) * 360.0f;
        else if (receiveMessage.shoot_platform.yaw < 0)
            sendStruct.yaw += ((int)(receiveMessage.shoot_platform.yaw / (2 * CV_PI) - 0.5)) * 360.0f;

        sendStruct.beat = fabs(deltaX) < 10 && fabs(deltaY) < 8;
        isFire_         = sendStruct.beat;
#ifdef PREDICT_DEBUG_OUTPUT
        std::cout << "PYMS: deltaX = " << deltaX << ", deltaY = " << deltaY << std::endl;
        std::cout << "PYMS: beat = " << bool(sendStruct.beat) << std::endl;
#endif
    }

    void RobotPredictor::FireCommand(const ReceiveMessage &receiveMessage, tdtusart::Send_Struct_t &sendStruct) {
        gyroCommand      = receiveMessage.gyro_command;
        sendStruct.beat  = false;
        sendStruct.pitch = receiveMessage.shoot_platform.pitch / CV_PI * 180; //当前角度
        sendStruct.yaw   = receiveMessage.shoot_platform.yaw / CV_PI * 180;   //当前角度
        CalcArmorsTag();
        MeasureAxisYaw(receiveMessage);
        double spinSpeed = CalcVelocity(); // 角度每秒，顺正逆负

        // std::cout << "spinSpeed = " << spinSpeed << "°" << std::endl;
#ifdef VIDEO_DEBUG
        Debug::SetMessage("装甲板预测", "小陀螺转速", double(spinSpeed)); // 角度
#endif

        Robot &currentRobot = *(history_.end() - 1);
        Robot &lastRobot    = (history_.size() > 1) ? (*(history_.end() - 2)) : (*(history_.end() - 1));
        int    res          = 0;
        for (int i = 0; i < 4; ++i)
            res += (currentRobot.GetArmorStatus(i) + lastRobot.GetArmorStatus(i));
        if (res < 2)
            return;

        const auto &currentArmors = currentRobot.GetResolvedArmors();
        const auto &lastArmors    = lastRobot.GetResolvedArmors();

#ifdef VIDEO_DEBUG
        for (auto &armor : currentArmors) {
            if (armor.GetTag() == -1) {
                continue;
            }
            using namespace std::string_literals;
            std::string tagString = to_string(armor.GetTag());
            std::string str       = "装甲板Tag"s + tagString;
            Debug::AddText(str, tagString, (armor.GetStickerRect().GetVertices2f()[1] + armor.GetStickerRect().GetVertices2f()[2]) / 2, cv::Scalar(255, 255, 255), 1, 3);
        }
#endif

        int status = GetStatus();
#ifdef PREDICT_DEBUG_OUTPUT
        cout << "status:" << status << endl;
#endif
#ifdef SPECIALMODE
        SpecialMode(receiveMessage, sendStruct);
        return;
#endif
        if (status == 2 || status == 3) {
            GyroModeShoot(receiveMessage, sendStruct);
        } else if (status == 1 || status == 0) {
            FollowModeShoot(receiveMessage, sendStruct);
        } else if (status == 4 || status == 5) {
            SwingModeShoot2(receiveMessage, sendStruct);
        }

        auto FOV = armorResolver->GetFOV();
        if ((sendStruct.yaw / 180.0f * CV_PI - receiveMessage.shoot_platform.yaw) > (FOV.first / 2)) {
            sendStruct.yaw = (receiveMessage.shoot_platform.yaw + FOV.first / 2 - 0.05) / CV_PI * 180.0f;
        } else if ((receiveMessage.shoot_platform.yaw - sendStruct.yaw / 180.0f * CV_PI) > (FOV.first / 2)) {
            sendStruct.yaw = (receiveMessage.shoot_platform.yaw - FOV.first / 2 + 0.05) / CV_PI * 180.0f;
        }

    } // FireCommand

} // namespace tdtrobot