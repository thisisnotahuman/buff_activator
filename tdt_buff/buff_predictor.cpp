
#include"buff_predictor.h"
#include "buff_detector.h"
#include <fstream>
using namespace std;
using namespace cv;
using namespace tdttoolkit;
namespace tdtbuff {

    BuffPredictor::BuffPredictor()
    {
        predict_point_=cv::Point3f(-1,-1,-1);
    }

    void BuffPredictor::BuffPredict(ReceiveMessage &receive_message,tdtusart::Send_Struct_t &send_message) {



        /**
         *最后一个暂时不考虑，后续进行计算
         */
        JudgeDirection();
        JudgeType();

#ifdef VIDEO_DEBUG
        Debug::SetMessage("串口信息","接收pitch",receive_message.shoot_platform.pitch);
        Debug::SetMessage("串口信息","接收yaw",receive_message.shoot_platform.yaw);
        Debug::SetMessage("串口信息", "接收弹速",receive_message.bulletspeed);
        Debug::SetMessage("解算和预测","Clock",int(clockwise_));

        Debug::SetMessage("解算和预测","Buff_type",int(buff_type_));
#endif
        Predict (receive_message);
        BuffCommandBuild(buff_type_,receive_message,send_message);
#ifdef VIDEO_DEBUG
        Debug::SetMessage("串口信息","是否击打",send_message.beat);
        Debug::SetMessage("串口信息","发送pitch",send_message.pitch);
        Debug::SetMessage("串口信息","发送yaw",send_message.yaw);
#endif


    }


    void BuffPredictor::JudgeDirection()
    {
        if(!is_first_) return;
        int times=0;

        if(buff_speeds_.size()<5) return;
        for(int i=0;i<buff_speeds_.size();i++)
        {
            if(buff_speeds_[i]>0) times++;
        }
        if(times>3) clockwise_=true;
        else clockwise_ = false;
        is_first_=false;

    }
    void BuffPredictor::JudgeType() {
        if(buff_type_) return;
       int times = 0;
       if(buff_speeds_.size()<10) return;
      for(int i=0;i<buff_speeds_.size()-1;i++)
      {


       if(fabs(buff_speeds_[i])>1.8||fabs(buff_speeds_[i])<0.4) times+=2;//1.7  0.4

       //if(fabs(buff_speeds_[i+1]-buff_speeds_[i])>0.35) times++;
      }
      buff_type_ = times>9;
    }
    void BuffPredictor::SetNewTarget(Buff &target) {
        is_final_=target.GetFinal();
        if (history_targets_.empty()) {
            history_targets_.emplace_back(target);
            return;
        }
        ///判断是否为连续装甲板
        cv::Point2f tmp_new_target = target.GetBuffArmors()[0].GetBuffRect().GetCenter2f();
        cv::Point2f tmp_old_target = history_targets_.back().GetBuffArmors()[0].GetBuffRect().GetCenter2f();

        bool same_armor = tdttoolkit::CalcDistance(tmp_new_target, tmp_old_target) < 200;
        bool same_time =target.GetBuffArmors()[0].GetTimeNow() - history_targets_.back().GetBuffArmors()[0].GetTimeNow() <
                        100000;
        //时间和位置的连续
        if (same_armor && same_time) {
            history_targets_.emplace_back(target);
        }
        else {
            vector<Buff>{target}.swap(history_targets_);
        }
        if (history_targets_.size() > 50) {
            history_targets_.erase(history_targets_.begin());
        }


        static int  add_times = 0;
        if (history_targets_.size() > 20/*13*/) {
            if (interval_ == 0) {
                float angle_speed = history_targets_[history_targets_.size() - 1].GetBuffArmors()[0].GetAngle() -
                                    history_targets_[history_targets_.size() - 3].GetBuffArmors()[0].GetAngle();
                float angle_spd = history_targets_[history_targets_.size() - 1].GetBuffArmors()[0].GetAngle() -
                                  history_targets_[history_targets_.size() - 13].GetBuffArmors()[0].GetAngle();
                if (fabs(angle_speed) > CV_PI)
                    angle_speed = angle_speed + 2 * float(CV_PI) * (angle_speed < 0 ? 1.f : -1.f);
                if (fabs(angle_spd) > CV_PI)
                    angle_spd = angle_spd + 2 * float(CV_PI) * (angle_spd < 0 ? 1.f : -1.f);
                angle_speed = angle_speed * 1000000 /
                              (history_targets_[history_targets_.size() - 1].GetBuffArmors()[0].GetTimeNow() -
                               history_targets_[history_targets_.size() - 3].GetBuffArmors()[0].GetTimeNow());
                angle_spd = angle_spd * 1000000 /
                            (history_targets_[history_targets_.size() - 1].GetBuffArmors()[0].GetTimeNow() -
                             history_targets_[history_targets_.size() - 13].GetBuffArmors()[0].GetTimeNow());

                buff_speeds_.emplace_back(angle_speed);
                if (buff_speeds_.size() > 10) {
                    buff_speeds_.erase(buff_speeds_.begin());
                }
                cout<<"angle_speed: "<<angle_speed<<endl;//匀速1.2~2.6

//                KF=tdttoolkit::KalmanFilter(100, false);
//                KF.SetQ(0.0655);
//                KF.SetR(0.4);
//                float angle=KF.Estimate(angle_spd, false);
                float angle = angle_spd;

//                if(inter%20==0) {
//                    std::ofstream f1out;
//                    f1out.open("/home/tdt/Git/函数拟合/正弦拟合time", ios::app);
//                    f1out << history_targets_[history_targets_.size() - 1].GetBuffArmors()[0].GetTimeNow() / 1000000 << ", "; //将变量的值写入文件
//                    f1out.close();                                                                                            //关闭文件
//                    std::ofstream f2out;
//                    f2out.open("/home/tdt/Git/函数拟合/正弦拟合angle_speed", ios::app);
//                    f2out << angle << ", "; //将变量的值写入文件
//                    f2out.close();          //关闭文件
//                }
//                inter++;
                cout<<"angle: "<<angle<<endl;//匀速1.2~2.4
                float A=0.0,A_max,B=0.0,A_min;
                buff_angle_speed.emplace_back(angle);
                //predict_angle_ = (angle+buff_angle_speed[4])/2*0.4;

                if (buff_angle_speed.size() >10 /*90*/) {
                    buff_angle_speed.erase(buff_angle_speed.begin());
                }
                vector<float>speed=buff_angle_speed;
                sort(speed.begin(),speed.end());
                if(clockwise_){A = speed.back(),B = speed.front();}
                else {A=speed.front(), B = speed.back();}
                speed.clear();
                A_min_vector.emplace_back(fabs(B));
                sort(A_min_vector.begin(),A_min_vector.end());
                A_min = A_min_vector.front();
                A_max_vector.emplace_back(fabs(A));
                sort(A_max_vector.begin(),A_max_vector.end());
                A_max = A_max_vector.back()-A_min;
                if (A_max_vector.size() > 10) {
                    A_max_vector.erase(A_max_vector.begin());
                }
                if (A_min_vector.size() > 10) {
                    A_min_vector.erase(A_min_vector.begin());
                }
                float p=0.0;
                if(buff_angle_speed.size()>8&&history_targets_.size()>10) {
                    p = ((asin((fabs(angle) - A_min) / (A_max )) /
                          (history_targets_[history_targets_.size() - 4].GetBuffArmors()[0].GetTimeNow() / 1000000)) -
                         (asin((fabs(buff_angle_speed[6]) - A_min) / (A_max )) /
                          (history_targets_[history_targets_.size() - 8].GetBuffArmors()[0].GetTimeNow() / 1000000))) /
                        ((1.0 /
                          (history_targets_[history_targets_.size() - 4].GetBuffArmors()[0].GetTimeNow() / 1000000)) -
                         (1.0 /
                          (history_targets_[history_targets_.size() - 8].GetBuffArmors()[0].GetTimeNow() / 1000000)));
                }
                float w=(asin((fabs(angle)-A_min)/(A_max))-p)/(history_targets_[history_targets_.size()-7].GetBuffArmors()[0].GetTimeNow()/1000000);
                float degree1 = -(A_max)/w*cos(w*(history_targets_[history_targets_.size() - 1].GetBuffArmors()[0].GetTimeNow() / 1000000)+p)+(A_min*(history_targets_[history_targets_.size() - 1].GetBuffArmors()[0].GetTimeNow() / 1000000));
                float degree2 = -(A_max)/w*cos(w*(history_targets_[history_targets_.size() - 1].GetBuffArmors()[0].GetTimeNow() / 1000000+0.45)+p)+(A_min*(history_targets_[history_targets_.size() - 1].GetBuffArmors()[0].GetTimeNow() / 1000000+0.45));
                cout<<"w=Asin(wt+p)+o: "<<A_max<<"sin("<<w<<"t+("<<p<<"))"<<"+"<<A_min<<endl;
                if(!buff_speeds_.empty())
                {
                    if(fabs(angle_speed-buff_speeds_[buff_speeds_.size()-2])<0.3)
                        //return
                        ;
                }
                if(!buff_angle_speed.empty())
                {
                    if(fabs(angle-buff_angle_speed[buff_angle_speed.size()-7])<0.24)
                        //return
                            ;

                }
                //float angle_speed_next=(A_max-A_min)*sin(w*(history_targets_[history_targets_.size()-2].GetBuffArmors()[0].GetTimeNow()/1000000+0.35)+p)+A_min;
                if(clockwise_)predict_angle_=fabs(degree2-degree1);
                else predict_angle_=-fabs(degree2-degree1);
                std::cout<<"拟合"<<history_targets_[history_targets_.size()-2].GetBuffArmors()[0].GetTimeNow()/1000000<<std::endl;
                std::cout<<"拟合2"<<angle_speed<<std::endl;

                //predict_angle_ = Script::Fit(history_targets_[history_targets_.size()-7].GetBuffArmors()[0].GetTimeNow()/1000000,angle_speed,0.50);
//                if(clockwise_)predict_angle_=fabs(degree2-degree1);
//                else predict_angle_=-fabs(degree2-degree1);
                if(clockwise_){predict_angle_=fabs(predict_angle_);}
                else{predict_angle_=-fabs(predict_angle_);}
//                float The_front=history_targets_[history_targets_.size()-1].GetBuffArmors()[0].GetAngle();
//                float The_back=history_targets_[history_targets_.size()-17].GetBuffArmors()[0].GetAngle();
//                if(clockwise_){if(The_front<The_back){The_front+=CV_PI*2;}}
//                else{if(The_front>The_back){The_front-=CV_PI*2;}}
                //if(fabs(angle)<5.8){predict_angle_=The_front-The_back;}
//                if(abs(angle)<1.0){
//                    if(clockwise_){ predict_angle_= 0.3; }
//                    else{ predict_angle_= -0.3; }
//                }
                if(target.disturbbuff){
                    predict_angle_=predict_angle_ + history_targets_.back().GetBuffArmors()[2].GetAngle()-history_targets_.back().GetBuffArmors()[0].GetAngle();
                }
                float pre_tmp_angle = history_targets_[history_targets_.size()-1].GetBuffArmors()[0].GetAngle()+predict_angle_;
                while (fabs(pre_tmp_angle) > CV_PI){

                    pre_tmp_angle =pre_tmp_angle + 2 * (float)(CV_PI) * (pre_tmp_angle < 0 ? 1.f : -1.f);
              }
                Debug::AddPoint("能量机关","实际",cv::Point2f(history_targets_[history_targets_.size()-1].GetBuffArmors()[0].GetTimeNow(),history_targets_[history_targets_.size()-1].GetBuffArmors()[0].GetAngle()));
                Debug::AddPoint("能量机关","预测",cv::Point2f(history_targets_[history_targets_.size()-1].GetBuffArmors()[0].GetTimeNow()+450000,pre_tmp_angle));
                Debug::AddPoint("能量机关","角速度",cv::Point2f(history_targets_[history_targets_.size()-1].GetBuffArmors()[0].GetTimeNow(),angle));
            }
            else if (interval_ < 3) {
                interval_++;
            } else {
                interval_ = 0;
            }}







    }
        void BuffPredictor::Predict(ReceiveMessage &receive_message) {
            bool is_final=false;
            if(is_final_==4)
            {
                is_final=true;
            }
//            if(history_targets_.back().GetFinal()!=0)
//            {
//                if(is_pass_) is_final=true;
//                else is_pass_=true;
//            }
//            if (is_final) {
//                 float predict_time = tdttoolkit::Time::GetTimeNow();
//                 predict_time-=last_beat_time_;
//                 predict_time+=((cos(history_targets_.back().GetPolar3f().pitch) / cos(receive_message.shoot_platform.pitch)) * (history_targets_.back().GetPolar3f().distance) / receive_message.shoot_platform.bulletspeed+110);
//                if ( buff_type_ )   predict_point_ = LittlePredictTime2Point (predict_time,receive_message,is_final);
//                else    predict_point_ = BigPredictTime2Point (predict_time,receive_message,is_final);
//            }

            //能量装甲板在物体坐标系下的坐标
//            const ShootPlatform &platform = receive_message.shoot_platform;
//            predict_angle_ = history_targets_.back().GetBuffArmors()[0].GetAngle() + CV_PI;
//            cv::Mat rvec;
//            cv::Vec3f vec_beat_point = cv::Vec3f({0, 70, 15.});
//            cv::Mat mat_beat_point = cv::Mat(vec_beat_point);
//            Rodrigues(history_targets_.back().GetRvec(), rvec);
//            mat_beat_point = rvec * mat_beat_point + history_targets_.back().GetTvec();
//            predict_point_ = cv::Point3f(mat_beat_point);
            float predict_time=(cos(history_targets_.back().GetPolar3f().pitch) / cos(receive_message.shoot_platform.pitch)) * (history_targets_.back().GetPolar3f().distance) / receive_message.shoot_platform.bulletspeed+220;
                Debug::AddPoint("测试点",Point2f(780,620),15,cv::Scalar(125,35,255));

        if ( buff_type_ )   predict_point_ = BigPredictTime2Point (predict_time_,receive_message,is_final);
        else
            predict_point_ =LittlePredictTime2Point (predict_time_,receive_message,is_final);
        }

        Point3f BuffPredictor::BigPredictTime2Point(float &predict_time, ReceiveMessage &receive_message,bool &is_final) {
            Mat rvec_predict,
                rvec_target,
                predict_in_object,
                target_in_object,
                target_in_object_R1,
                rvec_object_to_world,
                pnp_rvec_object_to_world,
                predict_mat;    //矩阵,旋转矩阵,p(旋转之后点在物体坐标系的坐标)
            Vec3f vec_predict_euler, vec_target_eular, p0;


//            if(is_final)
//            {
//                predict_angle_ += history_targets_.back().GetBuffArmors()[history_targets_.back().GetFinal()].GetAngle()-history_targets_.back().GetBuffArmors()[0].GetAngle();
//            }

if(predict_angle_==0) return cv::Point3f(0,0,0);
            //旋转的roll角度
            if(final==1){
                predict_angle_ = predict_angle_ + history_targets_.back().GetBuffArmors()[history_targets_.back().GetFinal()].GetAngle()-history_targets_.back().GetBuffArmors()[0].GetAngle();
            }
            vec_predict_euler = {0, 0, predict_angle_};
            vec_target_eular = {0, 0, 0};

            //旋转向量转换为旋转矩阵
            Rodrigues(Mat(vec_predict_euler), rvec_predict);
            Rodrigues(Mat(vec_target_eular), rvec_target);
            p0 = {0, 70, 15}; //物体坐标系原点为R标中心
            //预测点在物体坐标系下的位置
            predict_in_object = rvec_predict * Mat(p0);

            target_in_object = rvec_target * Mat(p0);

            target_in_object_R1 = rvec_target * Mat(p0);

            //物体到世界坐标系的旋转向量转为旋转矩阵
            Rodrigues(history_targets_.back().GetRvec(), rvec_object_to_world); //R2

            Rodrigues(history_targets_.back().GetPnpRvec(), pnp_rvec_object_to_world); //R1


            //预测点在世界坐标系下的位置
            predict_in_object = rvec_object_to_world * predict_in_object +
                                history_targets_.back().GetTvec();    //预测点在世界坐标系中的位置  R2 T2的世界位置
            target_in_object = rvec_object_to_world * target_in_object +
                               history_targets_.back().GetTvec();    //目标点在世界坐标系中的位置  R2 T2的世界位置
            target_in_object_R1 = pnp_rvec_object_to_world * target_in_object_R1 +
                                  history_targets_.back().GetTvec();    //目标点在世界坐标系中的位置  R1 T1的世界位置

            cout<<"predict_in_object_before "<<predict_in_object<<endl;
            predict_mat = predict_in_object - target_in_object;
            cout<<"predict_mat "<<predict_mat<<endl;
            predict_in_object = target_in_object_R1 + predict_mat;
            cout<<"predict_in_object_after "<<predict_in_object<<endl;

#ifdef VIDEO_DEBUG
            cv::Point2f tmp_point = history_targets_.back().GetBuffArmors()[0].GetBuffRect().GetCenter2f()-history_targets_.back().GetBuffArmors()[0].GetCirclePoint();
            cv::Point3f current_point = cv::Point3f(tmp_point.x,tmp_point.y,0);
            cv::Mat pre_point = cv::Mat(current_point);
            vec_predict_euler = cv::Vec3f{0,0,predict_angle_};
            Rodrigues(cv::Mat(vec_predict_euler), rvec_predict);
            pre_point = rvec_predict*pre_point;
            current_point = cv::Point3f(pre_point);
//            Debug::SetMessage("解算和预测","预测角度",predict_angle_);
            tmp_point =cv::Point2f(current_point.x+history_targets_.back().GetBuffArmors()[0].GetCirclePoint().x,current_point.y+history_targets_.back().GetBuffArmors()[0].GetCirclePoint().y);
//            cout<<"tmp_point "<<tmp_point<<endl;

            Debug::SetMessage("解算和预测","预测角度",predict_angle_);
            Debug::AddPoint("预测点",tmp_point,8,cv::Scalar(0,255,0));

//            angle_pre.push_back(tmp_point);
//            if(angle_pre.size()>20){
//                Debug::AddPoint("测试点",angle_pre[3],8,cv::Scalar(125,35,255));
//                angle_pre.erase(angle_pre.begin());
//            }
#endif

            return *(predict_in_object.ptr<Point3f>());       //转化为Point3f

        }

        Point3f BuffPredictor::LittlePredictTime2Point(float &predict_time, ReceiveMessage &receive_message,bool &is_final) {

            Mat rvec_predict,
                rvec_target,
                predict_in_object,
                target_in_object,
                target_in_object_R1,
                rvec_object_to_world,
                pnp_rvec_object_to_world,
                predict_mat;    //矩阵,旋转矩阵,p(旋转之后点在物体坐标系的坐标)
            //欧拉角旋转向量
            Vec3f vec_predict_euler, vec_target_eular, p0;
            float predict_angle;    //预测的角度


            //预测角度计算,因为6000ms一圈,所以预测时间除以6000乘以2pi就是预测角度,后面是旋转方向(顺逆时针)
            predict_angle = float( 2*CV_PI*predict_time/ 6000.f * (clockwise_ ? 1 : -1));
            //预测的角度与现在角度相结合
            float t1;
            t1 = history_targets_[history_targets_.size() - 1].GetBuffArmors()[0].GetTimeNow() / 1000;

            predict_time_ = predict_time_ - (t1-t2);

            t2 = history_targets_[history_targets_.size() - 1].GetBuffArmors()[0].GetTimeNow() / 1000;
            float pre_time=410,fire_intervel=110;
            LoadParam::ReadTheParam("predict_time",pre_time);
            LoadParam::ReadTheParam("fire_intervel",fire_intervel);
            if(predict_time_<pre_time&&predict_time_>=(pre_time-fire_intervel)){
                start = true;
            }
            if(predict_time_<(pre_time-fire_intervel)){
                start = false;
                predict_time_ = 600;
            }
            if(final==1){
                predict_angle=predict_angle + history_targets_.back().GetBuffArmors()[history_targets_.back().GetFinal()].GetAngle()-history_targets_.back().GetBuffArmors()[0].GetAngle();
            }
            if(receive_message.disturb_buff){
                predict_angle=predict_angle + history_targets_.back().GetBuffArmors()[2].GetAngle()-history_targets_.back().GetBuffArmors()[0].GetAngle();
            }
            predict_angle_ = predict_angle;
            //下一圈的角度
            //旋转的roll角度
            vec_predict_euler = {0, 0, predict_angle};
            vec_target_eular = {0, 0, 0};

            //旋转向量转换为旋转矩阵
            Rodrigues(cv::Mat(vec_predict_euler), rvec_predict);
            Rodrigues(cv::Mat(vec_target_eular), rvec_target);
            p0 = {0, 70, 15};

            //预测点在物体坐标系下的位置
            predict_in_object = rvec_predict * Mat(p0);
            target_in_object = rvec_target * Mat(p0);
            target_in_object_R1 = rvec_target * Mat(p0);
#ifdef VIDEO_DEBUG
            cv::Point2f tmp_point = history_targets_.back().GetBuffArmors()[0].GetBuffRect().GetCenter2f()-history_targets_.back().GetBuffArmors()[0].GetCirclePoint();
            cv::Point3f current_point = cv::Point3f(tmp_point.x,tmp_point.y,0);
            cv::Mat pre_point = cv::Mat(current_point);
            vec_predict_euler = cv::Vec3f{0,0,predict_angle};
            Rodrigues(cv::Mat(vec_predict_euler), rvec_predict);
            pre_point = rvec_predict*pre_point;
            current_point = cv::Point3f(pre_point);
            tmp_point =cv::Point2f(current_point.x+history_targets_.back().GetBuffArmors()[0].GetCirclePoint().x,current_point.y+history_targets_.back().GetBuffArmors()[0].GetCirclePoint().y);
            Debug::SetMessage("解算和预测","预测角度",predict_angle);
            Debug::AddPoint("预测点",tmp_point,8,cv::Scalar(0,255,0));
//            angle_pre.push_back(tmp_point);
//            if(angle_pre.size()>20){
//                Debug::AddPoint("测试点",angle_pre[3],8,cv::Scalar(125,35,255));
//                angle_pre.erase(angle_pre.begin());
//            }
#endif
            //物体到世界坐标系的旋转向量转为旋转矩阵
            Rodrigues(history_targets_.back().GetRvec(), rvec_object_to_world);
            Rodrigues(history_targets_.back().GetPnpRvec(), pnp_rvec_object_to_world);
            //预测点在世界坐标系下的位置
            predict_in_object = rvec_object_to_world * predict_in_object +
                                history_targets_.back().GetTvec();    //预测点在世界坐标系中的位置
            target_in_object = rvec_object_to_world * target_in_object +
                               history_targets_.back().GetTvec();    //目标点在世界坐标系中的位置 R2 T2的世界位置
            target_in_object_R1 = pnp_rvec_object_to_world * target_in_object_R1 +
                                  history_targets_.back().GetTvec();    //目标点在世界坐标系中的位置  R1 T1的世界位置

            predict_mat = predict_in_object - target_in_object;
            predict_in_object = target_in_object_R1 + predict_mat;

            predict_in_object.convertTo(predict_in_object, CV_32F);    //将p转换为32为浮点型向量
            return *(predict_in_object.ptr<Point3f>());       //转化为Point3f


        }


        void BuffPredictor::DetaConvert(const double angle,
                                        const double deta_x,
                                        const double deta_y,
                                        double &deta_radial,
                                        double &deta_tangent) {

            double l = pow((deta_x * deta_x + deta_y * deta_y), 0.5);
            double alpha = CV_PI / 2 - angle - atan(deta_y / deta_x);
            deta_radial = cos(alpha) * l;
            deta_tangent = sin(alpha) * l;
        }

        void
        BuffPredictor::BuffCommandBuild(bool buff_tzaype, ReceiveMessage recevice_message, tdtusart::Send_Struct_t &send_message) {

            const ShootPlatform &platform = recevice_message.shoot_platform;

            /////重力补偿/////
            Polar3f polar_object_in_world = tdttoolkit::RectangularToPolar(predict_point_);
            //斜抛公式进行计算
            float x1, y1;

            float command_y=25;
            LoadParam::ReadTheParam("command_y",command_y);
            y1 = predict_point_.y+command_y;
            x1 = sqrt(predict_point_.x * predict_point_.x + predict_point_.z * predict_point_.z);//平面坐标系中的x1,y1
            Vec2f phi = tdttoolkit::ParabolaSolve(Point2f(x1, y1), platform.bulletspeed);//过点(x1,y1)解抛物线方程,得到两个出射角的解

            
            send_message.pitch =//polar_object_in_world.pitch;
                    fabs(phi[0] )/*- platform.pitch)*/ < fabs(phi[1] )/*- platform.pitch)*/ ? phi[0] : phi[1];//选择较近的出射角
            send_message.yaw = polar_object_in_world.yaw;

            /////开火判断/////
            float deta_x, deta_y;
            deta_x = (sin(platform.yaw - send_message.yaw)) * x1;//x轴方向偏差
            deta_y = (tan(platform.pitch) - tan(send_message.pitch)) * x1;//y轴方向偏差
            double deta_r = 0;
            double deta_g = 0;
            AngleCorrect(predict_angle_);
            static int converge_count = 0;
            DetaConvert(predict_angle_, deta_x, deta_y, deta_r, deta_g);
            //大符和小符的开火判断
            float bigbuff_deta_r=4,bigbuff_deta_g=8;
            float smallbuff_deta_r=12,smallbuff_deta_g=12;
            LoadParam::ReadTheParam("bigbuff_deta_r",bigbuff_deta_r);
            LoadParam::ReadTheParam("bigbuff_deta_g",bigbuff_deta_g);
            LoadParam::ReadTheParam("smallbuff_deta_r",smallbuff_deta_r);
            LoadParam::ReadTheParam("smallbuff_deta_g",smallbuff_deta_g);
            if (buff_type_) {
                if (fabs(deta_r) < bigbuff_deta_r && fabs(deta_g) < bigbuff_deta_g) {
                    converge_count++;
                } else {
                    converge_count = 0;
                }
                if (converge_count > 2) {
                    send_message.beat = true;
                }

            }
            else {///小符
                if (fabs(deta_r) < smallbuff_deta_r && fabs(deta_g) < smallbuff_deta_g) {
                    converge_count++;
                } else {
                    converge_count = 0;
                }
                if (converge_count > 2) {
                    send_message.beat = true;
                }
            }
#ifdef VIDEO_DEBUG
            Debug::SetMessage("解算和预测","预测目标点",cv::Vec3f(predict_point_));
            Debug::SetMessage("解算和预测","deta_r",float(deta_r));
            Debug::SetMessage("解算和预测","deta_g",float(deta_g));
#endif
            //子弹 击打时间间隔
            if(final==0){
                if (tdttoolkit::Time::GetTimeNow() - last_beat_time_ < 700000) {
                    send_message.beat = false;
                    send_message.no_Obj = false;
                }
            }


            /*if(wait==4){
                send_message.beat=true;
            }*/

//            if(final==1){
//                wait=0;
//                start=true;
//                final=0;
//            }
//            if(start){
//                wait+=1;
//            }
            //if(is_final_!=4){start=false;}
            if(is_final_==4&&send_message.beat == true ){
                final=1;
            }
            if(is_final_!=4){
                final=0;
            }
            if(start&&send_message.beat==true&&contain==0&&buff_type_==0){
                my_pitch=send_message.pitch;
                my_yaw=send_message.yaw;
                contain = 1;
            }
            if(!start){
                contain = 0;
            }
            if (send_message.beat) {
                last_beat_time_ = tdttoolkit::Time::GetTimeNow();
                send_message.no_Obj = false;
                send_message.pitch = send_message.pitch/*-platform.pitch*/;
                send_message.yaw = send_message.yaw/*-platform.yaw*/;
                AngleCorrect(send_message.yaw);
                AngleCorrect(send_message.pitch);
                //send_message.beat = false;
            } else {
                send_message.no_Obj = false;
                send_message.pitch = send_message.pitch/*-platform.pitch*/;
                send_message.yaw = send_message.yaw/*-platform.yaw*/;
                AngleCorrect(send_message.yaw);
                AngleCorrect(send_message.pitch);
            }
            if(contain==1){
                last_beat_time_ = tdttoolkit::Time::GetTimeNow();
                send_message.no_Obj = false;
                send_message.pitch = my_pitch/*-platform.pitch*/;
                send_message.yaw = my_yaw/*-platform.yaw*/;
                AngleCorrect(send_message.yaw);
                AngleCorrect(send_message.pitch);
            }
            send_message.pitch = send_message.pitch*180/CV_PI;
            send_message.yaw = send_message.yaw*180/CV_PI;
        }
    }

