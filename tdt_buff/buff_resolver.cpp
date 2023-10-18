
#include"buff_resolver.h"

using namespace std;
using namespace cv;
using namespace tdttoolkit;
namespace tdtbuff {


    BuffResolver::BuffResolver(tdtcamera::Camera *camera) {
        camera_matrix_ = camera -> GetMatrix(); //内参
        distortion_matrix_ = camera -> GetDistCoeffs(); //畸变矩阵

    }

    void BuffResolver::UnifyCoordinate(cv::Mat &rvec, cv::Mat &tvec, ShootPlatform shootPlatform,
                                       bool to_world) {

        ///R(物体系 ->　相机系)*（相机系　->　世界系) = R(物体系->世界系)
        ///R(物体系 ->　世界系)*（世界系  ->　相机系) =R(物体系 ->相机系）

        //(|①)
        //①:平移矩阵表示单位，旋转矩阵表示转轴

        //世界系->相机系的旋转向量             (|相机系)
        float camera_pitch=0.020f,camera_yaw=-0.020f;
        LoadParam::ReadTheParam("camera_pitch",camera_pitch);
        LoadParam::ReadTheParam("camera_yaw",camera_yaw);
        cv::Vec3f euler_camera_to_world = {-(shootPlatform.pitch+camera_pitch), -(shootPlatform.yaw+camera_yaw), 0};

        //世界系原点在相机坐标系下的坐标取反　　 (|相机系)　
        cv::Vec3f vec_tvec_camera_in_world = {0, - 4.5, 11.9};


        if ( to_world ) {

            //相机系到世界系的平移矩阵      (|相机系)
            cv::Mat tvec_camera_in_world = cv::Mat (vec_tvec_camera_in_world);

            cv::Mat rotation_matrix_camera_to_world, rotation_matrix_object_to_camera;
            //相机到世界的旋转矩阵           (|世界系)
            rotation_matrix_camera_to_world = tdttoolkit::EulerAnglesToRotationMatrix (euler_camera_to_world,true);
            //物体到世界的平移矩阵               (|世界系)
            tvec = rotation_matrix_camera_to_world * (tvec+tvec_camera_in_world);
            //物体到相机的旋转矩阵               (|相机系)
            Rodrigues (rvec, rotation_matrix_object_to_camera);//旋转向量转换为旋转矩阵(物体到相机)


            //物体->相机*相机->世界 >> 物体到世界的旋转矩阵 >> 旋转向量          (|世界系)
            Rodrigues ( rotation_matrix_camera_to_world*rotation_matrix_object_to_camera , rvec);

        } else {
            //世界坐标系到相机坐标系的欧拉角          (|相机系)
            euler_camera_to_world = - euler_camera_to_world;
            //世界坐标系到相机坐标系的平移向量         (|相机系)
            vec_tvec_camera_in_world = - vec_tvec_camera_in_world;
            //世界坐标系到相机坐标系的平移矩阵         (|相机系)
            cv::Mat tvec_world_in_camera = cv::Mat (vec_tvec_camera_in_world);

            cv::Mat rotation_matrix_world_to_camera, rotation_matrix_object_to_world;
            //世界坐标系到相机坐标系的旋转矩阵
            rotation_matrix_world_to_camera = tdttoolkit::EulerAnglesToRotationMatrix (euler_camera_to_world, false);

            //相机坐标系到世界坐标系的平移矩阵 : 物体->相机的旋转矩阵*世界->物体旋＋物体->相机的平移矩阵　>>世界到相机的平移矩阵　(|相机系)
            tvec = rotation_matrix_world_to_camera *( tvec) + tvec_world_in_camera;
            //物体到世界的旋转矩阵
            Rodrigues (rvec, rotation_matrix_object_to_world);
            //物体到相机的旋转矩阵       物体->世界*世界->相机  >>　物体->相机
            Rodrigues (rotation_matrix_world_to_camera*rotation_matrix_object_to_world , rvec);
        }
    }


    Buff BuffResolver::BuffResolve(std::vector<BuffArmor> &buff_armors, ReceiveMessage &receive_message) {

        if ( buff_armors[0].GetEmpty () || buff_armors[0].GetBuffType () != FlowWater ) return Buff ();


        for(distance_correct_ = 0; distance_correct_<2; distance_correct_++) {

                GetPrimaryParam(buff_armors[0], receive_message);

            if(distance_correct_ == 0) {
                    GetPrimaryParam0(buff_armors[0], receive_message);
                }

            Angle_Correct(buff_armors[0], receive_message);

            if(distance_correct_ == 0)
            {
                Angle_Correct0(buff_armors[0], receive_message);
            }

            Distance_Correct(buff_armors[0], receive_message);
            if(distance_correct_ == 0) {

                Real_Distance_Correct();

                cout<<"real_dintance_test "<<real_distance<<endl;

            }
        }

        Buff buffTarget (buff_armors,rvec_object_to_world_, tvec_object_to_world_, pnp_rvec_object_to_world_1, pnp_tvec_object_to_world_1);//大神符目标

#ifdef VIDEO_DEBUG

        cv::Vec3f tvec_=  Vec3f(tvec_object_to_world_);
        Debug::SetMessage("解算和预测","平移矩阵",tvec_) ;

        tvec_  = Vec3f(rvec_object_to_world_);
        Debug::SetMessage("解算和预测","旋转矩阵",tvec_);
//        std::vector<cv::Point3f> buff_world_points;
//        std::vector<cv::Point2f> buff_image_points;
//        UnifyCoordinate(rvec_object_to_world_,tvec_object_to_world_,receive_message.shoot_platform,false);
//        for(int i=0;i<4;i++)
//        {
//            cv::Vec3f euler_ = cv::Vec3f{0,0,float(i*2*CV_PI/5)};
//            cv::Mat armor_rvec;
//            Rodrigues(cv::Mat(euler_),armor_rvec);
//            cv::Point3f buff_world_point = cv::Point3f(0,70,15);
//            armor_rvec = armor_rvec*cv::Mat(buff_world_point);
//            buff_world_point = cv::Point3f(armor_rvec);
//            buff_world_points.push_back(buff_world_point);
//        }
//        projectPoints(buff_world_points,rvec_object_to_world_,tvec_object_to_world_,camera_matrix_,distortion_matrix_,buff_image_points);
//        for(int i=0;i<4;i++)
//        {
//            Debug::AddPoint("Buff",buff_image_points[i],8,cv::Scalar(0,255,0));
//        }
#endif

        return buffTarget;
    }


    void BuffResolver::GetPrimaryParam(tdttoolkit::BuffArmor &buff_armor,tdttoolkit::ReceiveMessage &receive_message )
    {
        std::vector<cv::Point2f> image_points;
        std::vector<cv::Point3f> world_points;

        //image_points.emplace_back(buff_armor.GetCirclePoint()+Point2f(0,0));//圆心   ///50
        image_points.emplace_back(buff_armor.GetCirclePoint()+Point2f(50,50));
        image_points.emplace_back(buff_armor.GetCirclePoint()+Point2f(-50,-50));
        image_points.emplace_back(buff_armor.GetCirclePoint()+Point2f(50,-50));
        image_points.emplace_back(buff_armor.GetCirclePoint()+Point2f(-50,50));
        image_points.emplace_back(buff_armor.GetCirclePoint()+Point2f(25,25));
        image_points.emplace_back(buff_armor.GetCirclePoint()+Point2f(-25,-25));
        image_points.emplace_back(buff_armor.GetCirclePoint()+Point2f(25,-25));
        image_points.emplace_back(buff_armor.GetCirclePoint()+Point2f(-25,25));


        //world_points.emplace_back(0,0,0);//圆心//物体    ///20
        world_points.emplace_back(20,20,0);
        world_points.emplace_back(-20,-20,0);
        world_points.emplace_back(20,-20,0);
        world_points.emplace_back(-20,20,0);
        world_points.emplace_back(10,10,0);
        world_points.emplace_back(-10,-10,0);
        world_points.emplace_back(10,-10,0);
        world_points.emplace_back(-10,10,0);


        cv::Mat rvec_object_to_camera, tvec_object_in_camera;
        solvePnP (world_points, image_points, camera_matrix_, distortion_matrix_, rvec_object_to_camera,
                  tvec_object_in_camera, false);

        cv::Point3f object_in_camera = cv::Point3f (*(tvec_object_in_camera.ptr<cv::Point3d> ()));
        Polar3f object_in_camera_polar = tdttoolkit::RectangularToPolar(object_in_camera);
        if(distance_correct_==0)
        {
            real_distance = 700;
        }
        object_in_camera_polar.distance = real_distance;

        object_in_camera = tdttoolkit::PolarToRectangular (object_in_camera_polar);
        tvec_object_in_camera = cv::Mat (cv::Point3f (object_in_camera));

        //统一到世界坐标系
        rvec_object_to_camera.convertTo(rvec_object_to_camera, CV_32F);
        UnifyCoordinate (rvec_object_to_camera, tvec_object_in_camera, receive_message.shoot_platform, true);
//定距离

#ifdef VIDEO_DEBUG
    cv::Vec3f tvec_tmp = cv::Vec3f(tvec_object_in_camera);
    Debug::SetMessage("解算和预测","优化平移矩阵",tvec_tmp);

#endif

        rvec_object_to_world_ = rvec_object_to_camera.clone();
        tvec_object_to_world_=tvec_object_in_camera.clone();

        image_points.clear();
        world_points.clear();


    }

    void BuffResolver::GetPrimaryParam0(tdttoolkit::BuffArmor &buff_armor,tdttoolkit::ReceiveMessage &receive_message )
    {
        std::vector<cv::Point2f> image_points;
        std::vector<cv::Point3f> world_points;

        image_points.emplace_back(buff_armor.GetBuffRect().GetCenter2f()+Point2f(0,0));//圆心   //15
        image_points.emplace_back(buff_armor.GetBuffRect().GetTr());
        image_points.emplace_back(buff_armor.GetBuffRect().GetTl());
        image_points.emplace_back(buff_armor.GetBuffRect().GetBl());
        image_points.emplace_back(buff_armor.GetBuffRect().GetBr());

        float fHalfX=0;
        float fHalfY=0;

        float BuffWidth = 23.0;
        float BuffHeight = 12.7;

        fHalfX=BuffWidth/2.0;
        fHalfY=BuffHeight/2.0;

        world_points.emplace_back(cv::Point3f(0.0, 0.0, 0.0));//圆心//物体    ///6
        world_points.emplace_back(cv::Point3f(-fHalfX,-fHalfY,0.0));
        world_points.emplace_back(cv::Point3f(fHalfX,-fHalfY,0.0));
        world_points.emplace_back(cv::Point3f(fHalfX,fHalfY,0.0));
        world_points.emplace_back(cv::Point3f(-fHalfX,fHalfY,0.0));


        cv::Mat rvec_object_to_camera, tvec_object_in_camera;
        solvePnP (world_points, image_points, camera_matrix_, distortion_matrix_, rvec_object_to_camera,
                  tvec_object_in_camera, false, SOLVELP_MULTI);
        ///R1 T1  ，  T1的distance待求出

        ///这一系列的操作是将tvec_object_in_camera中的距离向量进行修正
        cv::Point3f object_in_camera = cv::Point3f (*(tvec_object_in_camera.ptr<cv::Point3d> ()));
        Polar3f object_in_camera_polar = tdttoolkit::RectangularToPolar(object_in_camera);

        real_distance = 700;

        object_in_camera_polar.distance = real_distance;

        object_in_camera = tdttoolkit::PolarToRectangular (object_in_camera_polar);
        tvec_object_in_camera = cv::Mat (cv::Point3f (object_in_camera));

        //统一到世界坐标系
        rvec_object_to_camera.convertTo(rvec_object_to_camera, CV_32F);
        UnifyCoordinate (rvec_object_to_camera, tvec_object_in_camera, receive_message.shoot_platform, true);
//定距离

        pnp_rvec_object_to_world_1 = rvec_object_to_camera.clone();
        pnp_rvec_object_to_world_1.convertTo(pnp_rvec_object_to_world_1, CV_32F);
        pnp_tvec_object_to_world_1 = tvec_object_in_camera.clone();

        image_points.clear();
        world_points.clear();


    }



    void BuffResolver::Angle_Correct(tdttoolkit::BuffArmor &buff_armor,tdttoolkit::ReceiveMessage &receive_message)
    {

        float rotate_angle = CV_PI + buff_armor.GetAngle ();

        //得到欧拉角，以竖直向下为转轴，顺时针为正，
        //能量机关到转轴的欧拉角
        cv::Vec3f euler_object_to_world = {-0.15 , 0, rotate_angle};
        //像素为单位的能量机关的半径

        //物体到世界的旋转矩阵和平移矩阵

        std::vector<cv::Point3f> real_world_points;
        tdttoolkit::WorldPointLists::GetBuffWorldPoints(0,real_world_points);
        float stride = 0.008f; //0.015f
        cv::Vec3f euler_tmp = euler_object_to_world;
        std::vector<cv::Point2f> pro_image_points;

        for ( int i = 0; i < 35; i ++ ) {
            //矫正picth和yaw欧拉角
            if ( i > 0) {
//            得到一个临时欧拉角，用于循环迭代，改变

                //第一个欧拉角的递增，促使绕x轴向上旋转
                euler_tmp[0] += stride;
                //欧拉角转为旋转矩阵         物体->世界
                rvec_object_to_world_ = tdttoolkit::EulerAnglesToRotationMatrix (euler_tmp, true);
                //旋转矩阵转为旋转向量       物体->世界
                Rodrigues (rvec_object_to_world_, rvec_object_to_world_);
                //  世界坐标系相对相机坐标系
                cv::Mat tvec_object_to_world = tvec_object_to_world_.clone();

                UnifyCoordinate (rvec_object_to_world_, tvec_object_to_world,receive_message.shoot_platform, false);

                pro_image_points.clear ();
                //根据旋转矩阵和平移矩阵进行重映射
                projectPoints (real_world_points, rvec_object_to_world_,
                               tvec_object_to_world,
                               camera_matrix_, distortion_matrix_, pro_image_points);

                //得到映射与实际的差值
                float pro_distance = CalcDistance(pro_image_points[1],buff_armor.GetBuffRect().GetCenter2f());
#ifdef VIDEO_DEBUG
                Debug::AddPoint("测试点",pro_image_points[1],2,cv::Scalar(255,255,0));
#endif

                if ( pro_distance <= pix_grade_) {
                    pix_grade_ =  pro_distance;
                    euler_object_to_world=euler_tmp;
                }
                else{
                    stride=-0.008f;
                }

            }
            else{
                //欧拉角转为旋转矩阵

                rvec_object_to_world_ = tdttoolkit::EulerAnglesToRotationMatrix (euler_tmp, true);

                //旋转矩阵转为旋转向量
                Rodrigues (rvec_object_to_world_, rvec_object_to_world_);
                //平移矩阵的复制　　sovepnp解出平移矩阵->定距离->平移矩阵
                cv::Mat tvec_object_to_world = tvec_object_to_world_.clone();

                UnifyCoordinate (rvec_object_to_world_, tvec_object_to_world, receive_message.shoot_platform,
                                 false);

                pro_image_points.clear ();
                //求出映射点

                projectPoints (real_world_points, rvec_object_to_world_,
                               tvec_object_to_world,
                               camera_matrix_, distortion_matrix_, pro_image_points);

                pix_grade_= tdttoolkit::CalcDistance(pro_image_points[1],buff_armor.GetBuffRect().GetCenter2f());

            }
            cv::Point2f dif = pro_image_points[1]-pro_image_points[0];
            float angle = atan2(dif.x,-dif.y);
            euler_tmp[2] += CalcAngleDifference(buff_armor.GetAngle(),angle);
            if(CalcDistance(pro_image_points[1],buff_armor.GetBuffRect().GetCenter2f())<0.3)
                break;
        }
        rvec_object_to_world_=EulerAnglesToRotationMatrix(euler_object_to_world,true);

        Rodrigues(rvec_object_to_world_,rvec_object_to_world_);
        UnifyCoordinate(rvec_object_to_world_,tvec_object_to_world_,receive_message.shoot_platform,false);
        pro_image_points.clear();

        projectPoints (real_world_points, rvec_object_to_world_, tvec_object_to_world_,
                       camera_matrix_, distortion_matrix_, pro_image_points);

        UnifyCoordinate(rvec_object_to_world_,tvec_object_to_world_,receive_message.shoot_platform,true);

    }

    void BuffResolver::Angle_Correct0(tdttoolkit::BuffArmor &buff_armor,tdttoolkit::ReceiveMessage &receive_message)
    {

        float rotate_angle = CV_PI + buff_armor.GetAngle ();

        //得到欧拉角，以竖直向下为转轴，顺时针为正，
        //能量机关到转轴的欧拉角
        cv::Vec3f euler_object_to_world = {-0.15 , 0, rotate_angle};
        //像素为单位的能量机关的半径

        //物体到世界的旋转矩阵和平移矩阵

        std::vector<cv::Point3f> real_world_points;
        tdttoolkit::WorldPointLists::GetBuffWorldPoints(0,real_world_points);
        float stride = 0.008f; //0.015f
        cv::Vec3f euler_tmp = euler_object_to_world;
        std::vector<cv::Point2f> pro_image_points;

        for ( int i = 0; i < 35; i ++ ) {
            //矫正picth和yaw欧拉角
            if ( i > 0) {
//            得到一个临时欧拉角，用于循环迭代，改变

                //第一个欧拉角的递增，促使绕x轴向上旋转
                euler_tmp[0] += stride;
                //欧拉角转为旋转矩阵         物体->世界
                pnp_rvec_object_to_world_1 = tdttoolkit::EulerAnglesToRotationMatrix (euler_tmp, true);
                //旋转矩阵转为旋转向量       物体->世界
                Rodrigues (pnp_rvec_object_to_world_1, pnp_rvec_object_to_world_1);
                //  世界坐标系相对相机坐标系
                cv::Mat tvec_object_to_world = tvec_object_to_world_.clone();

                UnifyCoordinate (pnp_rvec_object_to_world_1, tvec_object_to_world,receive_message.shoot_platform, false);

                pro_image_points.clear ();
                //根据旋转矩阵和平移矩阵进行重映射
                projectPoints (real_world_points, pnp_rvec_object_to_world_1,
                               tvec_object_to_world,
                               camera_matrix_, distortion_matrix_, pro_image_points);

                //得到映射与实际的差值
                float pro_distance = CalcDistance(pro_image_points[1],buff_armor.GetBuffRect().GetCenter2f());
#ifdef VIDEO_DEBUG
                Debug::AddPoint("测试点",pro_image_points[1],2,cv::Scalar(255,255,0));
#endif

                if ( pro_distance <= pix_grade_) {
                    pix_grade_ =  pro_distance;
                    euler_object_to_world=euler_tmp;
                }
                else{
                    stride=-0.008f;
                }

            }
            else{
                //欧拉角转为旋转矩阵

                pnp_rvec_object_to_world_1 = tdttoolkit::EulerAnglesToRotationMatrix (euler_tmp, true);

                //旋转矩阵转为旋转向量
                Rodrigues (pnp_rvec_object_to_world_1, pnp_rvec_object_to_world_1);
                //平移矩阵的复制　　sovepnp解出平移矩阵->定距离->平移矩阵
                cv::Mat tvec_object_to_world = tvec_object_to_world_.clone();

                UnifyCoordinate (pnp_rvec_object_to_world_1, tvec_object_to_world, receive_message.shoot_platform,
                                 false);

                pro_image_points.clear ();
                //求出映射点

                projectPoints (real_world_points, pnp_rvec_object_to_world_1,
                               tvec_object_to_world,
                               camera_matrix_, distortion_matrix_, pro_image_points);

                pix_grade_= tdttoolkit::CalcDistance(pro_image_points[1],buff_armor.GetBuffRect().GetCenter2f());

            }
            cv::Point2f dif = pro_image_points[1]-pro_image_points[0];
            float angle = atan2(dif.x,-dif.y);
            euler_tmp[2] += CalcAngleDifference(buff_armor.GetAngle(),angle);
            if(CalcDistance(pro_image_points[1],buff_armor.GetBuffRect().GetCenter2f())<0.3)
                break;
        }
        pnp_rvec_object_to_world_1=EulerAnglesToRotationMatrix(euler_object_to_world,true);

        Rodrigues(pnp_rvec_object_to_world_1,pnp_rvec_object_to_world_1);
        UnifyCoordinate(pnp_rvec_object_to_world_1,tvec_object_to_world_,receive_message.shoot_platform,false);
        pro_image_points.clear();

        projectPoints (real_world_points, pnp_rvec_object_to_world_1, tvec_object_to_world_,
                       camera_matrix_, distortion_matrix_, pro_image_points);

        UnifyCoordinate(pnp_rvec_object_to_world_1,tvec_object_to_world_,receive_message.shoot_platform,true);

    }


    void BuffResolver::Distance_Correct(tdttoolkit::BuffArmor &buff_armor,tdttoolkit::ReceiveMessage &receive_message) {

        std::vector<cv::Point2f> pro_image_points;
        //进行映射　　　平移矩阵：物体->相机　　旋转矩阵：物体->相机
        std::vector<cv::Point3f> real_world_points;
        tdttoolkit::WorldPointLists::GetBuffWorldPoints(0, real_world_points);

        cv::Mat rvec_camera_to_world = rvec_object_to_world_.clone();
        cv::Mat tvec_camera_to_world = tvec_object_to_world_.clone();
        UnifyCoordinate(rvec_camera_to_world, tvec_camera_to_world, receive_message.shoot_platform, false);

        float pixel_r = CalcDistance(buff_armor.GetCirclePoint(), buff_armor.GetBuffRect().GetCenter2f());
        float pixel_r_pro, diff_r_fix;
        for (int i = 0; i < 10; i++) {
            projectPoints(real_world_points, rvec_camera_to_world, tvec_camera_to_world, camera_matrix_, distortion_matrix_, pro_image_points);

            // float diff_r=CalcDistance(pro_image_points[1],pro_image_points[0])-pixel_r;

            pixel_r_pro = CalcDistance(pro_image_points[1], pro_image_points[0]);
            //三个相似，比例关系，diff_r_fix为重映射到图像的差距
            diff_r_fix = (pixel_r_pro - pixel_r) * tvec_camera_to_world.at<float>(2) / (pixel_r_pro - 70);

            cv::Point3f         camera_point = Point3f(*(tvec_camera_to_world.ptr<Point3f>())); //转化为Point3f
            tdttoolkit::Polar3f camera_polor = RectangularToPolar(camera_point);
            camera_polor.distance += diff_r_fix;
            camera_point         = PolarToRectangular(camera_polor);
            tvec_camera_to_world = cv::Mat(cv::Point3f(camera_point));
        }
        //    float add_or_sub=1.;
        //    for ( int i = 0; i < 20; i ++ ) {
        //
        //       if(i==0)
        //       {
        //           pro_image_points.clear();
        //           projectPoints (real_world_points, rvec_camera_to_world, tvec_camera_to_world,
        //                          camera_matrix_, distortion_matrix_, pro_image_points);
        //
        //           //得到映射与实际的差值
        //           float diff_r = tdttoolkit::CalcDistance(pro_image_points[1],buff_armor.GetBuffRect().GetCenter2f());
        //
        //           Polar3f tmp_polor = RectangularToPolar(cv::Point3f(tvec_camera_to_world));
        //           tmp_polor.distance+=add_or_sub*diff_r;
        //           tvec_camera_to_world = cv::Mat(PolarToRectangular(tmp_polor));
        //       }
        //       else
        //       {
        //           pro_image_points.clear();
        //
        //           projectPoints (real_world_points, rvec_camera_to_world, tvec_camera_to_world,
        //                          camera_matrix_, distortion_matrix_, pro_image_points);
        //           float diff_r = tdttoolkit::CalcDistance(pro_image_points[1],buff_armor.GetBuffRect().GetCenter2f());
        //            std::cout<<"第"<<i<<"个grade："<<diff_r<<std::endl;
        //           if(diff_r<pix_grade_)
        //            {
        //                pix_grade_=diff_r;
        //                tvec_object_to_world_=tvec_camera_to_world.clone();
        //                cv::Mat vec_tmp = rvec_camera_to_world.clone();
        //            }
        //            else if(add_or_sub==-1)  break;
        //            else add_or_sub=-1.;
        //
        //            Polar3f tmp_polor = RectangularToPolar(cv::Point3f(tvec_camera_to_world));
        //            tmp_polor.distance+=add_or_sub*diff_r;
        //            tvec_camera_to_world = cv::Mat(PolarToRectangular(tmp_polor));
        //       }
        //    }

        UnifyCoordinate(rvec_camera_to_world, tvec_camera_to_world, receive_message.shoot_platform, true);

#ifdef VIDEO_DEBUG
        Debug::AddPoint("Buff", pro_image_points[1], 10, cv::Scalar(0, 0, 255));
#endif

        tvec_object_to_world_ = tvec_camera_to_world.clone();
    }


    void BuffResolver::Real_Distance_Correct()
{

    Mat   rvec_object, buff_armors_point, rvec_object_to_camera_1, rvec_object_to_world_1;
    Vec3f vec_object_euler, point_object;

    Mat rvec_object_to_camera_0 = rvec_object_to_world_.clone(); /// R2
    Mat tvec_object_to_camera_0 = tvec_object_to_world_.clone(); /// T2

    vec_object_euler = {0, 0, 0};

    Rodrigues(Mat(vec_object_euler), rvec_object);
    point_object      = {0, 70, 15};
    buff_armors_point = rvec_object * Mat(point_object);
    Rodrigues(rvec_object_to_camera_0, rvec_object_to_camera_1);
    buff_armors_point = rvec_object_to_camera_1 * buff_armors_point + tvec_object_to_camera_0;

    cv::Point3f target_in_camera       = cv::Point3f(*(buff_armors_point.ptr<cv::Point3f>()));
    Polar3f     target_in_camera_polar = tdttoolkit::RectangularToPolar(target_in_camera);

        real_distance = target_in_camera_polar.distance;

}



}