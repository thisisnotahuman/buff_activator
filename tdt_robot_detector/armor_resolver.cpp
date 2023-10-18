#ifdef O3ENABLE
#pragma GCC optimize(3, "Ofast", "inline")
#endif
#include "armor_resolver.h"
#ifdef VIDEO_DEBUG
#include "Debug.h"
#endif
#include <fstream>
#include <omp.h>

using namespace tdttoolkit;
using namespace tdtcamera;
using namespace std;
using namespace cv;

namespace tdtrobot {
    ArmorResolver::ArmorResolver() {

        LoadParam::ReadTheParam("matrix", camera_matrix_);    //内参
        LoadParam::ReadTheParam("dist_coeffs", dist_coeffs_); //畸变矩阵
        robot_type_ = TYPEUNKNOW;
        pnp_rvec_   = Mat(3, 3, CV_32F, Scalar::all(0));
        pnp_tvec_   = Mat(3, 3, CV_32F, Scalar::all(0));
    }

    vector<ResolvedArmor> ArmorResolver::Resolve(vector<RobotArmor> &armors, ShootPlatform shoot_platform) {

        vector<ResolvedArmor> resolved_armors(armors.size());
        //将armors按照图像中从左到右的位置顺序排序
        sort(armors.begin(), armors.end(), [](RobotArmor a, RobotArmor b) -> bool { return (a.GetStickerRect().GetCenter().x < b.GetStickerRect().GetCenter().x); });
        auto t1 = cv::getTickCount();
        if (!armors.empty()) {
#ifdef HAVE_OPENMP
#pragma omp parallel for private(polar_in_camera_, robot_type_)
#endif
            for (int i = 0; i < armors.size(); i++) {
                auto      armor      = armors[i];
                RobotType robot_type = armor.GetRobotType();

                auto                     aWorldPointList = armor.GetWorldPointsList(); //装甲板的三维坐标
                auto                     aImagePointList = armor.GetImagePointsList(); //装甲板的二维坐标
                std::vector<cv::Point3f> worldPointList;
                std::vector<cv::Point2f> imagePointList;

                for (int j = 0; j < aImagePointList.size(); ++j) {
                    if (aImagePointList[j].x * aImagePointList[j].y != 0 && (aWorldPointList[j].x != 0 || aWorldPointList[j].y != 0)) {
                        imagePointList.push_back(aImagePointList[j]);
                        worldPointList.push_back(aWorldPointList[j]);
                    }
                }

                // std::cout << "imagePointList: \n" << imagePointList << "\n";
                Mat pnp_rvec, pnp_tvec, TVecObjInWorld = Mat(3, 1, CV_32F, Scalar::all(0)), RVecObjToWorld = Mat(3, 1, CV_32F, Scalar::all(0));
                // std::cout << "imagePointList.size=" << imagePointList.size() << std::endl;
                if (imagePointList.size() < 4) {
                    resolved_armors[i] = ResolvedArmor();
                    continue;
                }
                solvePnP(worldPointList, imagePointList, camera_matrix_, dist_coeffs_, pnp_rvec, pnp_tvec, false);
                ///画出解算的装甲板法线
                /*vector<Point3f> pointsxyz={Point3f(0,0,40),Point3f(0,0,0)};
                vector<Point2f> points;
                projectPoints(pointsxyz,pnp_rvec,pnp_tvec,camera_matrix_,dist_coeffs_,points);
                points[0].y=points[1].y;
                Debug::AddLine("解算的装甲板法线",points[0],points[1],Scalar(0,255,255));*/
                ////////////////

                //                vector<cv::Point2f> imgPoints;
                //                cv::projectPoints(worldPointList, pnp_rvec, pnp_tvec, camera_matrix_, dist_coeffs_, imgPoints);
                //                Debug::AddCircle("解算点投影", imgPoints[7], 6, Scalar(255,255,0));

                cv::Vec3f            euler_world_to_object = Vec3f(0, 0, 0);   //世界系->物体系  Z-X-Y欧拉角
                tdttoolkit ::Polar3f polar_in_world        = Polar3f{0, 0, 0}; //物体系原点(装甲板中心点)在世界系中的极坐标
                UnifyCoordinate(robot_type, pnp_rvec, pnp_tvec, shoot_platform, TVecObjInWorld, RVecObjToWorld, polar_in_world, euler_world_to_object);
                ResolvedArmor resolved_armor(robot_type, armor.GetStickerRect(), TVecObjInWorld, euler_world_to_object, polar_in_world, pnp_tvec, pnp_rvec, armor.GetImagePointsList(), armor.GetWorldPointsList(), armor.GetBelievable());

                resolved_armor.SetRVecToWorld(RVecObjToWorld);
                resolved_armor.SetTVecInWorld(TVecObjInWorld);
                resolved_armors[i] = resolved_armor;
                imagePointList.clear();
                worldPointList.clear();
            }
            return resolved_armors;
        }
    }

    // ResolvedArmor ArmorResolver::PreciseResolve(tdttoolkit::ResolvedArmor &resolvedArmor, tdttoolkit::ShootPlatform shoot_platform, tdttoolkit::ResolvedArmor &last_armor, Mat src, Mat &last_roi) {

    //     if (last_armor.GetBoundingRect().empty()) {
    //         last_armor = resolvedArmor;
    //         last_roi   = src(last_armor.GetBoundingRect());
    //         ResolvedArmor precise_resolved_armor;
    //         return precise_resolved_armor; //待验证：晚一帧开始传给预测值，是否会有问题？
    //     } else {
    //         if (CalcDistance(resolvedArmor.GetStickerRect().GetCenter(), last_armor.GetStickerRect().GetCenter()) > 5) {
    //             PreciseMeasurement pre_solve;
    //             vector<Point2f>    last_armor_points;
    //             vector<Point2f>    current_armor_points;
    //             Mat                current_roi = src(resolvedArmor.GetStickerRect().GetRect());
    //             GetSimilarPoint(last_roi, current_roi, last_armor_points, current_armor_points);
    //             vector<Point2f> image_points;
    //             projectPoints(last_armor.GetWorldPoints(), last_armor.GetPNPRvec(), last_armor.GetPNPTvec(), camera_matrix_, dist_coeffs_, image_points);
    //             Mat tran_parameter;
    //             GetPerspectiveTransformation(last_armor_points, current_armor_points, image_points, tran_parameter); //应该传出变换参数
    //             // pre_solve.IterationToApproach(resolvedArmor.GetImagePoints(),image_points,last_armor.GetPolar(),
    //             // last_armor.GetPNPTvec(),tran_parameter);//应该为last_armor的polar和euler
    //             // TODO　此处的polar和eulur可能不是自己想要的，记得改，要的是ｐｎｐ直接求出来算的polar和eulur
    //             Mat estimate_rvec, estimate_tvec;
    //             solvePnP(image_points, resolvedArmor.GetWorldPoints(), camera_matrix_, dist_coeffs_, estimate_rvec, estimate_tvec, false);
    //             pre_solve.GetBestMeasurement(estimate_rvec, estimate_tvec);
    //             pre_solve.GetRotationCenter();
    //             Mat rvec = pre_solve.GetPreciseRot();
    //             Mat tvec = pre_solve.GetPreciseTvec();
    //             UnifyCoordinate(resolvedArmor.GetRobotType(), rvec, tvec, shoot_platform);
    //             last_armor = resolvedArmor;
    //             last_roi   = current_roi;
    //             ResolvedArmor precise_resolved_armor(resolvedArmor.GetRobotType(), resolvedArmor.GetStickerRect(), TVecObjInWorld_, euler_world_to_object_, polar_in_world_, pre_solve.GetPreciseTvec(), pre_solve.GetPreciseRot(), resolvedArmor.GetImagePoints(), resolvedArmor.GetWorldPoints(), pre_solve.GetRotCenter(), resolvedArmor.GetBelievable());
    //             return precise_resolved_armor;
    //         }
    //         return resolvedArmor;
    //     }
    // }

    cv::Point2d ArmorResolver::ProjectArmor(const ShootPlatform &shootPlatform, const ResolvedArmor &resolvedArmor, cv::Point3f worldPoint) {
        cv::Mat RVecObjToWorld = resolvedArmor.GetRVecToWorld();
        cv::Mat TVecObjInWorld = resolvedArmor.GetTVecInWorld();
        cv::Mat TVecObjInCam   = resolvedArmor.GetPNPTvec();

        cv::Mat cameraMatrix, distCoeffs;
        LoadParam::ReadTheParam("matrix", cameraMatrix);
        LoadParam::ReadTheParam("dist_coeffs", distCoeffs);
        // 世界系到相机系的欧拉角 (Z-Y-X欧拉角)
        cv::Vec3f EulerWorldToCam = {shootPlatform.pitch, shootPlatform.yaw, 0};
        // 世界系到相机系的旋转矩阵
        cv::Mat RMatWorldToCam = tdttoolkit::EulerAnglesToRotationMatrix(EulerWorldToCam, true);

        // 物体系到相机系的旋转矩阵和旋转向量
        cv::Mat RMatObjToWorld;
        cv::Rodrigues(RVecObjToWorld, RMatObjToWorld);
        cv::Mat RMatObjToCam = RMatObjToWorld * RMatWorldToCam;
        cv::Mat RVecObjToCam;
        cv::Rodrigues(RMatObjToCam, RVecObjToCam);

        // 将装甲板中心点做反投影
        std::vector<cv::Point3f> worldPoints;
        std::vector<cv::Point2f> projPoints;
        worldPoints.push_back(worldPoint);
        cv::projectPoints(worldPoints, RVecObjToCam, TVecObjInCam, cameraMatrix, distCoeffs, projPoints);
        if (!projPoints.empty())
            return cv::Point2d(int(projPoints[0].x), int(projPoints[0].y));
    }
    void ArmorResolver::UnifyCoordinate(RobotType robot_type, Mat &pnp_rvec, Mat &pnp_tvec, ShootPlatform shootPlatform, Mat TVecObjInWorld, Mat RVecObjToWorld, tdttoolkit ::Polar3f &polar_in_world, cv::Vec3f &euler_world_to_object) {

        /// 统一数据类型为 float（CV_32F）, 类型不一致经常报错

        pnp_tvec    = Mat_<float>(pnp_tvec);
        robot_type_ = robot_type;

        // 世界系到相机系的欧拉角 (Z-Y-X欧拉角)
        Vec3f euler_world_to_camera = {shootPlatform.pitch, shootPlatform.yaw, 0};
        // 19年代码: diffeular={shootPlatform.pitch+0.01f, shootPlatform.yaw - 0.0035f, 0};//云台角度补偿
        // 相机系到世界系的欧拉角 (X-Y-Z欧拉角)
        Vec3f euler_camera_to_world = -euler_world_to_camera;

        // 相机系到世界系的旋转矩阵       R(相机系 -> 世界系)
        Mat rotation_matrix_camera_to_world = EulerAnglesToRotationMatrix(euler_camera_to_world, true); // X-Y-Z

        // 物体系原点在相机系中的坐标     P(物体系原点|相机系) = pnp_tvec

        // 相机系原点在世界系中的坐标     P(相机系原点|世界系)
        //         float DistCamToWorld;
        //         LoadParam::ReadTheParam("DistCamToWorld", DistCamToWorld);
        //         Polar3f current_platform_polar = {DistCamToWorld, shootPlatform.yaw, shootPlatform.pitch};
        //         Vec3f current_platform_polar = {DistCamToWorld, shootPlatform.yaw, shootPlatform.pitch};
        //         Mat tvec_camera_in_world = Mat((current_platform_polar));
        Mat tvec_camera_in_world = Mat(TvecCameraInWorld_); //相机到转轴的位置补偿,不同车不一样,理解为世界坐标系原点在相机坐标系下的位置,并方向取反

        // 物体系原点在世界系中的坐标     P(物体系原点|世界系)
        Mat tvec_object_in_world(3, 1, CV_32F);
        TVecObjInWorld = rotation_matrix_camera_to_world * (pnp_tvec + tvec_camera_in_world);

        // 物体系原点在相机系中的极坐标(Polar3f)
        polar_in_camera_ = RectangularToPolar(*pnp_tvec.ptr<Point3f>());

        // 物体系原点在世界系中的极坐标(Polar3f)
        //        TVecObjInWorld = *(tvec_object_in_world.ptr<Point3f>());
        polar_in_world = RectangularToPolar(*TVecObjInWorld.ptr<Point3f>());
        //        std::cout << "TEST,original: " << *TVecObjInWorld.ptr<Point3f>();
        //        cv::Point3f temp = PolarToRectangular(polar_in_world);
        //        std::cout << "TEST, transformed:  " << temp << std::endl;

        // 物体系到相机系的旋转矩阵       R(物体系 -> 相机系)
        Mat rotation_matrix_object_to_camera(3, 3, CV_32F);
        Rodrigues(pnp_rvec, rotation_matrix_object_to_camera);
        rotation_matrix_object_to_camera = Mat_<float>(rotation_matrix_object_to_camera);

        // 物体系到世界系的旋转矩阵       R(物体系 -> 世界系)
        Mat rotation_matrix_object_to_world = rotation_matrix_object_to_camera * rotation_matrix_camera_to_world;
        cv::Rodrigues(rotation_matrix_object_to_world, RVecObjToWorld); // 旋转向量
        // 世界系到物体系的旋转矩阵       R(世界系 -> 物体系)
        Mat rotation_matrix_world_to_object;
        transpose(rotation_matrix_object_to_world, rotation_matrix_world_to_object);

        // 世界系到物体系的Z-X-Y欧拉角
        Vec3f temp_angles = RotationMatrixToEulerAngles(rotation_matrix_world_to_object, false);

        euler_world_to_object = {temp_angles[0], temp_angles[1] - polar_in_world.yaw, temp_angles[2]};

        return; // 不显示解算结果

        cout << endl << "-------------------------resolve_armor [start]---------------------------" << endl;

        cout << "euler_world_to_camera (Z-Y-X) = " << euler_world_to_camera << endl;               //世界系到相机系的 Z-Y-X 欧拉角
        cout << "euler_camera_to_world (X-Y-Z) = " << euler_camera_to_world << endl;               //相机系到世界系的 X-Y-Z 欧拉角
        cout << "euler_world_to_object = " << euler_world_to_object / CV_PI * 180 << endl << endl; // 世界系到物体系的欧拉角

        cout << "tvec_object_in_camera = " << pnp_tvec << endl;                                   //物体系在相机系中的位置
        cout << "tvec_camera_in_world = " << tvec_camera_in_world << endl;                        // 相机系在世界系中的位置
        cout << "tvec_object_in_world = " << TVecObjInWorld << endl << endl;                      // 物体系在世界系中的位置
        cout << "polar_in_camera.distance_ = " << polar_in_camera_.distance << endl;              // 物体系在相机系中的极坐标[distance]
        cout << "polar_in_camera.yaw_ = " << polar_in_camera_.yaw / CV_PI * 180 << " 度\n";       // 物体系在相机系中的极坐标[yaw]
        cout << "polar_in_camera.pitch_ = " << polar_in_camera_.pitch / CV_PI * 180 << " 度\n\n"; // 物体系在相机系中的极坐标 [pitch]

        cout << "polar_in_world.distance_ = " << polar_in_world.distance << endl;            // 物体系在世界系中的极坐标[distance]
        cout << "polar_in_world.yaw_ = " << polar_in_world.yaw / CV_PI * 180 << " 度\n";     // 物体系在世界系中的极坐标[yaw]
        cout << "polar_in_world.pitch_ = " << polar_in_world.pitch / CV_PI * 180 << " 度\n"; // 物体系在世界系中的极坐标 [pitch]

        cout << "-------------------------resolve_armor [end]---------------------------" << endl << endl;

        /*ofstream tmpout("tmp.txt",ios::app);
        if(polar_in_world.distance<100){
                tmpout << "tvec_object_in_world = " << TVecObjInWorld << endl; // 物体系在世界系中的位置
                tmpout << polar_in_world.distance << endl<<endl;
        }
        tmpout.close();*/
    }

    cv::Point2d ArmorResolver::WorldToPixel(const tdttoolkit::ShootPlatform &shootPlatform, cv::Point3f worldPoint) {

        cv::Mat cameraMatrix, distCoeffs;
        LoadParam::ReadTheParam("matrix", cameraMatrix);
        LoadParam::ReadTheParam("dist_coeffs", distCoeffs);
        // 世界系到相机系的欧拉角 (Z-Y-X欧拉角)
        cv::Vec3f EulerWorldToCam = {shootPlatform.pitch, shootPlatform.yaw, 0};
        // 世界系到相机系的旋转矩阵
        cv::Mat RMatWorldToCam = tdttoolkit::EulerAnglesToRotationMatrix(EulerWorldToCam, false);

        cv::Mat RvecWorldToCam;
        cv::Rodrigues(RMatWorldToCam, RvecWorldToCam);

        // 将装甲板中心点做反投影
        std::vector<cv::Point3f> worldPoints;
        std::vector<cv::Point2f> projPoints;
        worldPoints.push_back(worldPoint);
        cv::projectPoints(worldPoints, RvecWorldToCam, cv::Mat(-TvecCameraInWorld_), cameraMatrix, distCoeffs, projPoints);
        if (!projPoints.empty())
            return cv::Point2d(int(projPoints[0].x), int(projPoints[0].y));
    }

    std::pair<float, float> ArmorResolver::GetFOV() {
        static std::pair<float, float> FOV = {-1, -1};
        if (FOV != (std::pair<float, float>){-1, -1})
            return FOV;
        auto f_pixel_x = camera_matrix_.at<double>(0, 0);
        auto f_pixel_y = camera_matrix_.at<double>(1, 1);
        FOV            = {2 * atan(0.5 * 1440 / f_pixel_x), 2 * atan(0.5 * 1080 / f_pixel_y)};
        // std::cout << "FOV=" << FOV.first << " " << FOV.second << std::endl;
        return FOV;
    }

} // namespace tdtrobot