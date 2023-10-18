#include "buff_new_resolver.h"

using namespace std;
using namespace cv;
using namespace tdttoolkit;
namespace tdtnewbuff {


    NEWBuffResolver::NEWBuffResolver(tdtcamera::Camera *camera) {
        camera_matrix_ = camera -> GetMatrix(); //内参
        distortion_matrix_ = camera -> GetDistCoeffs(); //畸变矩阵
    }

    void NEWBuffResolver::UnifyCoordinate(cv::Mat &rvec, cv::Mat &tvec, ShootPlatform shootPlatform,
                                       bool to_world) {

        ///R(物体系 ->　相机系)*（相机系　->　世界系) = R(物体系->世界系)
        ///R(物体系 ->　世界系)*（世界系  ->　相机系) =R(物体系 ->相机系）

        //(|①)
        //①:平移矩阵表示单位，旋转矩阵表示转轴

        //世界系->相机系的旋转向量             (|相机系)
        cv::Vec3f euler_camera_to_world = {-shootPlatform.pitch+0.020f, -shootPlatform.yaw-0.020f, 0};

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
}