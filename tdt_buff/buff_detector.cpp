#include "buff_detector.h"
#include "buff_resolver.h"
#include "tdtcamera.h"

using namespace std;
using namespace cv;
using namespace cv::ml;
using namespace tdttoolkit;

struct TDTBuffInformation
{
    float buff_armor_area=0;
    float buff_height=0;
    float buff_width=0;
    float r_area=0;
    float r_armor_area=0;   //识别r部分的装甲板面积
};
TDTBuffInformation BuffDebug = {0,0,0,0,0};
void EneryArmorDetectDilate(cv::Mat &src)
{
    int dilate_size,dilate_times;
    LoadParam::ReadTheParam("EnergyBuffDetect_dialesize",dilate_size);
    LoadParam::ReadTheParam("EnergyBuffDetect_dialetimes",dilate_times);
    for( int i =0;i<dilate_times;i++)
    {
        if(dilate_times ==0) continue;

        dilate(src,src,getStructuringElement(cv::MORPH_RECT,cv::Size(dilate_size,dilate_size)));//膨胀
}
}

void EneryArmorDetectErode(cv::Mat &src)
{
    int erode_size,erode_times;
    LoadParam::ReadTheParam ("EnergyBuffDetect_erodesize",erode_size);
    LoadParam::ReadTheParam("EnergyBuffDetect_erodetimes",erode_times);
    for(int i=0;i<erode_times;i++)
    {
        if(erode_size ==0) continue;
    erode(src,src,getStructuringElement(cv::MORPH_RECT,cv::Size(erode_size,erode_size)));//腐蚀
}}

bool RectSafety(Rect &rect,const Size &size) {
    Rect out_rect=Rect(0,0,size.width,size.height);
    rect=rect&out_rect;
    return rect.area()==0;
}
bool IsInPoints(const std::vector<cv::Point2f> &points_a,const cv::Point2f &point_b)
{
    float x_low=1500.,x_big=0,y_low=1500.,y_big=0;
    for(int i =0;i<points_a.size();i++)
    {
        if(points_a[i].x<x_low) x_low = points_a[i].x;
        if(points_a[i].x>x_big) x_big = points_a[i].x;
        if(points_a[i].y<y_low) y_low = points_a[i].y;
        if(points_a[i].y>y_big) y_big = points_a[i].y;
    }

    if(x_low<point_b.x && point_b.x<x_big)
    {
        if(y_low<point_b.y && point_b.y<y_big)
        {
            return true;
        }
    }
    return false;

}

Mat detector_HOG(Mat image) {
    HOGDescriptor hog(Size(96, 36),
                      Size(12, 12),
                      Size(6, 6),
                      Size(6, 6), 3);

    Mat featureMat;
    vector<float> descriptors;
    hog.compute(image, descriptors);
    featureMat = Mat::zeros(1, descriptors.size(), CV_32FC1);

    for (int j = 0; j<descriptors.size(); j++)
    {
        featureMat.at<float>(0, j) = descriptors[j];
    }
    return featureMat;
}

void svm_pridect_HOG(Ptr<SVM> &model, Mat test, float &rst)
{
    Mat tmp;
    float validity = 0;
    tmp = detector_HOG(test);
    tmp.convertTo(tmp, CV_32F);
    rst = model->predict(tmp);
    //cout << rst << "  " <<endl;
}



namespace tdtbuff{
    /**
     * @param src
     * @return
     */
BuffDetector::BuffDetector() {
/*********************参数初始化************************/
    int src_width,src_height;

    LoadParam::ReadTheParam("EnemyColor", this->enemy_color_);
    if(disturb) {
        this->enemy_color_=2-this->enemy_color_;
    }
    LoadParam::ReadTheParam("SrcWidth",src_width);
    LoadParam::ReadTheParam("SrcHeight",src_height);
    this->src_area_ = Rect2i (Point2i (0, 0),Size2i (src_width, src_height));


}


std::vector<BuffArmor> BuffDetector::Get(cv::Mat &src)
{

std::vector<BuffArmor> armors;
std::vector<BuffArmor> ret_armors(5);//返回的vector,size为5

/***************************检测并获取装甲板**************************/
Detect(armors,src);//图像中识别所有大神符装甲

/***************************获取流水灯装甲板及已激活装甲板排序**************************/
GetFlowArmor(armors,ret_armors);

times++;

return ret_armors;

}

void BuffDetector::Detect(std::vector<BuffArmor> &armors,const cv::Mat &src)
{

    TDT_DEBUG("当前帧率：%.2f",1000000./tdttoolkit::Time::GetFrameRunTime() );

/***************************图像处理**************************/

    int gray_thre,split_thre;
    LoadParam::ReadTheParam("EnergyBuffDetect_gray_thre",gray_thre);
    LoadParam::ReadTheParam("EnergyBuffDetect_split_thre",split_thre);

    cv::Mat rgb_img[3],color_img,gray_img,bin,gray_bin;
    split(src, rgb_img);

    subtract(rgb_img[2-enemy_color_], rgb_img[enemy_color_], color_img);
    threshold(color_img, bin,split_thre, 255, cv::THRESH_BINARY);
    //转灰度图

    cvtColor(src,gray_img,COLOR_BGR2GRAY);

    threshold(gray_img, gray_bin, gray_thre, 255, cv::THRESH_BINARY);

    bin=bin&gray_bin;
    //腐蚀
    EneryArmorDetectErode(bin);
    //膨胀
    EneryArmorDetectDilate(bin);



/***************************识别条件**************************/

std::vector<cv::Vec4i> hierarchy;
std::vector<std::vector<cv::Point> > contours,pass_father_contours,no_father_contours;
findContours(bin, contours, hierarchy, RETR_TREE, cv::CHAIN_APPROX_NONE);
for(int i =0;i<contours.size();i++)
{


        
    if(hierarchy[i][3] == -1) 
        no_father_contours.emplace_back(contours[i]);

    else pass_father_contours.emplace_back(contours[i]);
}


    BuffDebug = {0,0,0,0,0};

for(const auto &pass_father_contour:pass_father_contours )
{
    CustomRect contour_rect=CustomRect(pass_father_contour);


    /***************************识别条件Debug部分**************************/
#ifdef VIDEO_DEBUG
    Debug::AddCustomRect("装甲板轮廓查找",contour_rect,cv::Scalar(0,255,0));
#endif


    /***************************识别条件**************************/
    if(contour_rect.GetSize().area() <400 || contour_rect.GetSize().area()>3500) continue;      //大概800左右      官方为2800左右
    BuffDebug.buff_armor_area=1;


    if(contour_rect.GetWidth()>200||contour_rect.GetHeight()<10)continue;     //长和宽的筛选   大概40左右   比赛  70  40 左右



#ifdef VIDEO_DEBUG
    BuffDebug.buff_height=1;
    BuffDebug.buff_width=1;
#endif

    /***************************识别圆心**************************/

    BuffArmor tmp_buff_armor(contour_rect);

    /*******************融合部分****************/

    Point3f armor_world_center;

    ArmorWorldCenter(tmp_buff_armor, armor_world_center);

    //cout<<"armor_world_center "<<armor_world_center<<endl;
    /*******************融合部分****************/

    DetectorCircle(tmp_buff_armor, no_father_contours, gray_img);//检测大神符圆心位置


    tmp_buff_armor.SetTimeNow(tdttoolkit::Time::GetTimeNow());

    if(tmp_buff_armor.GetBuffType()!=None)
    {
        armors .emplace_back( tmp_buff_armor);
    }

}


    sort(armors.begin(),armors.end(),[](BuffArmor a,BuffArmor b)->bool{ return (a.GetFlowWaterType())>(b.GetFlowWaterType());});

}

    void BuffDetector::DetectorCircle( BuffArmor &buff_armor,
                                              const std::vector<std::vector<cv::Point>> &contours,
                                              cv::Mat &gray_img) {

        bool r_big = false;
        GetRBigCustomRect(buff_armor.GetBuffRect(),r_big);


        /***************************识别条件**************************/

        for (const auto &contour:contours){
            cv::Rect tem_r_rect=boundingRect(contour);

            if(tem_r_rect.size().area()>buff_armor.GetBuffRect().GetSize ().area()/0.5 ||tem_r_rect.size().area()<buff_armor.GetBuffRect().GetSize ().area()/3)continue;  //大致为0.06 0.07左右   测量R为  2000左右  这个地方可能有问题
            cv::Point2f tmp_circle_point =(tem_r_rect.br()+tem_r_rect.tl())/2;


            if(!IsInPoints(this->r_big_rect_.GetVertices2f(),tmp_circle_point))
            {
                r_big = !r_big;
                GetRBigCustomRect(buff_armor.GetBuffRect(),r_big);

                if(!IsInPoints(this->r_big_rect_.GetVertices2f(),tmp_circle_point))
                {

                    continue;
                }
            }

#ifdef VIDEO_DEBUG
            BuffDebug.r_area=1;
            BuffDebug.r_armor_area=1;
            Debug::AddRect("R",tem_r_rect,cv::Scalar(100,255,100));
            Debug::AddCustomRect("中心轮廓",this->r_big_rect_,cv::Scalar(200,255,200));
            Debug::AddPoint("装甲板识别",tmp_circle_point,5,cv::Scalar(0,155,0));
#endif

            cv::Mat r_img=gray_img(tem_r_rect);                     //用于knn学习

            /***************************识别流水灯**************************/
            cv::Point2f center_to_circle_ = buff_armor.GetBuffRect().GetCenter2f()-tmp_circle_point;
            float rotate_angle = atan2(center_to_circle_.x,-center_to_circle_.y);

            /***************************能量机关旋转角度**************************/
            float kd_r ;
            KnnRCompute(r_img,kd_r);
            buff_armor.SetKdR(kd_r);
            buff_armor.SetRRect(tem_r_rect);
            buff_armor.SetAngle(rotate_angle);
            buff_armor.SetCirclePoint(tmp_circle_point);

            BuffType buff_type = FlowWaterLight(buff_armor,gray_img,tmp_circle_point,rotate_angle);

            if(buff_type==None){continue;}
            else if (buff_type==FlowWater){
                cv::Mat circle_img=gray_img(tem_r_rect);
                buff_armor.SetBuffType(buff_type);
                bool empty = false;
                buff_armor.SetEmpty(empty);
            }
            else if(buff_type==NoFlowWater)
            {
                buff_armor.SetBuffType(NoFlowWater);
                bool empty = false;
                buff_armor.SetEmpty(empty);
            }

            /***************************得到更好的圆心**************************/

//            float tmp_grade=100;//总分
//            float diff_distance=tdttoolkit::CalcDistance(tmp_circle_point,buff_armor.GetBuffRect().GetTr())-tdttoolkit::CalcDistance(tmp_circle_point,buff_armor.GetBuffRect().GetBr());//圆心到左右两个顶点长度之差
//            float rate1=tdttoolkit::CalcDistance(tmp_circle_point,buff_armor.GetBuffRect().GetCenter())/buff_armor.GetBuffRect().GetSize().width;//圆心到装甲板中心距离与装甲板宽之比   !!!!!!为什么用这个？？？
//            tmp_grade-=fabs(diff_distance);
//            tmp_grade-=10*kd_r;
//            tmp_grade+=8*rate1;  //得分
//            if(tmp_grade>grade){
//                grade=tmp_grade;
//                circle_center=tmp_circle_point;
//            }
        }
    }
void BuffDetector::GetFlowArmor(std::vector<tdttoolkit::BuffArmor> &armors,std::vector<tdttoolkit::BuffArmor> &ret_armors)
{

    for(BuffArmor &armor:armors) {

        if(armor.GetBuffType()==FlowWater)
        {//第0个是流水灯

            if(armor.GetFlowWaterType() == 1)
            {

                ret_armors[0] = armor;
                break;
            }
            else
            {
                if (fabs(armor.GetBuffRect().GetSize ().area()/last_armor_area-1)<0.1&&fabs(last_radius/CalcDistance (armor.GetBuffRect().GetCenter(),armor.GetCirclePoint())-1)<0.1)
                {
                    ret_armors[0] = armor;
                    break;
                }
                else{continue;}
            }

        }
    }

    last_radius =CalcDistance (ret_armors[0].GetCirclePoint(),ret_armors[0].GetBuffRect().GetCenter());
    last_armor_area=ret_armors[0].GetBuffRect().GetSize ().area();
    if(ret_armors[0].GetBuffType ()==FlowWater)
    {//如果检测到了流水灯
        for(BuffArmor &armor:armors)
        {
            if(armor.GetCirclePoint()!=ret_armors[0].GetCirclePoint())continue;
            if(armor.GetBuffType()!=NoFlowWater)continue;//遍历已经激活过的装甲板
            int i_dex;
            float diff_angle=armor.GetAngle()-ret_armors[0].GetAngle();//通过角度差按顺时针顺序排序
            if(diff_angle>0)
            {//获得真正的角度差
                i_dex=cvRound(diff_angle/(2*CV_PI/5));
            }
            else
            {
                i_dex=5+cvRound(diff_angle/(2*CV_PI/5));
            }
            if(i_dex<1||i_dex>4)    continue;
            tdtlog::Log::ChannelLog("BuffDetector", "现在:%d帧", tdttoolkit::Time::GetFrameId());
            tdtlog::Log::ChannelLog("BuffDetector", "找到%d个能量机关", ret_armors.size());
            ret_armors[i_dex]=armor;

        }
    }
#ifdef VIDEO_DEBUG
    Debug::SetMessage("能量机关检测","装甲板面积",BuffDebug.buff_armor_area);
    Debug::SetMessage("能量机关检测","装甲板高度",BuffDebug.buff_height);
    Debug::SetMessage("能量机关检测","装甲板宽度",BuffDebug.buff_width);
    Debug::SetMessage("R的识别","r的面积",BuffDebug.r_area);
    Debug::SetMessage("R的识别","装甲板面积",BuffDebug.r_armor_area);
    Debug::SetMessage("流水灯识别","knn_r",ret_armors[0].GetKdR());
    Debug::SetMessage("流水灯识别","svm_flow_water",ret_armors[0].GetFlowWaterType());
    //for(int i =0;i<5;i++) {
            Debug::AddCustomRect("装甲板识别",ret_armors[0].GetBuffRect(),cv::Scalar(0,255,0));
    //}
#endif
    first_times_=true;


}

BuffType BuffDetector::FlowWaterLight(
            BuffArmor &buff_armor ,
            const cv::Mat& gray_img,
            const cv::Point2f &circle_point,
            float &rotate_angle)
    {

        cv::Mat armor_img,roi_img;
        //流水灯矩形构建
        //圈出流水灯矩形，并进行仿射变换到水平
        /***************************流水灯仿射变换**************************/
        GetFlowWaterRect(circle_point,buff_armor.GetBuffRect());

        if(RectSafety(bounding_rect_,src_area_.size()))
        {return None;
        }
        cv::Mat black_ground = gray_img(bounding_rect_);

        Mat rotation_mat = getRotationMatrix2D(Point(flow_rect_.GetCenter())-bounding_rect_.tl(),(rotate_angle+CV_PI/2)*180/CV_PI, 1);//仿射变换
        warpAffine(black_ground,armor_img,rotation_mat,black_ground.size());


        Size size2 = cv::Size2i((int)flow_rect_.GetWidth(),(int)(flow_rect_.GetHeight()*1.2));
        Rect2i real_size_rect(flow_rect_.GetCenter()-bounding_rect_.tl()-Point(flow_rect_.GetSize())/2,size2);

    if(!RectSafety(real_size_rect,armor_img.size()))
        {  armor_img = armor_img(real_size_rect);}
        else{return None;}

    //提取样本
        if(0)
        {
            if(armor_img.empty()) return None;
            stringstream str;
            str << times << ".jpg";
            cout << str.str() << endl;
            imwrite("/home/jarvis/图片/sample/SVMbuffflowrect3/" + str.str(), armor_img);
        }

        /***************************流水灯knn计算**************************/

//        float knn_flow_water;
//        KnnFlowCompute(armor_img,knn_flow_water);
//        buff_armor.SetKdFlowwater(knn_flow_water);        //原理应该是：每个像素点或每个块距每k个相同类型的元素的像素或特点的差值。

        /***************************流水灯轮廓计算**************************/


    /***************************SVM**************************/

    resize(armor_img, armor_img, Size(96, 36));

    float flow_water_type;

    svm_pridect_HOG(svm, armor_img, flow_water_type);  //与正样本相符为1，反之为0

    buff_armor.SetFlowWaterType(flow_water_type);
    /***************************SVM**************************/

        float small_contour,big_contour;

        FlowWaterContour(armor_img,bounding_rect_,small_contour,big_contour);



#ifdef VIDEO_DEBUG
        Debug::SetMessage("流水灯识别","旋转角度",rotate_angle);
        Debug::SetMessage("流水灯识别","小轮廓数量",small_contour);
        Debug::SetMessage("流水灯识别","大轮廓数量",big_contour);
#endif


        ///用于找小流水灯的数量
//    if(knn_flow_water<10){
//            return BuffType (FlowWater);
//        }
//        if(big_contour>0)
//        {
//            return BuffType (NoFlowWater);
//        }
//        else
//        {
//            if(small_contour>4)
//            {
//
//                return BuffType (FlowWater);
//            }
//            else
//            {
//                return BuffType (NoFlowWater);
//            }
//
//        }

        if(flow_water_type == 1)
        {

            return BuffType (FlowWater);
        }
//        if(big_contour>0)
//        {
//            return BuffType (NoFlowWater);
//        }
//        else
//        {
//            if(small_contour>4)
//            {
//
//                return BuffType (FlowWater);
//            }
//            else
//            {
//                return BuffType (NoFlowWater);
//            }
//
//        }
        else
        {
            return BuffType (NoFlowWater);
        }

    }
void BuffDetector::GetRBigCustomRect(const CustomRect &contour_rect,bool order)
{
    cv::Point2f side_diff;
    if(contour_rect.GetAngle()<135 &&contour_rect.GetAngle()>=45)
    {
        side_diff = (contour_rect.GetTr()-contour_rect.GetTl())*contour_rect.GetSize().width/contour_rect.GetSize().height;

    }
    else{
        side_diff = (contour_rect.GetTr()-contour_rect.GetBr())*contour_rect.GetSize().width/contour_rect.GetSize().height;
    }

    float rect_angle = contour_rect.GetAngle() > 90 ?(contour_rect.GetAngle()-90):(contour_rect.GetAngle()+90);
    if(order)
    {
        this->r_big_rect_ = CustomRect(contour_rect.GetCenter2f()+4.2*side_diff,cv::Size2f(contour_rect.GetSize().width*1.5,contour_rect.GetSize().width*1.5),rect_angle);      //学校：3.6
    }
    else{
        this->r_big_rect_ = CustomRect(contour_rect.GetCenter2f()-4.2*side_diff,cv::Size2f(contour_rect.GetSize().width*1.5,contour_rect.GetSize().width*1.5),rect_angle);

    }
}
void BuffDetector::GetFlowWaterRect(const cv::Point2f &circle_point,const CustomRect &custom_rect)
    {

        cv::Point2f flow_center = circle_point*4/9+custom_rect.GetCenter2f()*5/9;

        float flow_rect_angle = custom_rect.GetAngle()>90? (custom_rect.GetAngle()-90):(custom_rect.GetAngle()+90);
        this->flow_rect_ = CustomRect(flow_center,cv::Size2f(custom_rect.GetSize().width * 3.0,custom_rect.GetSize().width/1.6),flow_rect_angle);
        bounding_rect_ = cv::Rect((int)(flow_rect_.GetCenter2f().x-flow_rect_.GetWidth()/2),(int)(flow_rect_.GetCenter2f().y-flow_rect_.GetWidth()/2),(int)flow_rect_.GetWidth(),(int)(flow_rect_.GetWidth()));
#ifdef VIDEO_DEBUG
        Debug::AddRect("流水灯轮廓",bounding_rect_);
#endif


    }
void BuffDetector::KnnRCompute(cv::Mat &r_img, float &knn_r)
{
    std::vector<float> descriptors;
    resize(r_img, r_img, cv::Size(28,28));
    //knn学习并计算
    r_hog->compute(r_img, descriptors);
    cv::Mat des=cv::Mat(descriptors);
    transpose(des,des);
    cv::Mat dist;
    knn_Model_r->findNearest(des,1,cv::noArray(),cv::noArray(),dist);
    knn_r = dist.at<float>(0,0);
}
void BuffDetector::KnnFlowCompute(cv::Mat &armor_img,float &knn_flow_water)
{
    resize(armor_img,armor_img,cv::Size(96,28));

    //knn进行流水灯的判断
    std::vector<float> descriptors;
    flow_water_hog->compute(armor_img, descriptors);
    cv::Mat des=cv::Mat(descriptors);
    transpose(des,des);
    cv::Mat dist;
    knn_Model_flow_water->findNearest(des,1,cv::noArray(),cv::noArray(),dist);
    knn_flow_water = dist.at<float>(0,0)*6.5f;
}
void BuffDetector::FlowWaterContour(cv::Mat &armor_img,cv::Rect &bounding_rect,float &small_contour,float &big_contour)
{
    cv::Mat flow_water_image;

    threshold(armor_img, flow_water_image,0 ,255,cv::THRESH_OTSU);
    erode(flow_water_image,flow_water_image,getStructuringElement(cv::MORPH_RECT,cv::Size(4,2)));//腐蚀
    std::vector<std::vector<cv::Point> > contours;
    findContours(flow_water_image, contours, RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    float  rh=96;
    for(auto &flow_contour:contours){
        cv::Rect t_rect=boundingRect(flow_contour);
        if(t_rect.size().area()>bounding_rect.size().area()/3&&t_rect.size().width>rh*0.7)
        {
            big_contour+=1;
        }
        else
        {
            if(t_rect.size().height>rh/12&&t_rect.size().height<rh/4 && 1.2*t_rect.size().height>t_rect.size().width){
                small_contour++;}
            else{   continue;   }
        }
    }

}

/************************识别的解算...**************************/


    void BuffDetector::ArmorWorldCenter( tdttoolkit::BuffArmor &buff_armor, Point3f &armor_world_center)
    {
        std::vector<cv::Point2f>point2D;
        std::vector<cv::Point3f>point3D;

        ///四个点为装甲板的四个顶角
        GetPoint2D(buff_armor,point2D);
        GetPoint3D(buff_armor,point3D);
        SimplePnpResolve(point2D,point3D,buff_armor, armor_world_center);


    }

    void BuffDetector::GetPoint2D( tdttoolkit::BuffArmor &buff_armor, std::vector<cv::Point2f>&point2D){

        cv::Point2f Tl,Tr,Br,Bl;

        Tr = buff_armor.GetBuffRect().GetTr();   //TODO 平行
        Tl = buff_armor.GetBuffRect().GetTl();
        Bl = buff_armor.GetBuffRect().GetBl();
        Br = buff_armor.GetBuffRect().GetBr();

        point2D.clear();///先清空再存入
        point2D.emplace_back(Tr);
        point2D.emplace_back(Tl);
        point2D.emplace_back(Bl);
        point2D.emplace_back(Br);
    }

//矩形转换为3d坐标                                                                                                                                                                                             3
    void BuffDetector::GetPoint3D( tdttoolkit::BuffArmor &buff_armor, std::vector<cv::Point3f>&point3D)
    {

        float fHalfX=0;
        float fHalfY=0;

        float BuffWidth = 23.0;
        float BuffHeight = 12.7;

        fHalfX=BuffWidth/2.0;
        fHalfY=BuffHeight/2.0;

        point3D.emplace_back(cv::Point3f(-fHalfX,-fHalfY,0.0));
        point3D.emplace_back(cv::Point3f(fHalfX,-fHalfY,0.0));
        point3D.emplace_back(cv::Point3f(fHalfX,fHalfY,0.0));
        point3D.emplace_back(cv::Point3f(-fHalfX,fHalfY,0.0));

    }

    void BuffDetector::SimplePnpResolve(const std::vector<cv::Point2f>&point2D,
                      const std::vector<cv::Point3f>&point3D,
                      tdttoolkit::BuffArmor &buff_armor,
                      Point3f &armor_world_center)
    {
        cv::Mat armor_rvec_object_to_camera=cv::Mat::zeros(3,1,CV_64FC1);
        cv::Mat armor_tvec_object_in_camera=cv::Mat::zeros(3,1,CV_64FC1);
        cv::Mat known_caremaMatrix = (cv::Mat_<float>(3, 3) <<
                                                            1782.062074551373, 0, 707.967021037012,
            0, 1782.451599261389, 520.228531737792,
            0, 0, 1);
        cv::Mat known_distCoeffs = (cv::Mat_<float>(1, 5) <<
                                                          -0.566621424984894,
            0.2346071214970684,
            -0.002206135838931192,
            -0.0004793719647413983,
            0.7720103795048127);
        //cv::Mat armor_rvec_object_to_camera, armor_tvec_object_in_camera;
//     cv::solvePnP(point3D,point2D,caremaMatrix,distCoeffs,rvecs,tvecs,false,SOLVEPNP_UPNP);
        solvePnP (point3D, point2D, known_caremaMatrix, known_distCoeffs, armor_rvec_object_to_camera,
                  armor_tvec_object_in_camera, false);

        pnp_rvec_object_to_world_0 = armor_rvec_object_to_camera.clone();
        pnp_rvec_object_to_world_0.convertTo(pnp_rvec_object_to_world_0, CV_32F);
        pnp_tvec_object_to_world_0 = armor_tvec_object_in_camera.clone();



        /**********/  //TODO 封装为函数
        cv::Point3f object_in_camera = cv::Point3f (*(armor_tvec_object_in_camera.ptr<cv::Point3d> ()));
        Polar3f object_in_camera_polar = tdttoolkit::RectangularToPolar(object_in_camera);

        object_in_camera_polar.distance = 700;
        object_in_camera = tdttoolkit::PolarToRectangular (object_in_camera_polar);
        armor_tvec_object_in_camera = cv::Mat (cv::Point3f (object_in_camera));
        /**********/

        float tx = armor_tvec_object_in_camera.ptr<float>(0)[0];
        float ty = -armor_tvec_object_in_camera.ptr<float>(0)[1];
        float tz = armor_tvec_object_in_camera.ptr<float>(0)[2];

        armor_world_center.x = tx;
        armor_world_center.y = ty;
        armor_world_center.z = tz;

    }



/************************识别的解算...**************************/



}

