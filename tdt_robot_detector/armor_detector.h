#ifndef TDT_VISION_RM2020_ARMOR_DETECTOR_H
#define TDT_VISION_RM2020_ARMOR_DETECTOR_H

#include <opencv2/opencv.hpp>
#include "tdtcommon.h"
namespace tdtrobot
{
    class ArmorDetector
    {
    private: //数据类型声明
        class LightBarInfo : public tdttoolkit::CustomRect
        {
        public:
            inline LightBarInfo() = default ;
            inline explicit LightBarInfo(const tdttoolkit::CustomRect& rot_rect) : CustomRect(rot_rect){ this -> find_status_ = 0; };
            inline int GetFindStatus() const { return find_status_; };
            inline void SetFindStatus(int find_status) { this -> find_status_ = find_status; };
        private:
            //标志该灯条的左右两边是否被搜索过, -1代表左边被搜索过, 1代表右边, 0代表没有搜索过, 2代表全部搜索过
            int find_status_ = 0;
        };
        class ArmorDetectInfo
        {
        public:
            ArmorDetectInfo() = default;
            ArmorDetectInfo(const tdttoolkit::CustomRect &number_rot_rect, int threshold)
            {
                this -> threshold_ = threshold;
                this -> number_rot_rect_ = number_rot_rect;
                this -> angle = number_rot_rect.GetAngle();
            }
            ArmorDetectInfo(const LightBarInfo &signal_light_bar, const tdttoolkit::CustomRect &number_rot_rect, int lr, int threshold)
            {
                this -> threshold_ = threshold;
                this -> number_rot_rect_ = number_rot_rect;
                this -> angle = number_rot_rect.GetAngle();
                if(lr == -1)
                {
                    this -> have_right_bar_ = true;
                    this -> right_bar_rect_ = tdttoolkit::CustomRect(signal_light_bar); // NOLINT(cppcoreguidelines-slicing) //强制类型转换,丢掉查找状态信息
                    return;
                }
                this -> have_left_bar_ = true;
                this -> left_bar_rect_ = tdttoolkit::CustomRect(signal_light_bar); // NOLINT(cppcoreguidelines-slicing) //强制类型转换,丢掉查找状态信息
            }
            ArmorDetectInfo(const LightBarInfo &left_light_bar, const LightBarInfo &right_light_bar, const tdttoolkit::CustomRect &number_rot_rect, int threshold)
            {
                this -> number_rot_rect_ = number_rot_rect;
                this -> angle = number_rot_rect.GetAngle();
                this -> have_left_bar_ = true;
                this -> have_right_bar_ = true;
                this -> threshold_ = threshold;
                this -> right_bar_rect_ = tdttoolkit::CustomRect(right_light_bar); // NOLINT(cppcoreguidelines-slicing) //强制类型转换,丢掉查找状态信息
                this -> left_bar_rect_ = tdttoolkit::CustomRect(left_light_bar); // NOLINT(cppcoreguidelines-slicing) //强制类型转换,丢掉查找状态信息
            }
            inline tdttoolkit::RobotType GetArmorType() const { return this -> armor_type_; };
            inline cv::Rect2i GetRect() const { return this -> GetNumberRotRect().GetRect(); };
            inline int GetThreshold() const { return this -> threshold_; };
            inline float GetAngle () const { return this -> angle; };
            inline tdttoolkit::CustomRect GetNumberRotRect() const { return this -> number_rot_rect_; };
            inline tdttoolkit::CustomRect GetLeftBarRect() const { return this -> left_bar_rect_; };
            inline tdttoolkit::CustomRect GetRightBarRect() const { return this -> right_bar_rect_; };
            inline tdttoolkit::CustomRect GetArmorRotRect() const {return this -> armor_rot_rect_; };
            inline bool HaveRightBar() const { return this -> have_right_bar_; };
            inline bool HaveLeftBar() const { return this -> have_left_bar_; };

            inline void SetArmorRotRect(const tdttoolkit::CustomRect &armor_rot_rect) { this -> armor_rot_rect_ = armor_rot_rect; };
            inline void SetArmorType(tdttoolkit::RobotType armor_type) { this -> armor_type_ = armor_type; };
        private:
            tdttoolkit::RobotType armor_type_ = tdttoolkit::RobotType::TYPEUNKNOW;
            tdttoolkit::CustomRect number_rot_rect_ = tdttoolkit::CustomRect(cv::Point2i(0, 0), cv::Size2i(0, 0), 0);
            tdttoolkit::CustomRect left_bar_rect_ = tdttoolkit::CustomRect(cv::Point2i(0, 0), cv::Size2i(0, 0), 0);
            tdttoolkit::CustomRect right_bar_rect_ = tdttoolkit::CustomRect(cv::Point2i(0, 0), cv::Size2i(0, 0), 0);
            tdttoolkit::CustomRect armor_rot_rect_ = tdttoolkit::CustomRect(cv::Point2i(0, 0), cv::Size2i(0, 0), 0);

            bool have_right_bar_ = false;
            bool have_left_bar_ = false;
            int threshold_ = 0;

            float angle=0;
        };




    private: //工具
        /**
         * @brief 保证rect不出src边界
         * @note 需要this -> src_area_
         * @param rect 待判断的矩形, 如果有一部分出界,则返回其在src范围内的部分
         * @return 如果rect与src范围没有交际,返回false,否则返回true
         */
        inline bool RectSafety(cv::Rect2i &rect);

        /**
         * @brief rect中心点不变, 大小扩大到原来的gain倍(width*width, height*height), 且不超出src范围
         * @note 调用RectSafety
         * @param rect 待放大的矩形
         * @param gain 放大倍数
         */
        inline cv::Rect2i RectEnlarge(const cv::Rect2i &rect, const cv::Size2i &gain);
    public:
        /**
         * 完成整个检测器的初始化工作
         * @param camera
         */
        explicit ArmorDetector();

        /***
          *  @brief     装甲识别调用函数
          *  @input    src  相机输入图像
          *  @input    ReceiveMessage  下位机输入信息
          *  @output   Armor    通过装甲识别的装甲板（如果没有则返回一个空装甲板） 自找同一类型
          *  @author   李思祁
          ***/
        std::vector<tdttoolkit::RobotArmor> Get(cv::Mat &src, const tdttoolkit::ReceiveMessage &receive_message);//之后要加入下位机的输入

    public:
        std::vector<LightBarInfo> trace_light_bars_;//用于提高跟踪效果
    private: //初始化后不变的量
        cv::Rect2i src_area_;  //原图范围,初始化之后不得修改
         int enemy_color_;  //敌方颜色, 按照cv颜色通道顺序,0代表蓝, 1代表绿, 2代表红
        int threshold_ = 0;
        cv::Mat element_;  //滤波处理核心
        cv::Mat element2_;

    private: //每次运行结束更新的量
        std::vector<ArmorDetectInfo> last_armors_info_;  //上一帧识别到的装甲板中离图像中心最近的那个
        tdttoolkit::RobotType last_robot_type_=tdttoolkit::RobotType::TYPEUNKNOW; //上一次（区别于上一帧）识别出来的机器人类型

    private: //每次运行前更新的量
        cv::Rect2i armor_detect_roi_ ;  //用于检测装甲板的ROI区域
        bool is_locking_status_ = false; //用于记录是否锁


        /**
         * @name 跟踪
         * @brief 根据上一帧识别出的装甲板的周围区域来识别
         * @brief 仅使用灯条匹配，不使用数字识别
         * @note 效果很不好，会影响后续的预测，弃用
         */
        void track(const cv::Mat &src, std::vector<tdttoolkit::RobotArmor> &armors);
        void FindLightBar_track(const cv::Mat &src, std::vector<LightBarInfo> &output_light_bars,const cv::Point2i &point);



        /**
         * @name 取ROI区域
         * @brief 从上一帧识别的装甲板周围取一个区域来作为下一帧识别的ROI区域
         * @note 结果存储在armor_detect_roi_中, 与该函数有关的其他成员: last_armor_, src_area_
         */
        void RoiFilter();

        /**
         * @name 决策
         * @brief 将装甲板以与ROI中心点的距离从近到远排序
         * @brief 如果传入装甲板容器为空, 这返回空容器
         * @brief 如果传入装甲板容器只有工程, 则返回所有工程装甲板
         * @brief 如果传入装甲板容器有不止工程, 则返回距离ROI区域中心最近的非工程装甲板的容器
         * @param input_armors 装甲板的容器, 执行完函数其中装甲板会以距离ROI区域中心点距离中由近到远排列
         * @param output_armor 返回的装甲板容器
         */
        void Decide(std::vector<ArmorDetectInfo> &input_armors, std::vector<ArmorDetectInfo> &output_armor);

        /**
         * @name 数字识别
         * @brief 识别装甲板上的数字, 设给装甲板
         * @param src 原图
         * @param armors 装甲板们
         */
        void NumPredict(const cv::Mat &src, std::vector<ArmorDetectInfo> &armors);

        /**
         * @name 查找灯条
         * @brief 在src图像内查找灯条, 输出找到的灯条数组, 并按照从左到右排序
         * @param roi 图像
         * @param output_light_bars 灯条数组
         * @todo 引入knn, 进行灯条匹配
         */
        void FindLightBar(const cv::Mat &src, std::vector<LightBarInfo> &output_light_bars);

        /**
         * @name 单灯条检测
         * @brief 功能根据单个灯条去查找是非有疑似装甲板的区域, 输入的灯条以从左到右排列
         * @param src 原图像
         * @param light_bars 灯条, 需要以从左到右排列
         * @param output_armors 输出疑似装甲板的区域的数据包
         */
        void SingleLightBarDetect(const cv::Mat &src, std::vector<LightBarInfo> &light_bars, std::vector<ArmorDetectInfo> &output_armors);

        /**
         * @name 双灯条查找
         * @brief 功能根据两个灯条去查找是非有疑似装甲板的区域, 输入的灯条以从左到右排列
         * @param src 原图像
         * @param light_bars 灯条, 需要以从左到右排列
         * @param output_armors 输出疑似装甲板的区域的数据包
         */
        void DoubleLightBarDetect(const cv::Mat &src, std::vector<LightBarInfo> &light_bars, std::vector<ArmorDetectInfo> &output_armors);

        /**
         * @name 去处重复区域
         * @brief 防止由于光线干扰、画面撕裂等情况造成一个装甲板识别两次或误识别
         * @param output_armors 输出疑似装甲板的区域的数据包
         */
        void EraseDuplicate(std::vector<ArmorDetectInfo> &output_armors);

        /**
         * @name 数字贴纸检测
         * @brief 根据上一帧的装甲板位置进行无灯条/单灯条检测
         * @param src 原图像
         * @param output_armors 输出疑似装甲板的区域的数据包
         */
        void NumberStickerDetect(const cv::Mat &src, std::vector<ArmorDetectInfo> &output_armors);

        ////////////////////////////// 无情的工具函数 //////////////////////////////

        /**
         * @name 灯条是否匹配
         * @brief 判断左灯条与右灯条是否匹配
         * @param left_light_bar 左灯条
         * @param right_light_bar 右灯条
         * @return 匹配返回true,不匹配返回false
         */
        static bool IsLightBarMatched(const LightBarInfo &left_light_bar, const LightBarInfo &right_light_bar);

        /**
         * @name 大津法估算阈值
         * @param input_image 输入图像
         * @param output_image 输出处理后的图像
         * @param lr 在灯条左边还是右边, 用来确定数字的大概位置(-1表示该区域在灯条左边, 1表示该区域在灯条右边, 0表示在中间)
         * @return 返回大津法的阈值
         */
        static int RegionOustThreshold(const cv::Mat &input_image, cv::Mat &output_image, int lr);

        /**
         * @name 确定机器学习区域
         */
        static void CalaArmorRotRect(ArmorDetectInfo &armor);

        /**
         * @name 设置机器学习的ROI区域
         * @brief 为装甲板设置机器学习ROI区域
         * @param armor 装甲板
         * @param src 原图像
         */
        void GetMlRoi(const ArmorDetectInfo &armor, const cv::Mat &src, cv::Mat &ml_roi);

        /**
         * @name 装甲板接近?
         * @brief 判断两个装甲板是否足够靠近
         * @param armor_a a装甲板
         * @param armor_b b装甲板
         * @return 接近返回
         */
        static bool IsArmorNearby(const ArmorDetectInfo &armor_a, const ArmorDetectInfo &armor_b);

        static bool IsArmorSimilar(const ArmorDetectInfo &armor_a, const ArmorDetectInfo &armor_b);

        static void ArmorTransform(const ArmorDetectInfo &armor_info, tdttoolkit::RobotArmor &armor);

        ////////////////////////////////////////////////暂时没用///////////////////////////////////
        /**
         * @name 匹配上一帧装甲板
         * @brief 通过装甲板位置将这一帧装甲板与上一帧装甲板进行匹配,如果全部匹配成功则按上一帧装甲板顺序返回装甲板, 并将这一帧匹配到的装甲板类型设置为上一帧的类型, 如果失败返回空容器
         * @param input_armors 这一帧识别到的全部装甲板
         * @param output_armors 与上一帧匹配的装甲板, 失败返回空容器
         */
        void MatchWithLastArmor(const std::vector<ArmorDetectInfo> &input_armors, std::vector<ArmorDetectInfo> &output_armors);

        //void GetNearByArmor(const std::vector<Armor> &input_armors, std::vector<Armor> &output_armors);



    };
}

#endif //TDT_VISION_RM2020_ARMOR_DETECTOR_H
