#include <auto_aim_common/Armor.h>
#include <auto_aim_common/Armors.h>
#include <gimbal_driver/GimbalAngles.h>
#include <auto_aim_common/AngleImage.h>
#include <Logger/Logger.hpp>
#include <RosTools/RosTools.hpp>

#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt8.h>

#include <boost/lockfree/stack.hpp>
#include <boost/atomic.hpp>

#include "armor_detector/armor_filter.hpp"
#include "armor_detector/armor_refinder.hpp"
#include "armor_detector/pnp_solver.hpp"
#include "car_and_armor_detector.hpp"
#include "car_detector/car_finder.hpp"
#include <auto_aim_common/DetectionType.hpp>
#include <VideoStreamer/VideoStreamer.hpp>
#include "module/Camera.hpp"
using namespace ly_auto_aim;
using namespace LangYa;
// using boost::lockfree;
namespace {

LY_DEF_ROS_TOPIC(ly_aa_enable, "/ly/aa/enable", std_msgs::Bool);
LY_DEF_ROS_TOPIC(ly_ra_enable, "/ly/ra/enable", std_msgs::Bool);
LY_DEF_ROS_TOPIC(ly_outpost_enable, "/ly/outpost/enable", std_msgs::Bool);
LY_DEF_ROS_TOPIC(ly_camera_image, "/ly/camera/image", sensor_msgs::Image);
LY_DEF_ROS_TOPIC(ly_backcamera_image, "/ly/backcamera/image", sensor_msgs::Image);
LY_DEF_ROS_TOPIC(ly_me_is_team_red, "/ly/me/is_team_red", std_msgs::Bool);
LY_DEF_ROS_TOPIC(ly_bt_target, "/ly/bt/target", std_msgs::UInt8);
LY_DEF_ROS_TOPIC(ly_detector_armors, "/ly/detector/armors", auto_aim_common::Armors);
LY_DEF_ROS_TOPIC(ly_gimbal_angles, "/ly/gimbal/angles", gimbal_driver::GimbalAngles);
LY_DEF_ROS_TOPIC(ly_compressed_image, "/ly/compressed/image", sensor_msgs::CompressedImage);
// LY_DEF_ROS_TOPIC(ly_ra_image, "/ly/ra/image", sensor_msgs::Image);
LY_DEF_ROS_TOPIC(ly_ra_angle_image, "/ly/ra/angle_image", auto_aim_common::AngleImage);
LY_DEF_ROS_TOPIC(ly_outpost_armors, "/ly/outpost/armors", auto_aim_common::Armors);

/// 配置变量
bool pub_image = true;
bool web_show = true;
bool draw_image = true;
bool debug_mode = false;
bool debug_team_blue = false;
bool save_video = false;
bool use_video = false;
bool use_ros_bag = false;
std::string video_path;

/// 宏常量与状态量
std::atomic_bool myTeamRed{false};
std::atomic_bool aa_enable{true};
std::atomic_bool ra_enable{false};
std::atomic_bool outpost_enable{false};

constexpr const char AppName[] = "detector";
ROSNode<AppName> global_node;
// CorrectedDetector carAndArmorDetector{};
CarAndArmorDetector carAndArmorDetector{};
ArmorFilter filter{};
ArmorRefinder finder{};
CarFinder carFinder{};
CameraIntrinsicsParameterPack cameraIntrinsics{};
PoseSolver solver{cameraIntrinsics};
Camera Cam;
Camera SubCam;


using AngleType = float;
using Angle100Type = std::int16_t;
struct GimbalAnglesType {
    AngleType yaw{0.0f};
    AngleType pitch{0.0f};
}; // follow auto aim data structure

#pragma region image_queue
class ImageQueue {
private:
    std::queue<cv::Mat> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
    const size_t max_size_ = 10;  // 队列最大容量

public:
    void push(cv::Mat image) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= max_size_) {
            queue_.pop();  // 移除最旧图像
        }
        queue_.push(image.clone());  // 必须深拷贝
        cond_.notify_all();  // 通知消费者线程
    }

    cv::Mat wait_and_pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]{ return !queue_.empty(); });
        auto image = queue_.front();
        queue_.pop();
        return image;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
};
#pragma endregion image_quene

#pragma region image_stack
constexpr size_t MAX_STACK_SIZE = 30; // 按30Hz*1秒容量设计
#pragma endregion image_stack

std::atomic<AngleType> gimbal_angles_yaw;
std::atomic<AngleType> gimbal_angles_pitch;

/// 接受决策端的数据包
std::atomic<ArmorType> atomic_target{};

/// 发送的数据包
PNPAimResult aim_result{};

void InitialParam(){
    global_node.GetParam<bool>("detector_config/show", pub_image, true);
    global_node.GetParam<bool>("detector_config/draw", draw_image, true);
    global_node.GetParam<bool>("detector_config/debug_mode", debug_mode, false);
    global_node.GetParam<bool>("detector_config/debug_team_blue", debug_team_blue, false);
    global_node.GetParam<bool>("detector_config/save_video", save_video, false);
    global_node.GetParam<bool>("detector_config/web_show", web_show, true);
    global_node.GetParam<bool>("detector_config/use_video", use_video, false);
    global_node.GetParam<bool>("detector_config/use_ros_bag", use_ros_bag, false);
    global_node.GetParam<std::string>("detector_config/video_path", video_path, "");
}

void ConfigureCamera() {
    auto &config = Cam.Configure();
    config.AutoExposure.Value = GX_EXPOSURE_AUTO_OFF;
    global_node.GetParam<double>("camera_param/ExposureTime", config.ExposureTime.Value, 4000);
    config.AutoGain.Value = GX_GAIN_AUTO_OFF;
    global_node.GetParam<double>("camera_param/Gain", config.Gain.Value, 12);
    global_node.GetParam<double>("camera_param/RedBalanceRatio", config.RedBalanceRatio.Value, 1.2266f);
    global_node.GetParam<double>("camera_param/GreenBalanceRatio", config.GreenBalanceRatio.Value, 1.0f);
    global_node.GetParam<double>("camera_param/BlueBalanceRatio", config.BlueBalanceRatio.Value, 1.3711f);
}

void ConfigureSubCamera() {
    auto &config = SubCam.Configure();
    config.AutoExposure.Value = GX_EXPOSURE_AUTO_OFF;
    global_node.GetParam<double>("sub_camera_param/ExposureTime", config.ExposureTime.Value, 2000);
    config.AutoGain.Value = GX_GAIN_AUTO_OFF;
    global_node.GetParam<double>("sub_camera_param/Gain", config.Gain.Value, 24);
    global_node.GetParam<double>("sub_camera_param/RedBalanceRatio", config.RedBalanceRatio.Value, 1.7852f);
    global_node.GetParam<double>("sub_camera_param/GreenBalanceRatio", config.GreenBalanceRatio.Value, 1.0f);
    global_node.GetParam<double>("sub_camera_param/BlueBalanceRatio", config.BlueBalanceRatio.Value, 2.0898f);
}

void my_team_callback(const std_msgs::Bool::ConstPtr &msg) {
    myTeamRed = msg->data;
}

void gimbal_callback(const gimbal_driver::GimbalAngles::ConstPtr &msg) {
    gimbal_angles_yaw = msg->Yaw;
    gimbal_angles_pitch = msg->Pitch;
}

void get_target_callback(const std_msgs::UInt8::ConstPtr &msg) {
    atomic_target = static_cast<ArmorType>(msg->data);
}

void aa_enable_callback(const std_msgs::Bool::ConstPtr &msg) {
    aa_enable = msg->data;
}

void ra_enable_callback(const std_msgs::Bool::ConstPtr &msg) {
    ra_enable = msg->data;
}

void outpost_enable_callback(const std_msgs::Bool::ConstPtr &msg) {
    outpost_enable = msg->data;
}

void DrawArmor(cv::Mat &image, const ArmorObject &armor) {
    for (const auto &point : armor.apex) {
        cv::circle(image, point, 5, cv::Scalar(0, 0, 255), -1);
    }
    
    cv::line(image, armor.apex[0], armor.apex[2], cv::Scalar(255, 0, 0), 2);
    cv::line(image, armor.apex[1], armor.apex[3], cv::Scalar(0, 255, 0), 2);

    const std::string type_text = std::to_string(armor.type);
    cv::Point text_org = armor.apex[0] + cv::Point2f(10, 30); 
    cv::putText(image, type_text, text_org, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
}

void DrawCarBBox(cv::Mat& image, const CarDetection& car){
    cv::rectangle(image, car.bounding_rect, cv::Scalar(0, 255, 0), 2);
}

void DrawAllArmor(cv::Mat &image, const std::vector<ArmorObject> &armors) {
    for (const auto &armor : armors) {
        DrawArmor(image, armor);
    }
}

/// @brief  后面可以在添加一下car_id car_type之类的
/// @param image 
/// @param cars 
void DrawAllCar(cv::Mat &image, const std::vector<CarDetection> &cars) {
    for (const auto &car : cars) {
        DrawCarBBox(image, car);
    }
}

struct TimedArmors {
    ros::Time TimeStamp;
    std::vector<ArmorObject> Armors;
    GimbalAnglesType TimeAngles{};
};

/// @brief  用于对齐打符的云台和图像
struct AngleFrame{
    cv::Mat image;
    GimbalAnglesType angles;
};

//// 迫于使用rosbag的压力，将image全局持久化
ImageQueue callbackQueue;
void ImageLoop() {
    cv::Mat image;
    cv::Mat sub_image;
    cv::Mat concat_image;
    ImageQueue image_queue;
    boost::lockfree::stack<AngleFrame> angle_image_stack(MAX_STACK_SIZE);
  
    std::jthread subcamera_thread{[&]{
        auto begin = std::chrono::steady_clock::now();
 	    ros::Rate rate(10);
        std::size_t image_count{0};
        while (ros::ok()) {
            cv::Mat tmp_image;
            if (!SubCam.GetImage(tmp_image)) continue;
            sub_image = tmp_image.clone();
            /// 处理拼接
            if (!image.empty() && !sub_image.empty()) {
                cv::Size target_size(image.cols, image.rows);
                cv::resize(sub_image, sub_image, target_size, 0, 0, cv::INTER_LINEAR);
                cv::hconcat(image, sub_image, concat_image);
            }
            image_count++;
            auto now = std::chrono::steady_clock::now();
            const auto diff_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - begin);
            if (diff_ms > std::chrono::seconds(3)) {
                roslog::info("image rate: {}", float(image_count) / diff_ms.count());
                begin = now;
                image_count = 0;
            }
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", tmp_image).toImageMsg();
                msg->header.stamp = ros::Time::now();
                // global_node.Publisher<ly_camera_image>().publish(image_msg);
                global_node.Publisher<ly_backcamera_image>().publish(msg);
                rate.sleep();
        }   	
    }};

    std::jthread imagepub_thread{[&] {
        ros::Rate rate(80);
        while (ros::ok()) {
            if (!image_queue.empty()) {
                cv::Mat publish_image = image_queue.wait_and_pop();
                // sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", publish_image).toImageMsg();
                sensor_msgs::CompressedImagePtr compressed_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", publish_image)
                    .toCompressedImageMsg(cv_bridge::JPG);
                compressed_msg->header.stamp = ros::Time::now();
                // global_node.Publisher<ly_camera_image>().publish(image_msg);
                global_node.Publisher<ly_compressed_image>().publish(compressed_msg);
                rate.sleep();
            }
            // rate.sleep();
        }
    }};

    std::jthread ra_imagepub_thread{[&] {
        ros::Rate rate(100); // 100Hz频率
        while (ros::ok()) {
            if (!angle_image_stack.empty()) {
                AngleFrame angle_frame;
                if (!angle_image_stack.pop(angle_frame)) continue;
                sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", angle_frame.image).toImageMsg();
                image_msg->header.stamp = ros::Time::now();
                auto_aim_common::AngleImage angle_image_msg;
                angle_image_msg.image = *image_msg;
                angle_image_msg.Yaw = angle_frame.angles.yaw;
                angle_image_msg.Pitch = angle_frame.angles.pitch;
                // 发布到ly_ra_image话题
                global_node.Publisher<ly_ra_angle_image>().publish(angle_image_msg);
                // global_node.Publisher<ly_ra_image>().publish(image_msg);
                ros::spinOnce(); // force processing of callbacks
            }
            rate.sleep();
        }
    }};

    std::jthread detect_thread {
        [&] {
            ros::Rate rate(78);
            while (ros::ok()) {

                if(web_show && !concat_image.empty()){
                    VideoStreamer::setFrame(concat_image);
                } else if(web_show && !image.empty()) {
                    VideoStreamer::setFrame(image);
                }
                rate.sleep();

                TimedArmors armors;
                std::vector<CarDetection> cars;
                auto_aim_common::Armors armor_list_msg;
                std::vector<ArmorObject> filtered_armors{};
                ArmorObject target_armor;

                if(!use_ros_bag){
                    if (!Cam.GetImage(image)) continue; // match the gimbal gimbal driver
                }
                // if (callbackQueue.empty()) roslog::warn("11111");
                ros::spinOnce();
                if(use_ros_bag){
                    image = callbackQueue.wait_and_pop();
                }
                if (image.empty()) continue;

                ///  发布用于打符的图像
                if(ra_enable && !image.empty()){
                    GimbalAnglesType temp_angles{
                        gimbal_angles_yaw.load(),
                        gimbal_angles_pitch.load()
                    };
                    angle_image_stack.push({image.clone(), temp_angles});
                }

                /// DEBUG
                // gimbal_angles_yaw = -279.4f;
                // gimbal_angles_pitch = -5.4f;
                armors.TimeAngles.yaw = gimbal_angles_yaw;
                armors.TimeAngles.pitch = gimbal_angles_pitch;

                armors.TimeStamp = ros::Time::now();

                auto &detected_armors = armors.Armors;
                detected_armors.clear();
                // roslog::info("armor_detector_node> image size: ready to detect");
                if (!carAndArmorDetector.Detect(image, detected_armors, cars)) continue;

                armor_list_msg.armors.clear();
                armor_list_msg.cars.clear();
                armor_list_msg.header.frame_id = "camera";
                armor_list_msg.header.stamp = armors.TimeStamp;
                armor_list_msg.Yaw = armors.TimeAngles.yaw;
                armor_list_msg.Pitch = armors.TimeAngles.pitch;
                armor_list_msg.is_available_armor_for_predictor = false;

                // TODO here target is unused
                filtered_armors.clear();
                filter.is_team_red = myTeamRed;

                if (debug_mode && debug_team_blue) {
                    filter.is_team_red = false;
                } else if( debug_mode && !debug_team_blue){
                    filter.is_team_red = true;
                }

                roslog::warn("1111");
                ArmorType target = atomic_target;
                if (!filter.Filter(detected_armors, target, filtered_armors)) { continue; }

                if (!finder.ReFindAndSolveAll(solver, filtered_armors, target, target_armor, armor_list_msg)) {
                    /// 特立独行，这个不需要continue
                    // ROS_WARN("armor_refinder_and_solve_all> failed to find armor");
                }
                if(!carFinder.FindCar(cars, armor_list_msg.cars)){
                    roslog::warn("car_finder> failed to find car");
                }

                // if (draw_image) DrawArmor(image, target_armor);
                if(draw_image) DrawAllArmor(image, filtered_armors);
                if(draw_image) DrawAllCar(image, cars);
                roslog::warn("2222");   

                if(aa_enable){
                    global_node.Publisher<ly_detector_armors>().publish(armor_list_msg);
                }else if(outpost_enable){
                    global_node.Publisher<ly_outpost_armors>().publish(armor_list_msg);
                }
            }
        }
    };
}

//// bug 的原因在于订阅和接收用了同一个话题
#pragma region using image in rosbag
void image_callback(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    roslog::warn("image_callback> image size: {}", msg->data.size());
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &e) {
        roslog::error("cv_bridge exception: %s", e.what());
        return;
    }
    // Cam.SetImage(cv_ptr->image);
    callbackQueue.push(cv_ptr->image);
}
#pragma endregion using image in rosbag

void camera_benchmark(std::size_t step = 1000) {
    cv::Mat image;
    using clock_t = std::chrono::high_resolution_clock;
    auto begin = clock_t::now();
    for (std::size_t i = 0;i < step; i++){
        Cam.GetImage(image);
    }
    auto end = clock_t::now();
    auto cost = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin);
    auto avg = cost * 1.0f / step;
    roslog::info("benchmark: step({}) cost({}) avg({})", step, cost, avg);
}

} // namespace

int main(int argc, char **argv) try {
    std::string classifier_path;
    std::string detector_path;
    std::string car_model_path;

    /// 初始化ros
    global_node.Initialize(argc, argv);


    InitialParam();
    /// 初始化节点和模块

    if(!use_ros_bag){
        ConfigureCamera();
    	ConfigureSubCamera();
    }

    if (use_ros_bag){
        global_node.GenSubscriber<ly_compressed_image>(image_callback);
    } else if(use_video) {
        Cam.Initialize(video_path);
	//SubCam.Initialize();
    } else{
        Cam.Initialize("", "KE0200060396");
	SubCam.Initialize("", "FDN23020229");
    }
    // camera_benchmark();  // 基准测试
    // return 0;
    if(web_show){
        VideoStreamer::init();
    }
    
    global_node.GetParam<std::string>("detector_config/classifier_path", classifier_path, "");
    global_node.GetParam<std::string>("detector_config/detector_path", detector_path, "");    
    global_node.GetParam<std::string>("detector_config/car_model_path", car_model_path, "");
    if (!carAndArmorDetector.armorDetector.Corrector.Classifier.LoadModel(classifier_path)) return -1;
    if (!carAndArmorDetector.armorDetector.Detector.LoadModel(detector_path)) return -1;
    if (!carAndArmorDetector.carDetector.LoadModel(car_model_path)) return -1;

    global_node.GenSubscriber<ly_me_is_team_red>(my_team_callback);
    global_node.GenSubscriber<ly_bt_target>(get_target_callback);
    global_node.GenSubscriber<ly_gimbal_angles>(gimbal_callback);
    global_node.GenSubscriber<ly_aa_enable>(aa_enable_callback);

    ImageLoop();

    if (web_show) {
        VideoStreamer::cleanup();
    }
    return 0;
}
catch (const std::exception &e) {
    // TODO handle exception
    std::cerr << "Error in main: " << e.what() << std::endl;
    return 1;
}
