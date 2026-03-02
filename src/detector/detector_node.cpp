#include <auto_aim_common/msg/armor.hpp>
#include <auto_aim_common/msg/armors.hpp>
#include <gimbal_driver/msg/gimbal_angles.hpp>
#include <auto_aim_common/msg/angle_image.hpp>
#include <Logger/Logger.hpp>
#include <RosTools/RosTools.hpp>

#include <thread>
#include <iostream> 

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>

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

namespace {

LY_DEF_ROS_TOPIC(ly_aa_enable, "/ly/aa/enable", std_msgs::msg::Bool);
LY_DEF_ROS_TOPIC(ly_ra_enable, "/ly/ra/enable", std_msgs::msg::Bool);
LY_DEF_ROS_TOPIC(ly_outpost_enable, "/ly/outpost/enable", std_msgs::msg::Bool);
LY_DEF_ROS_TOPIC(ly_camera_image, "/ly/camera/image", sensor_msgs::msg::Image);
LY_DEF_ROS_TOPIC(ly_backcamera_image, "/ly/backcamera/image", sensor_msgs::msg::Image);
LY_DEF_ROS_TOPIC(ly_me_is_team_red, "/ly/me/is_team_red", std_msgs::msg::Bool);
LY_DEF_ROS_TOPIC(ly_bt_target, "/ly/bt/target", std_msgs::msg::UInt8);
LY_DEF_ROS_TOPIC(ly_detector_armors, "/ly/detector/armors", auto_aim_common::msg::Armors);
LY_DEF_ROS_TOPIC(ly_gimbal_angles, "/ly/gimbal/angles", gimbal_driver::msg::GimbalAngles);
LY_DEF_ROS_TOPIC(ly_compressed_image, "/ly/compressed/image", sensor_msgs::msg::CompressedImage);
LY_DEF_ROS_TOPIC(ly_ra_angle_image, "/ly/ra/angle_image", auto_aim_common::msg::AngleImage);
LY_DEF_ROS_TOPIC(ly_outpost_armors, "/ly/outpost/armors", auto_aim_common::msg::Armors);

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
std::shared_ptr<ROSNode<AppName>> global_node = nullptr;

CarAndArmorDetector carAndArmorDetector{};
ArmorFilter filter{};
ArmorRefinder finder{};
CarFinder carFinder{};
CameraIntrinsicsParameterPack cameraIntrinsics{};
PoseSolver solver{cameraIntrinsics};
Camera Cam;

using AngleType = float;
using Angle100Type = std::int16_t;
struct GimbalAnglesType {
    AngleType yaw{0.0f};
    AngleType pitch{0.0f};
}; 

#pragma region image_queue
class ImageQueue {
private:
    std::queue<cv::Mat> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cond_;
    const size_t max_size_ = 10; 

public:
    void push(cv::Mat image) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() >= max_size_) {
            queue_.pop(); 
        }
        queue_.push(image.clone()); 
        cond_.notify_all(); 
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
constexpr size_t MAX_STACK_SIZE = 30; 
#pragma endregion image_stack

std::atomic<AngleType> gimbal_angles_yaw;
std::atomic<AngleType> gimbal_angles_pitch;

std::atomic<ArmorType> atomic_target{};
PNPAimResult aim_result{};

// =========================================================
// 【核心修復】 恢復斜線 "/" 以匹配你的 YAML 格式
// =========================================================
void InitialParam(){
    // YAML 裡寫的是 "detector_config/show"，所以這裡必須用 "/"
    global_node->GetParam<bool>("detector_config/show", pub_image, true);
    global_node->GetParam<bool>("detector_config/draw", draw_image, true);
    global_node->GetParam<bool>("detector_config/debug_mode", debug_mode, false);
    global_node->GetParam<bool>("detector_config/debug_team_blue", debug_team_blue, false);
    global_node->GetParam<bool>("detector_config/save_video", save_video, false);
    global_node->GetParam<bool>("detector_config/web_show", web_show, true);
    global_node->GetParam<bool>("detector_config/use_video", use_video, false);
    global_node->GetParam<bool>("detector_config/use_ros_bag", use_ros_bag, false);
    global_node->GetParam<std::string>("detector_config/video_path", video_path, "");
}

void ConfigureCamera() {
    auto &config = Cam.Configure();
    config.AutoExposure.Value = GX_EXPOSURE_AUTO_OFF;
    // YAML 裡寫的是 "camera_param/ExposureTime"，所以用 "/"
    global_node->GetParam<double>("camera_param/ExposureTime", config.ExposureTime.Value, 4000);
    config.AutoGain.Value = GX_GAIN_AUTO_OFF;
    global_node->GetParam<double>("camera_param/Gain", config.Gain.Value, 12);
    global_node->GetParam<double>("camera_param/RedBalanceRatio", config.RedBalanceRatio.Value, 1.2266f);
    global_node->GetParam<double>("camera_param/GreenBalanceRatio", config.GreenBalanceRatio.Value, 1.0f);
    global_node->GetParam<double>("camera_param/BlueBalanceRatio", config.BlueBalanceRatio.Value, 1.3711f);
}


void my_team_callback(const std_msgs::msg::Bool::ConstSharedPtr msg) {
    myTeamRed = msg->data;
}

void gimbal_callback(const gimbal_driver::msg::GimbalAngles::ConstSharedPtr msg) {
    gimbal_angles_yaw = msg->yaw;
    gimbal_angles_pitch = msg->pitch;
}

void get_target_callback(const std_msgs::msg::UInt8::ConstSharedPtr msg) {
    atomic_target = static_cast<ArmorType>(msg->data);
}

void aa_enable_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg) {
    aa_enable = msg->data;
}

void ra_enable_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg) {
    ra_enable = msg->data;
}

void outpost_enable_callback(const std_msgs::msg::Bool::ConstSharedPtr &msg) {
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

void DrawAllCar(cv::Mat &image, const std::vector<CarDetection> &cars) {
    for (const auto &car : cars) {
        DrawCarBBox(image, car);
    }
}

struct TimedArmors {
    rclcpp::Time TimeStamp;
    std::vector<ArmorObject> Armors;
    GimbalAnglesType TimeAngles{};
};

struct AngleFrame{
    cv::Mat image;
    GimbalAnglesType angles;
};

ImageQueue callbackQueue;
void ImageLoop() {
    cv::Mat image;
    ImageQueue image_queue;
    boost::lockfree::stack<AngleFrame> angle_image_stack(MAX_STACK_SIZE);

    std::jthread imagepub_thread{[&] {
        rclcpp::Rate rate(80);
        while (rclcpp::ok()) {
            if (!image_queue.empty()) {
                cv::Mat publish_image = image_queue.wait_and_pop();
                sensor_msgs::msg::CompressedImage::SharedPtr compressed_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", publish_image)
                    .toCompressedImageMsg(cv_bridge::JPG);
                compressed_msg->header.stamp = global_node->now(); 
                global_node->Publisher<ly_compressed_image>()->publish(*compressed_msg);
                rate.sleep();
            }
        }
    }};

    std::jthread ra_imagepub_thread{[&] {
        rclcpp::Rate rate(100); 
        while (rclcpp::ok()) {
            if (!angle_image_stack.empty()) {
                AngleFrame angle_frame;
                if (!angle_image_stack.pop(angle_frame)) continue;
                sensor_msgs::msg::Image::SharedPtr image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", angle_frame.image).toImageMsg();
                image_msg->header.stamp = global_node->now(); 
                auto_aim_common::msg::AngleImage angle_image_msg;
                angle_image_msg.image = *image_msg;
                angle_image_msg.yaw = angle_frame.angles.yaw;
                angle_image_msg.pitch = angle_frame.angles.pitch;
                global_node->Publisher<ly_ra_angle_image>()->publish(angle_image_msg);
                rclcpp::spin_some(global_node->get_node_base_interface());; 
            }
            rate.sleep();
        }
    }};

    std::jthread detect_thread {
        [&] {
            rclcpp::Rate rate(78);
            while (rclcpp::ok()) {

                if(web_show && !image.empty()) {
                    VideoStreamer::setFrame(image);
                }
                rate.sleep();

                TimedArmors armors;
                std::vector<CarDetection> cars;
                auto_aim_common::msg::Armors armor_list_msg;
                std::vector<ArmorObject> filtered_armors{};
                ArmorObject target_armor;
                if(!use_ros_bag){
                    if (!Cam.GetImage(image)) {
                        // 如果讀不到 (影片播完了)，嘗試重置
                        if(use_video && !video_path.empty()){
                             std::cout << "[DEBUG] Video ended. Replaying..." << std::endl;
                             Cam.Initialize(video_path);
                        }
                        continue; 
                    }
                    
                    static int frame_cnt = 0;
                    if (++frame_cnt % 30 == 0) {
                        std::cout << "[DEBUG] Processing Frame: " << frame_cnt << std::endl;
                    }
                }
                rclcpp::spin_some(global_node->get_node_base_interface());;
                if(use_ros_bag){
                    image = callbackQueue.wait_and_pop();
                }
                if (image.empty()) continue;
                if(pub_image) image_queue.push(image);

                if(ra_enable && !image.empty()){
                    GimbalAnglesType temp_angles{
                        gimbal_angles_yaw.load(),
                        gimbal_angles_pitch.load()
                    };
                    angle_image_stack.push({image.clone(), temp_angles});
                }

                armors.TimeAngles.yaw = gimbal_angles_yaw;
                armors.TimeAngles.pitch = gimbal_angles_pitch;

                armors.TimeStamp = global_node->now(); 

                auto &detected_armors = armors.Armors;
                detected_armors.clear();
                if (!carAndArmorDetector.Detect(image, detected_armors, cars)) continue;

                armor_list_msg.armors.clear();
                armor_list_msg.cars.clear();
                armor_list_msg.header.frame_id = "camera";
                armor_list_msg.header.stamp = armors.TimeStamp;
                armor_list_msg.yaw = armors.TimeAngles.yaw;
                armor_list_msg.pitch = armors.TimeAngles.pitch;
                armor_list_msg.is_available_armor_for_predictor = false;

                filtered_armors.clear();
                filter.is_team_red = myTeamRed;

                if (debug_mode && debug_team_blue) {
                    filter.is_team_red = false;
                } else if( debug_mode && !debug_team_blue){
                    filter.is_team_red = true;
                }

                ArmorType target = atomic_target;
                if (!filter.Filter(detected_armors, target, filtered_armors)) { continue; }

                if (!finder.ReFindAndSolveAll(solver, filtered_armors, target, target_armor, armor_list_msg)) {
                }
                if(!carFinder.FindCar(cars, armor_list_msg.cars)){
                }

                if(draw_image) DrawAllArmor(image, filtered_armors);
                if(draw_image) DrawAllCar(image, cars);

                if(aa_enable){
                    global_node->Publisher<ly_detector_armors>()->publish(armor_list_msg);
                }else if(outpost_enable){
                    global_node->Publisher<ly_outpost_armors>()->publish(armor_list_msg);
                }
            }
        }
    };
}

#pragma region using image in rosbag
void image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr msg) {
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
    rclcpp::init(argc, argv);

    global_node = std::make_shared<ROSNode<AppName>>();

    std::string classifier_path;
    std::string detector_path;
    std::string car_model_path;

    InitialParam();

    /// 初始化节点和模块

    if(!use_ros_bag){
        if (use_video) {
            std::cout << "[DEBUG] Opening Video: " << video_path << std::endl;
            if(video_path.empty()){
                std::cerr << "[ERROR] Video path is empty! Check your YAML file." << std::endl;
                // return -1; // 這裡可以選擇報錯退出，或者讓 Cam.Initialize 處理
            }
            Cam.Initialize(video_path);
        } else {
            ConfigureCamera();
            Cam.Initialize("", "KE0200060396");
        }
    }

    if (use_ros_bag){
        global_node->GenSubscriber<ly_compressed_image>(image_callback);
    } 

    if(web_show){
        VideoStreamer::init();
    }
    
    // 【核心修復】 恢復斜線 "/" 以匹配你的 YAML 格式
    global_node->GetParam<std::string>("detector_config/classifier_path", classifier_path, "");
    global_node->GetParam<std::string>("detector_config/detector_path", detector_path, "");    
    global_node->GetParam<std::string>("detector_config/car_model_path", car_model_path, "");
    
    std::cout << "[DEBUG] Loading Classifier: " << classifier_path << std::endl;
    if (!carAndArmorDetector.armorDetector.Corrector.Classifier.LoadModel(classifier_path)) return -1;
    
    std::cout << "[DEBUG] Loading Detector: " << detector_path << std::endl;
    if (!carAndArmorDetector.armorDetector.Detector.LoadModel(detector_path)) return -1;

    std::cout << "[DEBUG] Loading CarModel: " << car_model_path << std::endl;
    // 這裡加上 try-catch 因為 yolo_detector 內部沒有保護
    try {
        if (!carAndArmorDetector.carDetector.LoadModel(car_model_path)) return -1;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] CarModel Crash: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "[ERROR] CarModel Unknown Crash" << std::endl;
        return -1;
    }

    std::cout << "[DEBUG] All Models Loaded Successfully!" << std::endl;

    global_node->GenSubscriber<ly_me_is_team_red>(my_team_callback);
    global_node->GenSubscriber<ly_bt_target>(get_target_callback);
    global_node->GenSubscriber<ly_gimbal_angles>(gimbal_callback);
    global_node->GenSubscriber<ly_aa_enable>(aa_enable_callback);
    // [修復] 補上缺失的訂閱者，否則 ra_enable/outpost_enable 永遠是 false
    global_node->GenSubscriber<ly_ra_enable>(ra_enable_callback);
    global_node->GenSubscriber<ly_outpost_enable>(outpost_enable_callback);

    // 進入圖像循環
    ImageLoop();

    if (web_show) {
        VideoStreamer::cleanup();
    }

    rclcpp::shutdown();
    return 0;
}
catch (const std::exception &e) {
    std::cerr << "Error in main: " << e.what() << std::endl;
    return 1;
}