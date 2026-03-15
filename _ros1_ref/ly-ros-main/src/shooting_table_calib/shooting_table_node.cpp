#include <car_tracker/tracker.hpp>
#include <car_tracker/tracker_matcher.hpp>
#include <solver/solver.hpp>

#include <predictor/predictor.hpp>
#include <controller/controller.hpp>

#include <armor_detector/armor_detector.hpp>
#include <armor_detector/armor_filter.hpp>
#include <armor_detector/armor_refinder.hpp>
#include <armor_detector/pnp_solver.hpp>
#include <car_detector/car_finder.hpp>
#include <car_and_armor_detector.hpp>
#include <camera/Camera.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>

#include <RosTools/RosTools.hpp>
#include <Logger/Logger.hpp>
#include <TimeStamp/TimeStamp.hpp>
#include <auto_aim_common/Armors.h>
#include <auto_aim_common/Target.h>
#include <auto_aim_common/DetectionType.hpp>
#include <auto_aim_common/SolverType.hpp>
#include <auto_aim_common/TrackerType.hpp>
#include <gimbal_driver/GimbalAngles.h>
#include <VideoStreamer/VideoStreamer.hpp>
#include <auto_aim_common/Location.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <atomic>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>

using namespace ly_auto_aim;
using namespace LangYa;

namespace {
    const char AppName[] = "shooting_table_calib_node";
    
    // 定义ROS话题
    LY_DEF_ROS_TOPIC(ly_gimbal_angles, "/ly/gimbal/angles", gimbal_driver::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_control_angles, "/ly/control/angles", gimbal_driver::GimbalAngles);
    LY_DEF_ROS_TOPIC(ly_predictor_target, "/ly/predictor/target", auto_aim_common::Target);
    LY_DEF_ROS_TOPIC(ly_control_firecode, "/ly/control/firecode", std_msgs::UInt8);

    // 射表记录结构
    struct ShootingRecord {
        double z_height;
        double horizontal_distance;
        double relative_yaw;
        double relative_pitch;
        cv::Point3d target_world_coord;
        double absolute_yaw;
        double absolute_pitch;
        ros::Time timestamp;
        double target_yaw;
        double fitted_pitch;
        double fitted_yaw;
    };

    // 键盘输入处理类
    class KeyboardInput {
    private:
        struct termios original_termios;
        int original_flags;
        bool initialized = false;
        
    public:
        KeyboardInput() {
            try {
                tcgetattr(STDIN_FILENO, &original_termios);
                original_flags = fcntl(STDIN_FILENO, F_GETFL, 0);
                
                struct termios new_termios = original_termios;
                new_termios.c_lflag &= ~(ICANON | ECHO);
                tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
                fcntl(STDIN_FILENO, F_SETFL, original_flags | O_NONBLOCK);
                initialized = true;
                std::cout << "✓ Terminal input mode initialized\n";
            } catch (...) {
                std::cerr << "✗ Failed to initialize terminal input mode\n";
            }
        }
        
        ~KeyboardInput() {
            restore();
        }
        
        void restore() {
            if (initialized) {
                try {
                    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
                    fcntl(STDIN_FILENO, F_SETFL, original_flags);
                    std::cout << "\n✓ Terminal input mode restored\n";
                    initialized = false;
                } catch (...) {
                    std::cerr << "✗ Failed to restore terminal input mode\n";
                }
            }
        }
        
        char getKey() {
            if (!initialized) return 0;
            char key;
            if (read(STDIN_FILENO, &key, 1) == 1) {
                return key;
            }
            return 0;
        }
        
        std::string getLine() {
            if (!initialized) return "";
            
            // 临时恢复正常输入模式
            tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
            fcntl(STDIN_FILENO, F_SETFL, original_flags);
            
            std::string line;
            std::getline(std::cin, line);
            
            // 重新设置非阻塞模式
            struct termios new_termios = original_termios;
            new_termios.c_lflag &= ~(ICANON | ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
            fcntl(STDIN_FILENO, F_SETFL, original_flags | O_NONBLOCK);
            
            return line;
        }
    };

    class ShootingTableCalibNode
    {
    private:
        ROSNode<AppName> node;
        KeyboardInput keyboard;
        
        // 算法模块 - 使用正确的流程
        std::unique_ptr<tracker::Tracker> tracker;
        std::shared_ptr<solver::Solver> solver;
        std::unique_ptr<predictor::Predictor> predictor;
        std::unique_ptr<controller::Controller> controller;
        
        // detector模块
        CarAndArmorDetector carAndArmorDetector;
        ArmorFilter filter;
        ArmorRefinder finder;
        CarFinder carFinder;
        pnp_solver::CameraIntrinsicsParameterPack cameraIntrinsics;
        PoseSolver pnpSolver;
        Camera camera;
        
        // Video功能支持
        bool use_video = false;
        bool use_ros_bag = false;
        std::string video_path;
        cv::VideoCapture video_cap;
        
        // 状态变量
        std::vector<ShootingRecord> records;
        std::string csv_filename;
        
        // 控制参数
        double pitch_adjustment = 0.0;
        double yaw_adjustment = 0.0;
        const double adjustment_step = 0.1;
        std::atomic<bool> is_shooting{false};
        ros::Time shoot_start_time;
        const double shoot_duration = 0.3;
        
        // 瞄准状态
        std::atomic<bool> is_aiming{false};
        std::atomic<bool> should_aim_once{false};
        cv::Point3d current_target_world;
        gimbal_driver::GimbalAngles current_gimbal_angles;
        double current_target_yaw = 0.0;
        
        // 控制命令状态
        std::atomic<bool> control_valid{false};
        std::atomic<bool> aim_only_mode{false};  // 新增：只瞄准模式
        double target_yaw = 0.0;
        double target_pitch = 0.0;
        
        // 射击表系数 - 支持动态修改
        struct ShootTableParams {
            bool enable;
            struct {
                double intercept, coef_z, coef_d, coef_z2, coef_zd, coef_d2;
            } pitch, yaw;
        } shoot_table_params;

        // 火控系统状态
        struct FireControlData {
            uint8_t fire_status = 0;  // 只使用最低2位：00 或 11
            bool last_fire_command = false;  // 记录上次的开火命令状态
        } fire_control;
        
        // 火控发布话题

        
        // 显示相关
        bool web_show = true;
        bool draw_image = true;
        bool myTeamBlue{false};

        // Remove the param_nh and last_param_check as they're no longer needed
        ros::Time last_param_check;
        const double param_check_interval = 3.0;

    public:
        ShootingTableCalibNode(int argc, char** argv) : pnpSolver(cameraIntrinsics)
        {
            if (!node.Initialize(argc, argv)) {
                roslog::error("Failed to initialize ROS node");
                return;
            }
            
            loadShootTableParams();
            initializeVideo();
            initializeCamera();
            initializeDetector();
            initializeAlgorithms();
            setupRosTopics();
            createCSVFile();
            printInstructions();
            
            if (web_show) {
                VideoStreamer::init();
            }
            
            last_param_check = ros::Time::now();
        }
        
        ~ShootingTableCalibNode()
        {
            if (web_show) {
                VideoStreamer::cleanup();
            }
            if (video_cap.isOpened()) {
                video_cap.release();
            }
        }

    private:
        void loadShootTableParams()
        {
            // 从config文件加载射击表参数
            node.GetParam<bool>("shoot_table_adjust/enable", shoot_table_params.enable, true);
            
            // pitch参数
            node.GetParam<double>("shoot_table_adjust/pitch/intercept", shoot_table_params.pitch.intercept, 0.0);
            node.GetParam<double>("shoot_table_adjust/pitch/coef_z", shoot_table_params.pitch.coef_z, 0.0);
            node.GetParam<double>("shoot_table_adjust/pitch/coef_d", shoot_table_params.pitch.coef_d, 0.0);
            node.GetParam<double>("shoot_table_adjust/pitch/coef_z2", shoot_table_params.pitch.coef_z2, 0.0);
            node.GetParam<double>("shoot_table_adjust/pitch/coef_zd", shoot_table_params.pitch.coef_zd, 0.0);
            node.GetParam<double>("shoot_table_adjust/pitch/coef_d2", shoot_table_params.pitch.coef_d2, 0.0);
            
            // yaw参数
            node.GetParam<double>("shoot_table_adjust/yaw/intercept", shoot_table_params.yaw.intercept, 0.0);
            node.GetParam<double>("shoot_table_adjust/yaw/coef_z", shoot_table_params.yaw.coef_z, 0.0);
            node.GetParam<double>("shoot_table_adjust/yaw/coef_d", shoot_table_params.yaw.coef_d, 0.0);
            node.GetParam<double>("shoot_table_adjust/yaw/coef_z2", shoot_table_params.yaw.coef_z2, 0.0);
            node.GetParam<double>("shoot_table_adjust/yaw/coef_zd", shoot_table_params.yaw.coef_zd, 0.0);
            node.GetParam<double>("shoot_table_adjust/yaw/coef_d2", shoot_table_params.yaw.coef_d2, 0.0);

            roslog::info("Loaded shoot table params - yaw coef_d2: {}", shoot_table_params.yaw.coef_d2);
            
            roslog::info("Loaded shoot table params - Enable: {}", shoot_table_params.enable);
            
            roslog::info("Loaded shoot table params - Enable: {}", shoot_table_params.enable);
        }
        
        void initializeVideo()
        {
            node.GetParam<bool>("detector_config/use_video", use_video, false);
            roslog::warn("use_video: {}", use_video);
            node.GetParam<bool>("detector_config/use_ros_bag", use_ros_bag, false);
            node.GetParam<std::string>("detector_config/video_path", video_path, "");
            
            if (use_video && !video_path.empty()) {
                video_cap.open(video_path);
                if (!video_cap.isOpened()) {
                    roslog::error("Failed to open video file: {}", video_path);
                    use_video = false;
                } else {
                    roslog::info("Video file loaded: {}", video_path);
                }
            }
        }
        
        void initializeCamera()
        {
            if (use_video || use_ros_bag) {
                roslog::info("Skipping camera initialization - using video/rosbag");
                return;
            }
            
            std::string camera_sn;
            node.GetParam<std::string>("camera_param/camera_sn", camera_sn, "KE0200060396");
            
            auto &config = camera.Configure();
            config.AutoExposure.Value = GX_EXPOSURE_AUTO_OFF;
            node.GetParam<double>("camera_param/ExposureTime", config.ExposureTime.Value, 4000.0);
            config.AutoGain.Value = GX_GAIN_AUTO_OFF;
            node.GetParam<double>("camera_param/Gain", config.Gain.Value, 12.0);
            node.GetParam<double>("camera_param/RedBalanceRatio", config.RedBalanceRatio.Value, 1.2266);
            node.GetParam<double>("camera_param/GreenBalanceRatio", config.GreenBalanceRatio.Value, 1.0);
            node.GetParam<double>("camera_param/BlueBalanceRatio", config.BlueBalanceRatio.Value, 1.3711);
            
            if (!camera.Initialize("", camera_sn)) {
                roslog::error("Failed to initialize camera");
                throw std::runtime_error("Camera initialization failed");
            }
            
            roslog::info("Camera initialized successfully");
        }
        
        void initializeDetector()
        {
            std::string classifier_path;
            std::string detector_path;
            std::string car_model_path;
            
            node.GetParam<std::string>("detector_config/classifier_path", classifier_path, "");
            node.GetParam<std::string>("detector_config/detector_path", detector_path, "");
            node.GetParam<std::string>("detector_config/car_model_path", car_model_path, "");
            
            if (!carAndArmorDetector.armorDetector.Corrector.Classifier.LoadModel(classifier_path)) {
                roslog::error("Failed to load classifier model: {}", classifier_path);
                throw std::runtime_error("Classifier model loading failed");
            }
            
            if (!carAndArmorDetector.armorDetector.Detector.LoadModel(detector_path)) {
                roslog::error("Failed to load detector model: {}", detector_path);
                throw std::runtime_error("Detector model loading failed");
            }
            
            if (!carAndArmorDetector.carDetector.LoadModel(car_model_path)) {
                roslog::error("Failed to load car model: {}", car_model_path);
                throw std::runtime_error("Car model loading failed");
            }
            
            // 设置队伍颜色
            node.GetParam<bool>("detector_config/debug_team_blue", myTeamBlue, true);
            filter.is_team_red = !myTeamBlue;

            roslog::info("Detector modules initialized successfully");
        }

        void initializeAlgorithms()
        {
            try {
                tracker = std::make_unique<tracker::Tracker>();
                solver = std::make_shared<solver::Solver>();
                predictor = std::make_unique<predictor::Predictor>();
                controller = std::make_unique<controller::Controller>();
                
                // 注册solver到location模块
                location::Location::registerSolver(solver);
                
                roslog::info("Algorithm modules initialized successfully");
            } catch (const std::exception& e) {
                roslog::error("Failed to initialize algorithms: {}", e.what());
                throw;
            }
        }
        
        void setupRosTopics()
        {
            // 订阅话题
            node.GenSubscriber<ly_gimbal_angles>([this](auto msg) {
                gimbalAngleCallback(msg);
            });
        }

        void createCSVFile()
        {
            // 创建record目录（如果不存在）
            std::string record_dir = "/home/hustlyrm/workspace/record";
            // system(("mkdir -p " + record_dir).c_str());
            
            csv_filename = record_dir + "/shooting_table_" + 
                          std::to_string(std::time(nullptr)) + ".csv";
            
            std::ofstream file(csv_filename);
            if (file.is_open()) {
                file << "timestamp,z_height,horizontal_distance,relative_yaw,relative_pitch,"
                     << "target_x,target_y,target_z,absolute_yaw,absolute_pitch,target_yaw,"
                     << "fitted_pitch,fitted_yaw\n";
                file.close();
                roslog::info("Created CSV file: {}", csv_filename);
            }
        }

        void sendFireControlCommand()
        {
            // 构造火控数据包 - 只使用最低2位
            std_msgs::UInt8 fire_msg;
            fire_msg.data = fire_control.fire_status;  // 直接发送，其他位都是0
            
            node.Publisher<ly_control_firecode>().publish(fire_msg);
            
            roslog::info("Fire control sent - FireCode: 0b{:08b} ({})", 
                        fire_msg.data, fire_msg.data);
        }

        

        void flipFireStatus()
        {
            // 翻转开火状态：00 <-> 11 (参考behavior_tree的FlipFireStatus逻辑)
            fire_control.fire_status = (fire_control.fire_status == 0) ? 0b11 : 0b00;
            roslog::info("Fire status flipped to: 0b{:08b}", fire_control.fire_status);
        }

        void updateFireControl(bool should_fire)
        {
            // 检查是否需要翻转开火状态
            if (should_fire != fire_control.last_fire_command) {
                if (should_fire) {
                    // 开始射击 - 翻转状态
                    flipFireStatus();
                    std::cout << "🔥 Fire control activated - Status flipped!\n";
                } else {
                    // 停止射击 - 保持当前状态不变（让云台停止响应）
                    std::cout << "🛑 Fire control deactivated\n";
                }
                fire_control.last_fire_command = should_fire;
                sendFireControlCommand();
            }
        }

        void sendAimOnlyCommand()
        {
            // 只瞄准，不射击 - 发送到control/angles
            gimbal_driver::GimbalAngles control_msg;
            
            // 设置目标角度（包含调整值）
            control_msg.Yaw = target_yaw + yaw_adjustment;
            control_msg.Pitch = target_pitch + pitch_adjustment;
            
            // 发布云台控制命令
            node.Publisher<ly_control_angles>().publish(control_msg);
            
            roslog::info("Aim command sent - Yaw: {:.2f}°, Pitch: {:.2f}°", 
                        control_msg.Yaw, control_msg.Pitch);
            std::cout << "🎯 Aim only - Yaw: " << std::fixed << std::setprecision(2) 
                     << control_msg.Yaw << "°, Pitch: " << control_msg.Pitch << "°\n";
        }

        void sendControlCommand()
        {
            // 瞄准并射击 - 发送到predictor/target
            auto_aim_common::Target target_msg;
            target_msg.header.stamp = ros::Time::now();
            target_msg.header.frame_id = "camera";
            
            // 设置目标角度（包含调整值）
            target_msg.yaw = target_yaw + yaw_adjustment;
            target_msg.pitch = target_pitch + pitch_adjustment;
            target_msg.status = true;  // 启用跟踪状态，开始射击
            target_msg.buff_follow = false;  // 不是打符模式
            
            // 发布预测器目标命令
            node.Publisher<ly_predictor_target>().publish(target_msg);
            
            roslog::info("Target command sent - Yaw: {:.2f}°, Pitch: {:.2f}°", 
                        target_msg.yaw, target_msg.pitch);
            std::cout << "🔥 Target published - Yaw: " << std::fixed << std::setprecision(2) 
                     << target_msg.yaw << "°, Pitch: " << target_msg.pitch << "°\n";
        }

        void sendStopCommand()
        {
            // 发送停止射击命令
            auto_aim_common::Target stop_msg;
            stop_msg.header.stamp = ros::Time::now();
            stop_msg.header.frame_id = "camera";
            stop_msg.yaw = current_gimbal_angles.Yaw;  // 保持当前角度
            stop_msg.pitch = current_gimbal_angles.Pitch;
            stop_msg.status = false;  // 禁用跟踪状态，停止开火
            stop_msg.buff_follow = false;
            
            node.Publisher<ly_predictor_target>().publish(stop_msg);
            
            // 停止火控 - 修复这里的错误调用
            updateFireControl(false);  // 原来这里错误地调用了sendStopCommand()
            
            roslog::info("Stop command sent");
            std::cout << "🛑 Stop shooting and fire control stopped\n";
        }

        void gimbalAngleCallback(const gimbal_driver::GimbalAngles::ConstPtr& msg)
        {
            current_gimbal_angles.Yaw = msg->Yaw;
            current_gimbal_angles.Pitch = msg->Pitch;
            
            // // 如果是瞄准模式且有有效控制
            // if (control_valid.load() && aim_only_mode.load()) {
            //     sendAimOnlyCommand();
            // }
            // // 如果是射击模式且正在射击
            // else if (control_valid.load() && is_shooting.load()) {
            //     sendControlCommand();
            // }
        }

        bool getImage(cv::Mat& image)
        {
            if (use_video && video_cap.isOpened()) {
                return video_cap.read(image);
            } else if (!use_video && !use_ros_bag) {
                return camera.GetImage(image);
            }
            return false;
        }

        void convertToDetections(const std::vector<ArmorObject>& armors, std::vector<Detection>& detections)
        {
            detections.clear();
            detections.reserve(armors.size());
            for (const auto& armor : armors) {
                detections.emplace_back(Detection{
                    .tag_id = armor.type,
                    .corners = {
                        {armor.apex[0].x, armor.apex[0].y},
                        {armor.apex[1].x, armor.apex[1].y},
                        {armor.apex[2].x, armor.apex[2].y},
                        {armor.apex[3].x, armor.apex[3].y}
                    }
                });
            }
        }

        void convertToCarDetections(const std::vector<CarDetection>& cars, std::vector<CarDetection>& car_detections)
        {
            car_detections = cars; // 直接复制，因为结构相同
        }

        void processImageDetections()
        {
            cv::Mat image;
            if (!getImage(image)) return;
            
            if (image.empty()) return;

            // 检测装甲板和车辆
            std::vector<ArmorObject> detected_armors;
            std::vector<CarDetection> cars;
            
            if (!carAndArmorDetector.Detect(image, detected_armors, cars)) {
                if (web_show) {
                    VideoStreamer::setFrame(image);
                }
                return;
            }

            // 过滤装甲板
            std::vector<ArmorObject> filtered_armors;
            ArmorType target = ArmorType::Infantry2;
            
            if (!filter.Filter(detected_armors, target, filtered_armors)) {
                if (web_show) {
                    VideoStreamer::setFrame(image);
                }
                return;
            }

            // 转换为tracker所需的格式
            std::vector<Detection> detections;
            std::vector<CarDetection> car_detections;
            convertToDetections(filtered_armors, detections);
            convertToCarDetections(cars, car_detections);

            // 使用tracker和solver处理
            GimbalAngleType gimbal_angle{current_gimbal_angles.Pitch, current_gimbal_angles.Yaw};
            
            tracker->merge(detections);
            tracker->merge(car_detections);
            
            auto track_results = tracker->getTrackResult(ros::Time::now(), gimbal_angle);
            solver->solve_all(track_results, gimbal_angle);

            // 如果需要进行单次瞄准
            if (should_aim_once.load() && !track_results.first.empty()) {
                // 选择第一个目标
                auto& best_track = track_results.first[0];
                
                // 从tracker结果获取世界坐标
                XYZ target_xyz = best_track.location.xyz_imu;
                
                if (calculateBallisticSolution(target_xyz)) {
                    current_target_world = cv::Point3d(target_xyz.x, target_xyz.y, target_xyz.z);
                    is_aiming.store(true);
                    control_valid.store(true);
                    should_aim_once.store(false);
                    
                    // 计算射击表拟合值
                    double distance = std::sqrt(target_xyz.x * target_xyz.x + target_xyz.y * target_xyz.y);
                    double fitted_pitch_val = fitPitch(target_xyz.z, distance);
                    double fitted_yaw_val = fitYaw(target_xyz.z, distance);
                    
                    std::cout << "✓ Target locked! Control enabled\n";
                    std::cout << "  Target distance: " << distance << "m\n";
                    std::cout << "  Target height: " << target_xyz.z << "m\n";
                    std::cout << "  Ballistic yaw: " << target_yaw << "°\n";
                    std::cout << "  Ballistic pitch: " << target_pitch << "°\n";
                    std::cout << "  Fitted yaw: " << fitted_yaw_val << "°\n";
                    std::cout << "  Fitted pitch: " << fitted_pitch_val << "°\n";
                    
                    // if (shoot_table_params.enable) {
                    //     std::cout << "  Using fitted values from shoot table\n";
                    //     target_yaw = fitted_yaw_val;
                    //     target_pitch = fitted_pitch_val;
                    // } else {
                    //     std::cout << "  Using ballistic calculation\n";
                    // }
                } else {
                    std::cout << "✗ Ballistic calculation failed\n";
                    should_aim_once.store(false);
                }
            } else if (should_aim_once.load() && track_results.first.empty()) {
                std::cout << "✗ No target detected. Try again.\n";
                should_aim_once.store(false);
            }

            // 绘制调试信息
            if (draw_image) {
                drawDebugInfo(image, filtered_armors, cars, track_results);
            }

            // 发送到web显示
            if (web_show) {
                VideoStreamer::setFrame(image);
            }
        }

        void drawDebugInfo(cv::Mat& image, const std::vector<ArmorObject>& armors, 
                          const std::vector<CarDetection>& cars, 
                          const std::pair<std::vector<tracker::TrackResult>, std::vector<tracker::CarTrackResult>>& track_results)
        {
            // 绘制十字准心
            int center_x = image.cols / 2;
            int center_y = image.rows / 2;
            cv::line(image, cv::Point(center_x - 20, center_y), 
                     cv::Point(center_x + 20, center_y), cv::Scalar(0, 255, 0), 2);
            cv::line(image, cv::Point(center_x, center_y - 20), 
                     cv::Point(center_x, center_y + 20), cv::Scalar(0, 255, 0), 2);
            
            // 显示调整信息和控制状态
            std::string adj_text = "Adj Y:" + std::to_string(yaw_adjustment) + 
                                  " P:" + std::to_string(pitch_adjustment) +
                                  (control_valid.load() ? " [CTRL ON]" : " [CTRL OFF]");
            cv::putText(image, adj_text, cv::Point(10, 30), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
            
            // 显示状态
            std::string status;
            if (is_shooting.load()) {
                status = "SHOOTING";
            } else if (is_aiming.load()) {
                status = "TARGET LOCKED";
            } else if (should_aim_once.load()) {
                status = "AIMING...";
            } else {
                status = "READY - Press 'a' to aim";
            }
            cv::putText(image, status, cv::Point(10, 60), 
                       cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
            
            // 绘制装甲板
            for (const auto& armor : armors) {
                for (const auto& point : armor.apex) {
                    cv::circle(image, point, 5, cv::Scalar(0, 0, 255), -1);
                }
                
                cv::line(image, armor.apex[0], armor.apex[2], cv::Scalar(255, 0, 0), 2);
                cv::line(image, armor.apex[1], armor.apex[3], cv::Scalar(0, 255, 0), 2);

                const std::string type_text = std::to_string(armor.type);
                cv::Point text_org = armor.apex[0] + cv::Point2f(10, 30); 
                cv::putText(image, type_text, text_org, cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
                
                // 显示距离信息
                if (!armors.empty()) {
                    double distance = std::sqrt(current_target_world.x * current_target_world.x + 
                                              current_target_world.y * current_target_world.y);
                    std::string dist_text = std::to_string((int)distance) + "m";
                    cv::putText(image, dist_text, cv::Point(armor.apex[0].x, armor.apex[0].y - 20), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
                }
            }
            
            // 绘制车辆边界框
            for (const auto& car : cars) {
                cv::rectangle(image, car.bounding_rect, cv::Scalar(0, 255, 0), 2);
            }
            
            // 绘制跟踪结果
            for (const auto& track : track_results.first) {
                CXYD coord = track.location.cxy;
                cv::circle(image, cv::Point(coord.cx, coord.cy), 12, cv::Scalar(255, 0, 255), 3);
                
                // 显示距离信息
                XYZ xyz_data = track.location.xyz_imu;
                double distance = std::sqrt(xyz_data.x * xyz_data.x + xyz_data.y * xyz_data.y);
                std::string dist_text = std::to_string((int)distance) + "m";
                cv::putText(image, dist_text, cv::Point(coord.cx, coord.cy - 20), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);
            }
        }

        bool calculateBallisticSolution(const XYZ& target_xyz)
        {
            // 弹道计算参数
            const double PI = 3.1415926;
            const double GRAVITY = 9.794;
            const double C_D = 0.42;
            const double RHO = 1.169;
            const double bullet_mass = 3.2e-3;
            const double bullet_diameter = 16.8e-3;
            const double bullet_speed = 23.0;
            const double tol = 1e-6;
            const int max_iter = 100;
            
            double distance = std::sqrt(target_xyz.x * target_xyz.x + target_xyz.y * target_xyz.y);
            double theta = 0.0;
            double delta_z = 0.0;
            double k1 = C_D * RHO * (PI * bullet_diameter * bullet_diameter) / 8 / bullet_mass;
            
            // 迭代计算pitch角
            bool calc_success = false;
            for (int i = 0; i < max_iter; i++) {
                double t = (exp(k1 * distance) - 1) / (k1 * bullet_speed * cos(theta));
                delta_z = target_xyz.z - bullet_speed * sin(theta) * t / cos(theta) + 
                         0.5 * GRAVITY * t * t / cos(theta) / cos(theta);
                
                if (fabs(delta_z) < tol) {
                    calc_success = true;
                    break;
                }
                
                theta -= delta_z / (-(bullet_speed * t) / pow(cos(theta), 2) + 
                                   GRAVITY * t * t / (bullet_speed * bullet_speed) * 
                                   sin(theta) / pow(cos(theta), 3));
            }
            
            if (calc_success) {
                double aim_pitch_rad = theta;
                double aim_yaw_rad = atan2(target_xyz.y, target_xyz.x);
                
                target_pitch = aim_pitch_rad * 180 / PI;
                target_yaw = aim_yaw_rad * 180 / PI;
                
                // 处理yaw角度连续性
                target_yaw = current_gimbal_angles.Yaw + 
                           std::remainder(target_yaw - current_gimbal_angles.Yaw, 360.0);
                
                return true;
            }
            
            return false;
        }

        double fitPitch(double z_height, double horizontal_distance)
        {
            const auto& p = shoot_table_params.pitch;
            double z2 = z_height * z_height;
            double d2 = horizontal_distance * horizontal_distance;
            double zd = z_height * horizontal_distance;
            
            return p.intercept + 
                   p.coef_z * z_height + 
                   p.coef_d * horizontal_distance + 
                   p.coef_z2 * z2 + 
                   p.coef_zd * zd + 
                   p.coef_d2 * d2;
        }

        double fitYaw(double z_height, double horizontal_distance)
        {
            const auto& y = shoot_table_params.yaw;
            double z2 = z_height * z_height;
            double d2 = horizontal_distance * horizontal_distance;
            double zd = z_height * horizontal_distance;
            
            return y.intercept + 
                   y.coef_z * z_height + 
                   y.coef_d * horizontal_distance + 
                   y.coef_z2 * z2 + 
                   y.coef_zd * zd + 
                   y.coef_d2 * d2;
        }

        void checkAndUpdateParams()
        {
            ros::Time current_time = ros::Time::now();
            if ((current_time - last_param_check).toSec() >= param_check_interval) {
                loadShootTableParams();
                last_param_check = current_time;
            }
        }
        
        void printCurrentParams()
        {
            std::cout << "\n=== Current Shoot Table Parameters ===\n";
            std::cout << "Enable: " << (shoot_table_params.enable ? "true" : "false") << "\n";
            std::cout << "Pitch coefficients:\n";
            std::cout << "  intercept: " << shoot_table_params.pitch.intercept << "\n";
            std::cout << "  coef_z: " << shoot_table_params.pitch.coef_z << "\n";
            std::cout << "  coef_d: " << shoot_table_params.pitch.coef_d << "\n";
            std::cout << "  coef_z2: " << shoot_table_params.pitch.coef_z2 << "\n";
            std::cout << "  coef_zd: " << shoot_table_params.pitch.coef_zd << "\n";
            std::cout << "  coef_d2: " << shoot_table_params.pitch.coef_d2 << "\n";
            std::cout << "Yaw coefficients:\n";
            std::cout << "  intercept: " << shoot_table_params.yaw.intercept << "\n";
            std::cout << "  coef_z: " << shoot_table_params.yaw.coef_z << "\n";
            std::cout << "  coef_d: " << shoot_table_params.yaw.coef_d << "\n";
            std::cout << "  coef_z2: " << shoot_table_params.yaw.coef_z2 << "\n";
            std::cout << "  coef_zd: " << shoot_table_params.yaw.coef_zd << "\n";
            std::cout << "  coef_d2: " << shoot_table_params.yaw.coef_d2 << "\n";
            std::cout << "=====================================\n\n";
        }
        
        void setParameter(const std::string& param_path, double value)
        {
            ros::NodeHandle nh;
            nh.setParam("shoot_table_adjust/" + param_path, value);
            loadShootTableParams();
            std::cout << "✓ Parameter updated: " << param_path << " = " << value << "\n";
        }
        
        void interactiveParamEdit()
        {
            std::cout << "\n=== Interactive Parameter Editor ===\n";
            std::cout << "Available parameters:\n";
            std::cout << "1. enable (true/false)\n";
            std::cout << "2. pitch/intercept\n";
            std::cout << "3. pitch/coef_z\n";
            std::cout << "4. pitch/coef_d\n";
            std::cout << "5. pitch/coef_z2\n";
            std::cout << "6. pitch/coef_zd\n";
            std::cout << "7. pitch/coef_d2\n";
            std::cout << "8. yaw/intercept\n";
            std::cout << "9. yaw/coef_z\n";
            std::cout << "10. yaw/coef_d\n";
            std::cout << "11. yaw/coef_z2\n";
            std::cout << "12. yaw/coef_zd\n";
            std::cout << "13. yaw/coef_d2\n";
            std::cout << "Enter parameter path (e.g., 'pitch/intercept'): ";
            
            std::string param_path = keyboard.getLine();
            
            if (param_path == "enable") {
                std::cout << "Enter value (true/false): ";
                std::string value_str = keyboard.getLine();
                bool value = (value_str == "true");
                ros::NodeHandle nh;
                nh.setParam("shoot_table_adjust/enable", value);
                loadShootTableParams();
                std::cout << "✓ Parameter updated: enable = " << (value ? "true" : "false") << "\n";
            } else {
                std::cout << "Enter value: ";
                std::string value_str = keyboard.getLine();
                try {
                    double value = std::stod(value_str);
                    setParameter(param_path, value);
                } catch (const std::exception& e) {
                    std::cout << "✗ Invalid value: " << e.what() << "\n";
                }
            }
            std::cout << "Press any key to continue...\n";
        }

        void saveShootingRecord()
        {
            if (!is_aiming.load()) {
                std::cout << "✗ No target locked. Cannot save record.\n";
                return;
            }

            try {
                ShootingRecord record;
                
                record.z_height = current_target_world.z;
                record.horizontal_distance = std::sqrt(current_target_world.x * current_target_world.x + 
                                                      current_target_world.y * current_target_world.y);
                record.relative_yaw = yaw_adjustment;
                record.relative_pitch = pitch_adjustment;
                record.target_world_coord = current_target_world;
                record.absolute_yaw = current_gimbal_angles.Yaw;
                record.absolute_pitch = current_gimbal_angles.Pitch;
                record.target_yaw = target_yaw;
                record.fitted_pitch = fitPitch(record.z_height, record.horizontal_distance);
                record.fitted_yaw = fitYaw(record.z_height, record.horizontal_distance);
                record.timestamp = ros::Time::now();
                
                // 保存到文件
                std::ofstream file(csv_filename, std::ios::app);
                if (file.is_open()) {
                    file << std::fixed << std::setprecision(6)
                         << record.timestamp.toSec() << ","
                         << record.z_height << ","
                         << record.horizontal_distance << ","
                         << record.relative_yaw << ","
                         << record.relative_pitch << ","
                         << record.target_world_coord.x << ","
                         << record.target_world_coord.y << ","
                         << record.target_world_coord.z << ","
                         << record.absolute_yaw << ","
                         << record.absolute_pitch << ","
                         << record.target_yaw << ","
                         << record.fitted_pitch << ","
                         << record.fitted_yaw << std::endl;
                    file.close();
                }
                
                records.push_back(record);
                
                std::cout << "✓ Record saved! Z: " << record.z_height 
                         << "m, Distance: " << record.horizontal_distance << "m\n";
                std::cout << "  Adjustment - Yaw: " << record.relative_yaw 
                         << "°, Pitch: " << record.relative_pitch << "°\n";
                std::cout << "  Total records: " << records.size() << "\n";
                
                // 停止射击并重置状态
                is_shooting.store(false);
                aim_only_mode.store(false);
                sendStopCommand();
                is_aiming.store(false);
                control_valid.store(false);
                std::cout << "Control disabled. Ready for next target.\n";
                
            } catch (const std::exception& e) {
                std::cout << "✗ Error saving record: " << e.what() << "\n";
            }
        }

        void handleKeyboard()
        {
            char key = keyboard.getKey();
            if (key == 0) return;
            
            std::cout << "Key pressed: '" << key << "'\n";
            
            switch (key) {
                case 'a':
                    std::cout << "🎯 Triggering single aim detection...\n";
                    should_aim_once.store(true);
                    break;
                    
                case 'g':  // 只瞄准，不射击
                    if (is_aiming.load()) {
                        std::cout << "🎯 Aiming only (no shooting)...\n";
                        aim_only_mode.store(true);
                        is_shooting.store(false);
                        
                        // 确保立即发送控制命令
                        sendAimOnlyCommand();
                        updateFireControl(false);  // 不开火
                        
                        std::cout << "✓ Aim command published to /ly/control/angles\n";
                    } else {
                        std::cout << "✗ No target locked. Aim first with 'a'.\n";
                    }
                    break;
                    
                case 'f':  // 瞄准并射击
                    if (is_aiming.load()) {
                        std::cout << "🔥 Firing! Sending target command and fire control...\n";
                        aim_only_mode.store(false);
                        is_shooting.store(true);
                        shoot_start_time = ros::Time::now();
                        
                        // 确保立即发送控制命令和火控
                        sendControlCommand();
                        updateFireControl(true);  // 开火 - 会翻转状态
                        
                        std::cout << "✓ Target command published to /ly/predictor/target\n";
                        std::cout << "✓ Fire control published to /ly/control/firecode\n";
                    } else {
                        std::cout << "✗ No target locked. Aim first with 'a'.\n";
                    }
                    break;
                    
                case 't':  // 测试发布功能 - 强制发布
                    {
                        std::cout << "🧪 Force testing all publications...\n";
                        
                        // 强制发布云台角度
                        gimbal_driver::GimbalAngles test_msg;
                        test_msg.Yaw = current_gimbal_angles.Yaw + yaw_adjustment;
                        test_msg.Pitch = current_gimbal_angles.Pitch + pitch_adjustment;
                        test_msg.header.stamp = ros::Time::now();
                        node.Publisher<ly_control_angles>().publish(test_msg);
                        std::cout << "✓ Published to /ly/control/angles - Yaw: " << test_msg.Yaw 
                                 << "°, Pitch: " << test_msg.Pitch << "°\n";
                        
                        // 强制发布火控
                        flipFireStatus();
                        sendFireControlCommand();
                        std::cout << "✓ Published to /ly/control/firecode - Status: " 
                                 << static_cast<int>(fire_control.fire_status) << "\n";
                        
                        // 强制发布目标
                        auto_aim_common::Target test_target;
                        test_target.header.stamp = ros::Time::now();
                        test_target.header.frame_id = "camera";
                        test_target.yaw = test_msg.Yaw;
                        test_target.pitch = test_msg.Pitch;
                        test_target.status = true;
                        test_target.buff_follow = false;
                        node.Publisher<ly_predictor_target>().publish(test_target);
                        std::cout << "✓ Published to /ly/predictor/target\n";
                        
                        std::cout << "🧪 All test publications completed!\n";
                    }
                    break;
                    
                case 'x':  // 停止射击命令
                    std::cout << "🛑 Stopping all commands...\n";
                    is_shooting.store(false);
                    aim_only_mode.store(false);
                    updateFireControl(false);  // 停止开火
                    sendStopCommand();
                    break;
                    
                case 'w':
                    pitch_adjustment += adjustment_step;
                    std::cout << "⬆️ Pitch adjustment: " << std::fixed << std::setprecision(1) 
                             << pitch_adjustment << "°\n";
                    // 如果当前在瞄准模式，立即更新控制命令
                    if (control_valid.load() && aim_only_mode.load()) {
                        sendAimOnlyCommand();
                    } else if (control_valid.load() && is_shooting.load()) {
                        sendControlCommand();
                    }
                    break;
                    
                case 's':
                    pitch_adjustment -= adjustment_step;
                    std::cout << "⬇️ Pitch adjustment: " << std::fixed << std::setprecision(1) 
                             << pitch_adjustment << "°\n";
                    // 如果当前在瞄准模式，立即更新控制命令
                    if (control_valid.load() && aim_only_mode.load()) {
                        sendAimOnlyCommand();
                    } else if (control_valid.load() && is_shooting.load()) {
                        sendControlCommand();
                    }
                    break;
                    
                case 'd':
                    yaw_adjustment += adjustment_step;
                    std::cout << "➡️ Yaw adjustment: " << std::fixed << std::setprecision(1) 
                             << yaw_adjustment << "°\n";
                    // 如果当前在瞄准模式，立即更新控制命令
                    if (control_valid.load() && aim_only_mode.load()) {
                        sendAimOnlyCommand();
                    } else if (control_valid.load() && is_shooting.load()) {
                        sendControlCommand();
                    }
                    break;
                    
                case 'j':
                    yaw_adjustment -= adjustment_step;
                    std::cout << "⬅️ Yaw adjustment: " << std::fixed << std::setprecision(1) 
                             << yaw_adjustment << "°\n";
                    // 如果当前在瞄准模式，立即更新控制命令
                    if (control_valid.load() && aim_only_mode.load()) {
                        sendAimOnlyCommand();
                    } else if (control_valid.load() && is_shooting.load()) {
                        sendControlCommand();
                    }
                    break;
                    
                case '/':
                    if (is_aiming.load()) {
                        double z_height = current_target_world.z;
                        double horizontal_distance = std::sqrt(current_target_world.x * current_target_world.x + 
                                                              current_target_world.y * current_target_world.y);
                        yaw_adjustment = fitYaw(z_height, horizontal_distance);
                        pitch_adjustment = fitPitch(z_height, horizontal_distance);
                        std::cout << "Auto adjustment applied: Yaw: " << yaw_adjustment 
                                 << "°, Pitch: " << pitch_adjustment << "°\n";
                    } else {
                        std::cout << "✗ No target locked. Press 'a' to aim first.\n";
                    }
                    break;
                    
                case 'h':
                    std::cout << "💾 Saving shooting record...\n";
                    saveShootingRecord();
                    break;
                    
                case 'r':
                    pitch_adjustment = 0.0;
                    yaw_adjustment = 0.0;
                    control_valid.store(false);
                    is_aiming.store(false);
                    is_shooting.store(false);
                    aim_only_mode.store(false);
                    
                    // 重置火控状态
                    fire_control.fire_status = 0;
                    fire_control.last_fire_command = false;
                    sendFireControlCommand();
                    
                    sendStopCommand();
                    std::cout << "🔄 All systems reset\n";
                    break;
                    
                case 'p':
                    printCurrentParams();
                    break;
                    
                case 'e':
                    interactiveParamEdit();
                    break;
                    
                case 'l':
                    loadShootTableParams();
                    std::cout << "✓ Parameters reloaded from config\n";
                    break;
                    
                case 'q':
                    std::cout << "👋 Quitting...\n";
                    
                    // 停止所有控制
                    is_shooting.store(false);
                    is_aiming.store(false);
                    updateFireControl(false);
                    
                    sendStopCommand();
                    keyboard.restore();
                    ros::shutdown();
                    break;
                    
                default:
                    std::cout << "❓ Unknown key. Press 'q' to quit.\n";
                    break;
            }
        }

        void printInstructions()
        {
            std::cout << "\n=== Shooting Table Calibration System ===\n";
            std::cout << "🔥 Flip-based Fire Control System (00 <-> 11) 🔥\n";
            std::cout << "Keyboard Controls:\n";
            std::cout << "  'a' - Trigger single aim detection\n";
            std::cout << "  'g' - Aim only (no shooting) - sends to /ly/control/angles\n";
            std::cout << "  'f' - Fire (aim + shoot) - flips fire status to activate\n";
            std::cout << "  't' - Test publish + flip fire control\n";
            std::cout << "  'x' - Stop all shooting and fire control\n";
            std::cout << "  'w' - Increase pitch (+0.1°)\n";
            std::cout << "  's' - Decrease pitch (-0.1°)\n";
            std::cout << "  'd' - Increase yaw (+0.1°)\n";
            std::cout << "  'j' - Decrease yaw (-0.1°)\n";
            std::cout << "  '/' - Auto adjust using shoot table\n";
            std::cout << "  'h' - Confirm hit and save record\n";
            std::cout << "  'r' - Reset all systems\n";
            std::cout << "  'p' - Print current parameters\n";
            std::cout << "  'e' - Edit parameters interactively\n";
            std::cout << "  'l' - Reload parameters from config\n";
            std::cout << "  'q' - Quit\n";
            std::cout << "==========================================\n\n";
            std::cout << "💡 Fire Control Logic:\n";
            std::cout << "  • Fire status uses only lowest 2 bits\n";
            std::cout << "  • 00 (0) and 11 (3) are the two states\n";
            std::cout << "  • Flip between states to trigger fire\n";
            std::cout << "  • Based on behavior_tree FlipFireStatus\n\n";
        }

    public:
        void spin()
        {
            ros::Rate rate(78);
            
            std::cout << "🚀 Starting independent shooting table calibration...\n";
            std::cout << "🔥 Flip-based fire control system initialized\n";
            std::cout << "📡 Publishing to topics:\n";
            std::cout << "  • /ly/control/angles (gimbal control)\n";
            std::cout << "  • /ly/control/firecode (fire control)\n";
            std::cout << "  • /ly/predictor/target (target commands)\n\n";
            
            // 启动时测试发布一次，确保话题创建
            std::cout << "🧪 Initial topic test...\n";
            
            gimbal_driver::GimbalAngles init_msg;
            init_msg.Yaw = current_gimbal_angles.Yaw;
            init_msg.Pitch = current_gimbal_angles.Pitch;
            init_msg.header.stamp = ros::Time::now();
            node.Publisher<ly_control_angles>().publish(init_msg);
            
            std_msgs::UInt8 init_fire;
            init_fire.data = 0;
            node.Publisher<ly_control_firecode>().publish(init_fire);
            
            std::cout << "✓ Initial publications sent\n\n";
            
            while (ros::ok()) {
                try {
                    // 处理键盘输入
                    handleKeyboard();
                    
                    // 定期检查参数更新
                    checkAndUpdateParams();
                    
                    // 处理图像检测
                    processImageDetections();
                    
                    // 检查射击时间
                    if (is_shooting.load()) {
                        ros::Time current_time = ros::Time::now();
                        if ((current_time - shoot_start_time).toSec() >= shoot_duration) {
                            is_shooting.store(false);
                            updateFireControl(false);  // 停止开火
                            std::cout << "⏰ Auto-stop after " << shoot_duration << "s shooting\n";
                            sendStopCommand();
                        } else {
                            // 射击期间持续发送控制命令（但不重复翻转火控）
                            sendControlCommand();
                        }
                    }
                    
                    ros::spinOnce();
                    rate.sleep();
                } catch (const std::exception& e) {
                    std::cerr << "Error in main loop: " << e.what() << std::endl;
                    break;
                }
            }
            
            // 清理
            std::cout << "\n🛑 Shutting down fire control system\n";
            updateFireControl(false);
            sendStopCommand();
            keyboard.restore();
            
            std::cout << "\n=== Shooting Table Calibration Summary ===\n";
            std::cout << "Total records: " << records.size() << "\n";
            std::cout << "Records saved to: " << csv_filename << "\n";
            std::cout << "Fire control: Flip-based system (00 <-> 11)\n";
            std::cout << "==========================================\n";
        }
    };
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "shooting_table_calib_node");
    
    try {
        ShootingTableCalibNode calib_node(argc, argv);
        calib_node.spin();
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}