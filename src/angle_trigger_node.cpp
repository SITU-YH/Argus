#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <filesystem>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <sstream>
#include <algorithm>

namespace fs = std::filesystem;

// 用 constexpr 替代非标准的 M_PI，C++17 保证跨平台一致
constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;

// 异步存图任务包
struct SaveTask {
    sensor_msgs::msg::Image::ConstSharedPtr msg;
    std::string filename;
};

class AngleTriggerNode : public rclcpp::Node {
public:
    AngleTriggerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("angle_trigger_node", options) {
        
        // 1. 动态读取参数
        this->declare_parameter("fps", 10.0);
        fps_ = this->get_parameter("fps").as_double();

        this->declare_parameter("trigger_interval_deg", 90.0);
        trigger_interval_deg_ = this->get_parameter("trigger_interval_deg").as_double();

        // 参数合法性校验
        if (trigger_interval_deg_ <= 0.0 || trigger_interval_deg_ > 360.0) {
            RCLCPP_WARN(this->get_logger(),
                "trigger_interval_deg=%.1f 不合法，回退为默认值 90°", trigger_interval_deg_);
            trigger_interval_deg_ = 90.0;
        }

        trigger_interval_rad_ = trigger_interval_deg_ * DEG_TO_RAD;
        num_streams_ = static_cast<int>(std::ceil(360.0 / trigger_interval_deg_));

        // 验证 360° 恰好被角度间隔整除，否则采集方向会有重叠或遗漏
        double remainder = std::fmod(360.0, trigger_interval_deg_);
        if (remainder > 1e-9 && std::abs(remainder - trigger_interval_deg_) > 1e-9) {
            RCLCPP_WARN(this->get_logger(),
                "360° 不能被 %.1f° 整除！最后一段仅有 %.1f°，各 stream 采集帧数可能不均",
                trigger_interval_deg_, remainder);
        }
        
        // 2. 自动创建存放目录，并清空历史残留图片
        const char* home_dir = getenv("HOME");
        base_dir_ = home_dir ? std::string(home_dir) + "/argus_data" : "./argus_data";
        
        for (int i = 0; i < num_streams_; ++i) {
            std::string dir_name = base_dir_ + "/stream_" + std::to_string(static_cast<int>(i * trigger_interval_deg_));
            fs::create_directories(dir_name);
            
            // 清理旧照片
            for (const auto& entry : fs::directory_iterator(dir_name)) {
                if (entry.path().extension() == ".jpg") {
                    fs::remove(entry.path());
                }
            }
            stream_dirs_.push_back(dir_name);
            image_counts_.push_back(0);
        }
        
        // 3. 启动后台异步存图线程
        save_thread_ = std::thread(&AngleTriggerNode::save_thread_func, this);
        
        // 4. 🌟 终极并发优化：创建独立的互斥回调组
        // 这能保证 odom 和 image 真正并行运行，而不会互相阻塞
        odom_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        image_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions odom_options;
        odom_options.callback_group = odom_cb_group_;
        
        rclcpp::SubscriptionOptions image_options;
        image_options.callback_group = image_cb_group_;

        // 订阅里程计
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) { odom_callback(msg); },
            odom_options);

        // 订阅相机图像 (海康驱动使用 RELIABLE QoS，必须一致)
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/driver/hikvision/argus_camera/image_raw", qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { image_callback(msg); },
            image_options);

        RCLCPP_INFO(this->get_logger(), "🚀 [C++工业版] 全向采集已启动！");
        RCLCPP_INFO(this->get_logger(), "⚡ 已启用 [多线程物理双核并发] + [内存零拷贝] + [后台异步写盘]");
        RCLCPP_INFO(this->get_logger(), "⚙️ 当前物理转速/视频帧率设定为: %.1f FPS (r/s)", fps_);
    }

    ~AngleTriggerNode() {
        // 捕获 Ctrl+C 时，安全关闭后台线程
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stop_thread_ = true;
        }
        cv_.notify_all();
        if (save_thread_.joinable()) {
            save_thread_.join();
        }
        // 线程死透后，开始生成视频
        generate_all_videos();
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto q = msg->pose.pose.orientation;
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        double current_yaw = std::atan2(siny_cosp, cosy_cosp);

        if (first_odom_) {
            last_odom_yaw_ = current_yaw;
            first_odom_ = false;
            return;
        }

        double diff = current_yaw - last_odom_yaw_;
        diff = std::atan2(std::sin(diff), std::cos(diff));  // 归一化到 [-π, π]

        // 合理性检查：若单帧角度变化超过阈值，说明 SLAM 可能发生了重定位/跳变
        // 合理上限：假设机器人最大转速 720°/s，10Hz 下每帧 < 1.26 rad
        constexpr double MAX_DIFF_PER_TICK = 1.5;  // ~86° per tick at 10Hz
        if (std::abs(diff) > MAX_DIFF_PER_TICK) {
            RCLCPP_WARN(this->get_logger(),
                "角度跳变过大 (%.1f deg)，疑似 SLAM 重定位或丢帧，跳过此帧以免累计偏差",
                diff * RAD_TO_DEG);
            last_odom_yaw_ = current_yaw;
            return;
        }

        accumulated_yaw_ += diff;
        last_odom_yaw_ = current_yaw;

        debug_cnt_++;
        if (debug_cnt_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "🔍 [探针] 当前原始Yaw: %.1f deg, 真实累计已转: %.1f deg",
                        current_yaw * RAD_TO_DEG, accumulated_yaw_ * RAD_TO_DEG);
        }

        int current_expected_triggers = static_cast<int>(std::abs(accumulated_yaw_) / trigger_interval_rad_);

        if (current_expected_triggers > expected_trigger_count_) {
            int new_tasks = current_expected_triggers - expected_trigger_count_;
            pending_captures_ += new_tasks;
            expected_trigger_count_ = current_expected_triggers;
            
            RCLCPP_INFO(this->get_logger(), "🎯 物理跨越 %.1f° 边界！已向后台队列派发抓拍任务...",
                        current_expected_triggers * trigger_interval_deg_);
        }
    }

    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (pending_captures_ <= 0) return;

        int target_idx = current_stream_idx_;
        std::string target_dir = stream_dirs_[target_idx];
        int frame_count = image_counts_[target_idx];
        
        // 格式化文件名 (例如: frame_00005.jpg)
        std::ostringstream ss;
        ss << target_dir << "/frame_" << std::setw(5) << std::setfill('0') << frame_count << ".jpg";
        std::string filename = ss.str();

        // 【极速闪避】：这部分在主进程跑，只把指针和名字扔进队列，唤醒后台线程，然后立马撤退！
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            save_queue_.push({msg, filename});
        }
        cv_.notify_one();

        RCLCPP_INFO(this->get_logger(), "📸 派发存盘任务 [方向 %.1f°]: frame_%05d.jpg",
                    target_idx * trigger_interval_deg_, frame_count);

        image_counts_[target_idx]++;
        
        if ((target_idx + 1) % num_streams_ == 0) {
            lap_count_++;
            RCLCPP_INFO(this->get_logger(), "🟢 =======================================");
            RCLCPP_INFO(this->get_logger(), "🟢 完美闭环！已成功完成第 %d 圈全向数据采集！", lap_count_);
            RCLCPP_INFO(this->get_logger(), "🟢 =======================================");
        }

        current_stream_idx_ = (current_stream_idx_ + 1) % num_streams_;
        pending_captures_--;
    }

    // 后台大管家线程：专职对付耗时的磁盘 I/O
    void save_thread_func() {
        while (true) {
            SaveTask task;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                // 深度休眠，不吃一点CPU，直到队列里有活儿，或者系统喊停
                cv_.wait(lock, [this]() { return !save_queue_.empty() || stop_thread_; });
                
                if (stop_thread_ && save_queue_.empty()) {
                    break;
                }
                
                task = save_queue_.front();
                save_queue_.pop();
            }

            // 在后台执行转换和写盘
            try {
                // 🌟 终极零拷贝魔法：传入 "" 表示维持原始格式（单通道灰度或Bayer），节约 CPU
                cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(task.msg, "");
                cv::imwrite(task.filename, cv_ptr->image);
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge 转换失败: %s", e.what());
            } catch (std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "存图异常: %s", e.what());
            }
        }
    }

    void generate_all_videos() {
        RCLCPP_INFO(this->get_logger(), "======================================");
        RCLCPP_INFO(this->get_logger(), "🏁 收到停止指令。本次真实完成 %d 圈。开始按 %.1f FPS 合成视频...", lap_count_, fps_);
        RCLCPP_INFO(this->get_logger(), "======================================");

        for (int i = 0; i < num_streams_; ++i) {
            int angle = static_cast<int>(i * trigger_interval_deg_);
            std::string output_video = base_dir_ + "/video_" + std::to_string(angle) + "deg.mp4";
            
            std::vector<std::string> images;
            for (const auto& entry : fs::directory_iterator(stream_dirs_[i])) {
                if (entry.path().extension() == ".jpg") {
                    images.push_back(entry.path().string());
                }
            }
            std::sort(images.begin(), images.end());

            if (images.empty()) {
                RCLCPP_WARN(this->get_logger(), "⚠️ 方向 %d° 没有拍到照片，跳过合成。", angle);
                continue;
            }

            cv::Mat first_frame = cv::imread(images[0]);
            if (first_frame.empty()) continue;

            // 🌟 容错保护：自动识别图片是单通道还是三通道，防止 VideoWriter 崩溃
            bool is_color = (first_frame.channels() == 3);
            cv::VideoWriter writer(output_video, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps_, first_frame.size(), is_color);

            RCLCPP_INFO(this->get_logger(), "🎬 正在合成 %d° 视频 (共 %zu 帧) -> %s", angle, images.size(), output_video.c_str());

            for (const auto& img_path : images) {
                cv::Mat frame = cv::imread(img_path);
                if (!frame.empty()) writer.write(frame);
            }
            writer.release();
        }
        RCLCPP_INFO(this->get_logger(), "✅ 所有视频合成完毕！可以安全退出了。");
    }

    // --- 成员变量 ---
    double fps_;
    double trigger_interval_deg_;
    double trigger_interval_rad_;
    int num_streams_;
    std::string base_dir_;
    std::vector<std::string> stream_dirs_;
    
    double accumulated_yaw_ = 0.0;
    bool first_odom_ = true;
    double last_odom_yaw_ = 0.0;
    int expected_trigger_count_ = 0;
    std::atomic<int> pending_captures_{0};
    int current_stream_idx_ = 0;
    std::vector<int> image_counts_;
    int lap_count_ = 0;
    int debug_cnt_ = 0;

    // 回调组
    rclcpp::CallbackGroup::SharedPtr odom_cb_group_;
    rclcpp::CallbackGroup::SharedPtr image_cb_group_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    std::thread save_thread_;
    std::mutex queue_mutex_;
    std::condition_variable cv_;
    std::queue<SaveTask> save_queue_;
    bool stop_thread_ = false;
};

// 作为一个独立的节点入口
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AngleTriggerNode>();
    
    // 🌟 终极多核并发：使用多线程执行器
    // ROS2 会自动将分配在不同 CallbackGroup 的回调任务扔给不同的 CPU 核心
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
