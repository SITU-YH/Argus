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

namespace fs = std::filesystem;

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
        
        // --- 核心配置参数 ---
        trigger_interval_deg_ = 90.0;
        trigger_interval_rad_ = trigger_interval_deg_ * M_PI / 180.0;
        num_streams_ = static_cast<int>(360.0 / trigger_interval_deg_);
        
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
        
        // 4. 订阅话题
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10, std::bind(&AngleTriggerNode::odom_callback, this, std::placeholders::_1));
            
        rclcpp::QoS qos(5);
        qos.best_effort(); // 匹配海康相机的 QoS
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/driver/hikvision/argus_camera/image_raw", qos, 
            std::bind(&AngleTriggerNode::image_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "🚀 [C++工业版] 全向采集已启动！(启用零拷贝 + 多线程异步存盘)");
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
        diff = std::atan2(std::sin(diff), std::cos(diff));

        // 🚀 核心物理规则：强行掰正由掉帧导致的时光倒流
        if (diff < -M_PI / 2.0) {
            diff += 2 * M_PI;
        }

        accumulated_yaw_ += diff;
        last_odom_yaw_ = current_yaw;

        debug_cnt_++;
        if (debug_cnt_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "🔍 [探针] 当前原始Yaw: %.1f°, 真实累计已转: %.1f°",
                        current_yaw * 180.0 / M_PI, accumulated_yaw_ * 180.0 / M_PI);
        }

        int current_expected_triggers = static_cast<int>(std::abs(accumulated_yaw_) / trigger_interval_rad_);

        if (current_expected_triggers > expected_trigger_count_) {
            int new_tasks = current_expected_triggers - expected_trigger_count_;
            pending_captures_ += new_tasks;
            expected_trigger_count_ = current_expected_triggers;
            
            RCLCPP_INFO(this->get_logger(), "🎯 物理跨越 %.1f° 边界！已向队列派发任务...",
                        current_expected_triggers * trigger_interval_deg_);
        }
    }

    void image_callback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        if (pending_captures_ <= 0) return;

        int target_idx = current_stream_idx_;
        std::string target_dir = stream_dirs_[target_idx];
        int frame_count = image_counts_[target_idx];
        
        // 格式化文件名 (类似于 Python 的 :05d)
        std::ostringstream ss;
        ss << target_dir << "/frame_" << std::setw(5) << std::setfill('0') << frame_count << ".jpg";
        std::string filename = ss.str();

        // 【极速闪避】：只把指针和名字扔进队列，唤醒后台线程，然后立马撤退！
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

    // 后台大管家：专职对付耗时的磁盘 I/O
    void save_thread_func() {
        while (true) {
            SaveTask task;
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                // 深度休眠，直到队列里有活儿，或者系统喊停
                cv_.wait(lock, [this]() { return !save_queue_.empty() || stop_thread_; });
                
                if (stop_thread_ && save_queue_.empty()) {
                    break;
                }
                
                task = save_queue_.front();
                save_queue_.pop();
            }

            // 在后台执行转换和写盘
            try {
                // 🌟 终极零拷贝魔法：toCvShare 直接引用原内存，不发生任何拷贝！
                cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(task.msg, "bgr8");
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

            cv::VideoWriter writer(output_video, cv::VideoWriter::fourcc('m', 'p', '4', 'v'), fps_, first_frame.size());

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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
