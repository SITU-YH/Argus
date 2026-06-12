#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <deque>
#include <filesystem>
#include <fstream>
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
#include <map>
#include <limits>
#include <future>
#include <chrono>

namespace fs = std::filesystem;

constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;

struct ImageFrame {
    sensor_msgs::msg::Image::ConstSharedPtr msg;
    rclcpp::Time stamp;
};

struct TriggerRequest {
    rclcpp::Time trigger_time;
    double yaw_deg;
    int stream_idx;
};

struct SaveTask {
    sensor_msgs::msg::Image::ConstSharedPtr img_msg;
    std::string img_path;
    double yaw_deg;
    int64_t trigger_ns;
    int stream_idx;
};

class AngleTriggerNode : public rclcpp::Node {
public:
    AngleTriggerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("angle_trigger_node", options) {

        this->declare_parameter("fps", 10.0);
        fps_ = this->get_parameter("fps").as_double();

        this->declare_parameter("trigger_interval_deg", 90.0);
        interval_deg_ = this->get_parameter("trigger_interval_deg").as_double();
        if (interval_deg_ <= 0 || interval_deg_ > 360) interval_deg_ = 90.0;
        interval_rad_ = interval_deg_ * DEG_TO_RAD;
        num_streams_ = static_cast<int>(std::ceil(360.0 / interval_deg_));

        // 🌟 新增：读取旋转参数，直接在此节点后台完成旋转
        this->declare_parameter("rotate_code", 2);
        int code = this->get_parameter("rotate_code").as_int();
        if (code >= 0 && code <= 2) {
            rotate_code_ = static_cast<cv::RotateFlags>(code);
            do_rotate_ = true;
        } else {
            do_rotate_ = false; // 不做旋转
        }

        // 创建存储目录
        const char* home = getenv("HOME");
        base_dir_ = home ? std::string(home) + "/argus_data" : "./argus_data";
        for (int i = 0; i < num_streams_; ++i) {
            std::string dir = base_dir_ + "/stream_" + std::to_string(static_cast<int>(i * interval_deg_));
            fs::create_directories(dir);
            for (const auto& e : fs::directory_iterator(dir)) {
                auto ext = e.path().extension();
                if (ext == ".jpg" || ext == ".mp4") fs::remove(e.path());
                if (e.path().filename() == "metadata.json") fs::remove(e.path());
            }
            stream_dirs_.push_back(dir);
            image_counts_.push_back(0);
        }
        stream_metadata_.resize(num_streams_);

        // 后台存图线程
        save_thread_ = std::thread(&AngleTriggerNode::save_loop, this);

        // 🌟 重新启用：回调组 (CallbackGroup) 分离里程计与图像线程
        odom_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        image_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        rclcpp::SubscriptionOptions odom_options;
        odom_options.callback_group = odom_cb_group_;
        rclcpp::SubscriptionOptions image_options;
        image_options.callback_group = image_cb_group_;

        // 订阅
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/Odometry", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr m) { odom_cb(m); }, odom_options);

        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        // 🌟 核心修改：直接订阅原始的 image_raw，不要用中间节点了！
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/driver/hikvision/argus_camera/image_raw", qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr m) { image_cb(m); }, image_options);

        RCLCPP_INFO(this->get_logger(), "🚀 全向采集(高阶版)启动 | 间隔=%.1f° | 流数=%d", interval_deg_, num_streams_);
        if (do_rotate_) RCLCPP_INFO(this->get_logger(), "🔄 图像将在后台进行无阻塞旋转 (Code: %d)", code);
    }

    ~AngleTriggerNode() {
        {
            std::lock_guard<std::mutex> lk(queue_mtx_);
            stop_ = true;
        }
        cv_.notify_all();
        if (save_thread_.joinable()) save_thread_.join();
        RCLCPP_INFO(this->get_logger(), "🛑 节点析构，写出元数据 + 合成视频...");
        write_metadata();
        auto video_future = std::async(std::launch::async, &AngleTriggerNode::generate_videos, this);
        if (video_future.wait_for(std::chrono::seconds(120)) == std::future_status::timeout) {
            RCLCPP_ERROR(this->get_logger(), "⏰ 视频合成超时，跳过");
        }
    }

private:
    // ─── 里程计回调 ───
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto& q = msg->pose.pose.orientation;
        double yaw = std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));

        if (first_odom_) {
            last_yaw_ = yaw;
            last_odom_stamp_ = msg->header.stamp;
            first_odom_ = false;
            return;
        }

        double diff = yaw - last_yaw_;
        diff = std::atan2(std::sin(diff), std::cos(diff));

        if (std::abs(diff) > 2.5) {
            if (dir_ema_ > 0.05 && diff < 0) {
                diff += 2.0 * PI;
            } else if (dir_ema_ < -0.05 && diff > 0) {
                diff -= 2.0 * PI;
            }
        }
        if (std::abs(diff) > 3.0) {
            RCLCPP_WARN(this->get_logger(), "⚠️ 跳变 %.0f° 跳过", diff * RAD_TO_DEG);
            last_yaw_ = yaw;
            last_odom_stamp_ = msg->header.stamp;
            return;
        }

        if (std::abs(diff) < 1.5) dir_ema_ = 0.85 * dir_ema_ + 0.15 * diff;

        // 🌟 保存累加前的值 + 上一帧时间戳，用于线性插值
        double acc_before = accumulated_;
        rclcpp::Time odom_prev_stamp = last_odom_stamp_;
        accumulated_ += diff;
        last_yaw_ = yaw;
        last_odom_stamp_ = msg->header.stamp;

        int new_trig = static_cast<int>(std::abs(accumulated_) / interval_rad_);
        for (int k = expected_trig_ + 1; k <= new_trig; ++k) {
            int stream = k % num_streams_;

            // 🌟 线性插值：精确估算到达目标角度那一刻的时间
            double target_abs = k * interval_rad_;
            double acc_before_abs = std::abs(acc_before);
            double acc_after_abs  = std::abs(accumulated_);
            rclcpp::Time interp_stamp = msg->header.stamp;  // fallback

            if (acc_after_abs > acc_before_abs) {
                double frac = (target_abs - acc_before_abs) / (acc_after_abs - acc_before_abs);
                if (frac >= 0.0 && frac <= 1.0) {
                    rclcpp::Time cur_stamp = msg->header.stamp;
                    double dt = (cur_stamp - odom_prev_stamp).seconds();
                    interp_stamp = odom_prev_stamp + rclcpp::Duration::from_seconds(frac * dt);
                }
            }

            TriggerRequest req;
            req.trigger_time = interp_stamp;
            req.yaw_deg = k * interval_deg_;
            req.stream_idx = stream;

            // 🌟 必须加锁！保护挂起任务队列
            {
                std::lock_guard<std::mutex> lock(pending_mtx_);
                pending_.push_back(req);
            }

            RCLCPP_INFO(this->get_logger(), "🎯 触发 %.0f° → stream_%d | 插值=%.0f%%",
                       req.yaw_deg, stream,
                       (acc_after_abs > acc_before_abs) ?
                       100.0 * (target_abs - acc_before_abs) / (acc_after_abs - acc_before_abs) : 100.0);
        }
        expected_trig_ = new_trig;

        if (++dbg_ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), "🔍 yaw=%.0f° 累计=%.0f°", yaw * RAD_TO_DEG, accumulated_ * RAD_TO_DEG);
        }
    }

    // ─── 图像回调 ───
    void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        // 环形缓冲操作
        ImageFrame f{msg, msg->header.stamp};
        buffer_.push_back(f);
        auto now = this->now();
        while (!buffer_.empty() && (now - buffer_.front().stamp).seconds() > 2.0)
            buffer_.pop_front();
        while (buffer_.size() > 30) buffer_.pop_front();

        // 🌟 加锁访问 pending_
        std::lock_guard<std::mutex> pending_lock(pending_mtx_);
        
        while (!pending_.empty()) {
            auto& req = pending_.front();

            const ImageFrame* best = nullptr;
            double best_d = std::numeric_limits<double>::max();
            for (auto& b : buffer_) {
                double d = std::abs((b.stamp - req.trigger_time).seconds());
                if (d < best_d) { best_d = d; best = &b; }
            }

            if (best && best_d < 0.3) {
                int n = image_counts_[req.stream_idx]++;
                std::ostringstream ss;
                ss << stream_dirs_[req.stream_idx] << "/frame_" << std::setw(5) << std::setfill('0') << n;

                SaveTask t;
                t.img_msg = best->msg;
                t.img_path = ss.str() + ".jpg";
                t.yaw_deg = req.yaw_deg;
                t.trigger_ns = req.trigger_time.nanoseconds();
                t.stream_idx = req.stream_idx;
                {
                    std::lock_guard<std::mutex> lk(queue_mtx_);
                    save_queue_.push(t);
                }
                cv_.notify_one();

                RCLCPP_INFO(this->get_logger(), "📸 [%.0f°] 匹配帧 (Δt=%.0fms)", req.yaw_deg, best_d * 1000);

                if ((req.stream_idx + 1) % num_streams_ == 0) {
                    laps_++;
                    RCLCPP_INFO(this->get_logger(), "🟢 第%d圈完成!", laps_);
                }
                pending_.pop_front();
            } else if (best && (now - req.trigger_time).seconds() > 1.0) {
                int n = image_counts_[req.stream_idx]++;
                std::ostringstream ss;
                ss << stream_dirs_[req.stream_idx] << "/frame_" << std::setw(5) << std::setfill('0') << n;

                SaveTask t;
                t.img_msg = best->msg;
                t.img_path = ss.str() + ".jpg";
                t.yaw_deg = req.yaw_deg;
                t.trigger_ns = req.trigger_time.nanoseconds();
                t.stream_idx = req.stream_idx;
                {
                    std::lock_guard<std::mutex> lk(queue_mtx_);
                    save_queue_.push(t);
                }
                cv_.notify_one();
                RCLCPP_WARN(this->get_logger(), "⚠️ 弱匹配超时 [%.0f°] Δt=%.0fms", req.yaw_deg, best_d * 1000);
                pending_.pop_front();
            } else {
                break;
            }
        }
    }

    // ─── 后台存图 ───
    void save_loop() {
        while (true) {
            SaveTask t;
            {
                std::unique_lock<std::mutex> lk(queue_mtx_);
                cv_.wait(lk, [this]{ return !save_queue_.empty() || stop_; });
                if (stop_ && save_queue_.empty()) break;
                t = save_queue_.front();
                save_queue_.pop();
            }

            try {
                // 零拷贝获取原图
                cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(t.img_msg, t.img_msg->encoding);
                
                // 🌟 如果需要旋转，在这里（后台线程）执行，绝不阻塞主线程！
                cv::Mat img_to_save;
                if (do_rotate_) {
                    cv::rotate(cv_ptr->image, img_to_save, rotate_code_);
                } else {
                    img_to_save = cv_ptr->image.clone();  // 深拷贝，避免修改共享内存
                }

                // 为了保证兼容性，如果是 RGB 格式，转存 JPG 前确保通道顺序为 BGR
                if (t.img_msg->encoding == "rgb8") {
                    cv::cvtColor(img_to_save, img_to_save, cv::COLOR_RGB2BGR);
                }
                
                cv::imwrite(t.img_path, img_to_save);

                // 🌟 收集元数据到内存，退出时统一写一个 metadata.json
                std::string fname = fs::path(t.img_path).filename().string();
                std::ostringstream meta;
                meta << "{\"frame\":\"" << fname
                     << "\",\"yaw_deg\":" << t.yaw_deg
                     << ",\"trigger_ns\":" << t.trigger_ns << "}";
                stream_metadata_[t.stream_idx].push_back(meta.str());
            } catch (std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "存盘失败: %s", e.what());
            }
        }
    }

    // ─── 退出时合成视频 ───
    void generate_videos() {
        RCLCPP_INFO(this->get_logger(), "🎬 合成视频 (共%d圈)...", laps_);
        for (int i = 0; i < num_streams_; ++i) {
            // 收集图片路径并按时间排序
            std::vector<std::pair<std::string, int64_t>> frames;
            for (const auto& e : fs::directory_iterator(stream_dirs_[i])) {
                if (e.path().extension() == ".jpg")
                    frames.emplace_back(e.path().string(), 0);
            }
            if (frames.empty()) continue;

            // 从 metadata.json 读取时间戳
            std::string meta_path = stream_dirs_[i] + "/metadata.json";
            std::map<std::string, int64_t> ts_map;
            std::ifstream mf(meta_path);
            if (mf.is_open()) {
                std::string line;
                while (std::getline(mf, line)) {
                    auto pos_f = line.find("\"frame\":\"");
                    auto pos_t = line.find("\"trigger_ns\":");
                    if (pos_f != std::string::npos && pos_t != std::string::npos) {
                        std::string fname = line.substr(pos_f + 9, line.find("\"", pos_f + 9) - pos_f - 9);
                        int64_t ts = std::stoll(line.substr(pos_t + 13));
                        ts_map[fname] = ts;
                    }
                }
            }

            for (auto& [path, ts] : frames) {
                std::string fname = fs::path(path).filename().string();
                auto it = ts_map.find(fname);
                if (it != ts_map.end()) ts = it->second;
            }
            std::sort(frames.begin(), frames.end(),
                      [](const auto& a, const auto& b) { return a.second < b.second; });

            // 根据真实时间戳计算实际 FPS
            double real_fps = fps_;
            if (frames.size() >= 2 && frames.front().second > 0 && frames.back().second > 0) {
                double total_sec = (frames.back().second - frames.front().second) / 1e9;
                if (total_sec > 0.01) real_fps = (frames.size() - 1) / total_sec;
            }

            cv::Mat first = cv::imread(frames[0].first);
            if (first.empty()) continue;
            bool color = (first.channels() == 3);
            std::string out = base_dir_ + "/video_" + std::to_string(static_cast<int>(i * interval_deg_)) + "deg.mp4";
            cv::VideoWriter w(out, cv::VideoWriter::fourcc('m','p','4','v'), real_fps, first.size(), color);
            for (auto& [path, ts] : frames) {
                cv::Mat f = cv::imread(path);
                if (!f.empty()) w.write(f);
            }
            w.release();
            RCLCPP_INFO(this->get_logger(), "  ✓ %s (%zu帧, %.2f fps)", out.c_str(), frames.size(), real_fps);
        }
    }

    // ─── 写出每个 stream 的 metadata.json ───
    void write_metadata() {
        for (int i = 0; i < num_streams_; ++i) {
            if (stream_metadata_[i].empty()) continue;
            std::string path = stream_dirs_[i] + "/metadata.json";
            std::ofstream of(path);
            of << "[\n";
            for (size_t j = 0; j < stream_metadata_[i].size(); ++j) {
                of << "  " << stream_metadata_[i][j];
                if (j + 1 < stream_metadata_[i].size()) of << ",";
                of << "\n";
            }
            of << "]\n";
            RCLCPP_INFO(this->get_logger(), "📋 metadata.json → %s (%zu条)", path.c_str(), stream_metadata_[i].size());
        }
    }
    // ─── 成员变量 ───
    double fps_, interval_deg_, interval_rad_;
    int num_streams_;
    std::string base_dir_;
    std::vector<std::string> stream_dirs_;
    std::vector<int> image_counts_;
    std::vector<std::vector<std::string>> stream_metadata_;  // 每条一行 JSON
    
    cv::RotateFlags rotate_code_;
    bool do_rotate_ = false;

    double accumulated_ = 0, last_yaw_ = 0, dir_ema_ = 0;
    bool first_odom_ = true;
    int expected_trig_ = 0, dbg_ = 0, laps_ = 0;
    rclcpp::Time last_odom_stamp_;  // 上一帧里程计时间戳，用于插值

    std::deque<ImageFrame> buffer_;
    std::deque<TriggerRequest> pending_;
    std::mutex pending_mtx_; // 🌟 必须有的锁：保护 pending_ 队列被双线程并发访问

    rclcpp::CallbackGroup::SharedPtr odom_cb_group_;
    rclcpp::CallbackGroup::SharedPtr image_cb_group_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    std::thread save_thread_;
    std::mutex queue_mtx_;
    std::condition_variable cv_;
    std::queue<SaveTask> save_queue_;
    bool stop_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // 🌟 恢复多核并发！真正榨干 CPU 性能
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<AngleTriggerNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}