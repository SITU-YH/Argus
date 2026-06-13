#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

        this->declare_parameter("rotate_code", 2);
        int code = this->get_parameter("rotate_code").as_int();
        if (code >= 0 && code <= 2) {
            rotate_code_ = static_cast<cv::RotateFlags>(code);
            do_rotate_ = true;
        } else {
            do_rotate_ = false; 
        }

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

        save_thread_ = std::thread(&AngleTriggerNode::save_loop, this);

        imu_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        image_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        rclcpp::SubscriptionOptions imu_options;
        imu_options.callback_group = imu_cb_group_;
        rclcpp::SubscriptionOptions image_options;
        image_options.callback_group = image_cb_group_;

        // 🌟 终极杀招：抛弃 Odometry，直接高频订阅 200Hz 的底层 IMU！
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/livox/imu", rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Imu::ConstSharedPtr m) { imu_cb(m); }, imu_options);

        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/driver/hikvision/argus_camera/image_raw", qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr m) { image_cb(m); }, image_options);

        RCLCPP_INFO(this->get_logger(), "🚀 全向采集(纯IMU降维版)启动 | 间隔=%.1f° | 流数=%d", interval_deg_, num_streams_);
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
    // ─── 纯陀螺仪高频积分回调 ───
// ─── 纯陀螺仪高频积分回调 (带零偏过滤 & 主轴自适应) ───
    void imu_cb(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        rclcpp::Time curr_stamp = msg->header.stamp;

        if (first_imu_) {
            last_imu_stamp_ = curr_stamp;
            first_imu_ = false;
            return;
        }

        double dt = (curr_stamp - last_imu_stamp_).seconds();
        if (dt <= 0.0 || dt > 0.1) {
            last_imu_stamp_ = curr_stamp;
            return;
        }

        // 1. 获取三个轴的原始角速度
        double wx = msg->angular_velocity.x;
        double wy = msg->angular_velocity.y;
        double wz = msg->angular_velocity.z;

        // 2. 🌟 零偏死区滤波 (Deadband Filter)
        // 屏蔽掉低于 0.05 rad/s (约 2.8°/s) 的静态环境底噪，彻底解决静止时的角度漂移！
        if (std::abs(wx) < 0.05) wx = 0.0;
        if (std::abs(wy) < 0.05) wy = 0.0;
        if (std::abs(wz) < 0.05) wz = 0.0;

        // 3. 🌟 自动寻找主旋转轴 (自适应雷达安装方向)
        // 既然是高速转台，只要一开机，真正的那个旋转轴的角速度绝对是最大的！
        double main_w = 0.0;
        if (std::abs(wx) >= std::abs(wy) && std::abs(wx) >= std::abs(wz)) {
            main_w = wx;
        } else if (std::abs(wy) >= std::abs(wx) && std::abs(wy) >= std::abs(wz)) {
            main_w = wy;
        } else {
            main_w = wz;
        }

        // 如果三个轴的转速都没超过死区，说明转台处于静止，直接退出，绝不产生假积分！
        if (main_w == 0.0) {
            last_imu_stamp_ = curr_stamp;
            return; 
        }

        // 4. 纯粹的数学积分
        double acc_before = accumulated_;
        accumulated_ += main_w * dt;
        last_imu_stamp_ = curr_stamp;

        int new_trig = static_cast<int>(std::abs(accumulated_) / interval_rad_);
        for (int k = expected_trig_ + 1; k <= new_trig; ++k) {
            int stream = k % num_streams_;

            double target_abs = k * interval_rad_;
            double acc_before_abs = std::abs(acc_before);
            double acc_after_abs  = std::abs(accumulated_);
            rclcpp::Time interp_stamp = msg->header.stamp; 

            if (acc_after_abs > acc_before_abs) {
                double frac = (target_abs - acc_before_abs) / (acc_after_abs - acc_before_abs);
                if (frac >= 0.0 && frac <= 1.0) {
                    interp_stamp = curr_stamp - rclcpp::Duration::from_seconds((1.0 - frac) * dt);
                }
            }

            TriggerRequest req;
            req.trigger_time = interp_stamp;
            req.yaw_deg = k * interval_deg_;
            req.stream_idx = stream;

            {
                std::lock_guard<std::mutex> lock(pending_mtx_);
                pending_.push_back(req);
            }

            RCLCPP_INFO(this->get_logger(), "🎯 触发 %.0f° → stream_%d | 插值追溯=%.1f%%",
                       req.yaw_deg, stream, (acc_after_abs > acc_before_abs) ? 
                       100.0 * (target_abs - acc_before_abs) / (acc_after_abs - acc_before_abs) : 100.0);
        }
        expected_trig_ = new_trig;

        // 🌟 打印三个轴的真实转速，让你一眼看穿物理世界！
        if (++dbg_ % 200 == 0) {
            RCLCPP_INFO(this->get_logger(), "🔍 IMU积分: 累计=%.0f° | 真实转速(XYZ)=[%.2f, %.2f, %.2f] rad/s", 
                        accumulated_ * RAD_TO_DEG, msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        }
    }
    // ─── 图像回调 ───
    void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        ImageFrame f{msg, msg->header.stamp};
        buffer_.push_back(f);
        auto now = this->now();
        while (!buffer_.empty() && (now - buffer_.front().stamp).seconds() > 2.0)
            buffer_.pop_front();
        while (buffer_.size() > 60) buffer_.pop_front(); // 放宽一点缓冲池

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

                RCLCPP_INFO(this->get_logger(), "📸 [%.0f°] 时光倒流匹配 (Δt=%.0fms)", req.yaw_deg, best_d * 1000);

                if ((req.stream_idx + 1) % num_streams_ == 0) {
                    laps_++;
                    RCLCPP_INFO(this->get_logger(), "🟢 第%d圈完美闭环!", laps_);
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
                RCLCPP_WARN(this->get_logger(), "⚠️ 弱匹配兜底 [%.0f°] Δt=%.0fms", req.yaw_deg, best_d * 1000);
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
                cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(t.img_msg, t.img_msg->encoding);
                
                cv::Mat img_to_save;
                if (do_rotate_) {
                    cv::rotate(cv_ptr->image, img_to_save, rotate_code_);
                } else {
                    img_to_save = cv_ptr->image.clone();
                }

                if (t.img_msg->encoding == "rgb8") {
                    cv::cvtColor(img_to_save, img_to_save, cv::COLOR_RGB2BGR);
                }
                
                cv::imwrite(t.img_path, img_to_save);

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
            std::vector<std::pair<std::string, int64_t>> frames;
            for (const auto& e : fs::directory_iterator(stream_dirs_[i])) {
                if (e.path().extension() == ".jpg")
                    frames.emplace_back(e.path().string(), 0);
            }
            if (frames.empty()) continue;

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

    double fps_, interval_deg_, interval_rad_;
    int num_streams_;
    std::string base_dir_;
    std::vector<std::string> stream_dirs_;
    std::vector<int> image_counts_;
    std::vector<std::vector<std::string>> stream_metadata_;
    
    cv::RotateFlags rotate_code_;
    bool do_rotate_ = false;

    // 🌟 清理了大量无用的跳变检测变量，只留下纯粹的数学积分
    double accumulated_ = 0;
    bool first_imu_ = true;
    rclcpp::Time last_imu_stamp_;
    
    int expected_trig_ = 0, dbg_ = 0, laps_ = 0;

    std::deque<ImageFrame> buffer_;
    std::deque<TriggerRequest> pending_;
    std::mutex pending_mtx_; 

    rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
    rclcpp::CallbackGroup::SharedPtr image_cb_group_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_; // 🌟 改为订阅 IMU
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    std::thread save_thread_;
    std::mutex queue_mtx_;
    std::condition_variable cv_;
    std::queue<SaveTask> save_queue_;
    bool stop_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<AngleTriggerNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}