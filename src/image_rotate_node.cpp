#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageRotateNode : public rclcpp::Node {
public:
    ImageRotateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("image_rotate_node", options) {

        // OpenCV RotateFlags: 0=CW_90, 1=ROTATE_180, 2=CCW_90
        this->declare_parameter("rotate_code", 2); // 默认逆时针90°摆正(相机顺时针偏转90°)
        int code = this->get_parameter("rotate_code").as_int();
        if (code < 0 || code > 2) code = 2;
        rotate_code_ = static_cast<cv::RotateFlags>(code);

        const char* names[] = {"顺时针90°", "旋转180°", "逆时针90°"};
        RCLCPP_INFO(this->get_logger(), "🔄 图像旋转节点已启动，旋转方向: %s", names[code]);

        // 订阅原始图像
        rclcpp::QoS qos = rclcpp::SensorDataQoS();
        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/driver/hikvision/argus_camera/image_raw", qos,
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) { callback(msg); });

        // 发布旋转后的图像
        pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/driver/hikvision/argus_camera/image_rotated", qos);

        RCLCPP_INFO(this->get_logger(), "📡 监听: /driver/hikvision/argus_camera/image_raw");
        RCLCPP_INFO(this->get_logger(), "📡 发布: /driver/hikvision/argus_camera/image_rotated");
    }

private:
    void callback(sensor_msgs::msg::Image::ConstSharedPtr msg) {
        try {
            // 🌟 零拷贝读取，维持原始编码
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
            cv::Mat rotated;
            cv::rotate(cv_ptr->image, rotated, rotate_code_);

            // 构造输出消息
            auto out = std::make_unique<sensor_msgs::msg::Image>();
            out->header = msg->header;
            // frame_id 保持不变，因为旋转不改变相机坐标系
            out->height = rotated.rows;
            out->width = rotated.cols;
            out->encoding = msg->encoding;
            out->is_bigendian = msg->is_bigendian;
            out->step = rotated.cols * rotated.elemSize();
            out->data.assign(rotated.datastart, rotated.dataend);

            pub_->publish(std::move(out));
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "图像旋转/转换失败: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    cv::RotateFlags rotate_code_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageRotateNode>());
    rclcpp::shutdown();
    return 0;
}
