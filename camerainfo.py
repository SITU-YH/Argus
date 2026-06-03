import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher_hack')
        # 订阅真实的图像话题，为了获取完美同步的时间戳
        self.sub = self.create_subscription(
            Image, 
            '/driver/hikvision/argus_camera/raw/image', 
            self.image_callback, 
            10)
        
        # 创建内参发布者
        self.pub = self.create_publisher(
            CameraInfo, 
            '/driver/hikvision/argus_camera/raw/camera_info', 
            10)

        # 填入你标定出的全部真实数据
        self.info_msg = CameraInfo()
        self.info_msg.header.frame_id = 'argus_camera'
        self.info_msg.height = 1080
        self.info_msg.width = 1440
        self.info_msg.distortion_model = 'plumb_bob'
        
        # D: 畸变参数 (d0, d1, d2, d3, 0)
        self.info_msg.d = [-0.2309, 0.0318, 0.0, 0.0, 0.0]
        # K: 3x3 内参矩阵 (fx, 0, cx, 0, fy, cy, 0, 0, 1)
        self.info_msg.k = [560.777, 0.0, 747.417, 0.0, 524.511, 525.162, 0.0, 0.0, 1.0]
        # R: 3x3 旋转矩阵 (单位阵)
        self.info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        # P: 3x4 投影矩阵 (fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0)
        self.info_msg.p = [560.777, 0.0, 747.417, 0.0, 0.0, 524.511, 525.162, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.get_logger().info("内参发布节点已启动！正在等待相机图像以同步时间戳...")

    def image_callback(self, msg):
        # 核心：每次收到图像，就把图像的时间戳抄过来，发布内参！
        self.info_msg.header.stamp = msg.header.stamp
        self.pub.publish(self.info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()