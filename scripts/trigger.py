#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import os
import glob

class AngleTriggerNode(Node):
    def __init__(self):
        super().__init__('angle_trigger_node')
        
        # 1. 动态读取 FPS
        self.declare_parameter('fps', 10.0)
        self.video_fps = self.get_parameter('fps').get_parameter_value().double_value
        
        # --- 核心配置参数 ---
        self.trigger_interval_deg = 90.0 
        self.trigger_interval_rad = math.radians(self.trigger_interval_deg)
        
        # 自动创建存放目录，并【清空历史残留图片】
        self.num_streams = int(360.0 / self.trigger_interval_deg)
        self.base_dir = os.path.expanduser("~/argus_data")
        self.stream_dirs = []
        for i in range(self.num_streams):
            dir_name = os.path.join(self.base_dir, f"stream_{int(i * self.trigger_interval_deg)}")
            os.makedirs(dir_name, exist_ok=True)
            
            # ========== 新增：启动时清空该目录下以前拍的旧照片 ==========
            old_files = glob.glob(os.path.join(dir_name, "*.jpg"))
            for f in old_files:
                try:
                    os.remove(f)
                except OSError:
                    pass
            # ============================================================
            
            self.stream_dirs.append(dir_name)
            
        # 状态变量
        self.last_trigger_yaw = None
        self.current_stream_idx = 0 
        self.image_counts = [0] * self.num_streams 
        self.trigger_flag = False
        self.lap_count = 0  
        self.bridge = CvBridge()

        # --- 订阅话题 ---
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        
        # 订阅由 YAML 动态命名的新版 Topic
        self.image_sub = self.create_subscription(Image, '/driver/hikvision/argus_camera/image_raw', self.image_callback, qos)

        self.get_logger().info(f"🚀 全向视频流采集已启动！(已自动清理历史数据)")
        self.get_logger().info(f"⚙️ 当前物理转速/视频帧率设定为: {self.video_fps:.1f} FPS (r/s)")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.last_trigger_yaw is None:
            self.last_trigger_yaw = current_yaw
            return

        angle_diff = current_yaw - self.last_trigger_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff)) 

        if abs(angle_diff) >= self.trigger_interval_rad:
            self.last_trigger_yaw += math.copysign(self.trigger_interval_rad, angle_diff)
            self.trigger_flag = True

    def image_callback(self, msg):
        if not self.trigger_flag:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            target_dir = self.stream_dirs[self.current_stream_idx]
            frame_count = self.image_counts[self.current_stream_idx]
            
            img_name = os.path.join(target_dir, f"frame_{frame_count:05d}.jpg")
            cv2.imwrite(img_name, cv_image)
            
            self.get_logger().debug(f"抓拍方向 {self.current_stream_idx * 90}°: {img_name}")
            
            self.image_counts[self.current_stream_idx] += 1
            
            # ==========================================
            # 修改：每完整转完一圈，立刻播报！
            # ==========================================
            if (self.current_stream_idx + 1) % self.num_streams == 0:
                self.lap_count += 1
                self.get_logger().info(f"🔄 抓拍中... 已成功完成第 {self.lap_count} 圈全向数据采集！")
            
            self.current_stream_idx = (self.current_stream_idx + 1) % self.num_streams
            self.trigger_flag = False 
            
        except Exception as e:
            self.get_logger().error(f"图像转换或保存失败: {e}")

    def generate_all_videos(self):
        self.get_logger().info("======================================")
        self.get_logger().info(f"🏁 收到停止指令。本次有效完成 {self.lap_count} 圈。开始按 {self.video_fps:.1f} FPS 合成视频...")
        self.get_logger().info("======================================")

        for i, target_dir in enumerate(self.stream_dirs):
            angle = int(i * self.trigger_interval_deg)
            output_video_path = os.path.join(self.base_dir, f"video_{angle}deg.mp4")

            search_path = os.path.join(target_dir, "*.jpg")
            images = sorted(glob.glob(search_path))

            if not images:
                self.get_logger().warn(f"⚠️ 方向 {angle}° 没有拍到照片，跳过合成。")
                continue

            first_frame = cv2.imread(images[0])
            height, width, layers = first_frame.shape
            size = (width, height)

            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            video_writer = cv2.VideoWriter(output_video_path, fourcc, self.video_fps, size)

            self.get_logger().info(f"🎬 正在合成 {angle}° 视频 (共 {len(images)} 帧) -> {output_video_path}")

            for img_path in images:
                frame = cv2.imread(img_path)
                video_writer.write(frame)

            video_writer.release()
            
        self.get_logger().info("✅ 所有视频合成完毕！可以安全退出了。")

def main(args=None):
    rclpy.init(args=args)
    node = AngleTriggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.generate_all_videos()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()