#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import math
import os

class AngleTriggerNode(Node):
    def __init__(self):
        super().__init__('angle_trigger_node')
        
        # --- 配置参数 ---
        # 目标触发间隔（比如每转过 30度 拍一张）
        self.trigger_interval_deg = 180.0 
        self.trigger_interval_rad = math.radians(self.trigger_interval_deg)
        
        # 图片保存路径 (存在当前终端目录下)
        self.save_dir = "./trigger_images1"
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            
        # 状态变量
        self.last_trigger_yaw = None
        self.image_count = 0
        self.trigger_flag = False
        self.bridge = CvBridge()

        # --- 订阅话题 ---
        # 1. 订阅 FAST-LIO 的里程计话题 (请根据实际情况修改话题名)
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/Odometry',  
            self.odom_callback, 
            10)
            
        # 2. 订阅海康相机的最新图像
        # 注意：这里可能需要加上之前提到的 qos_profile (Best Effort) 才能收到图像
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.image_sub = self.create_subscription(
            Image, 
            '/driver/hikvision/argus_camera/raw/image', 
            self.image_callback, 
            qos)

        self.get_logger().info(f"角度触发节点已启动！触发间隔: {self.trigger_interval_deg}°")

    def odom_callback(self, msg):
        # 1. 提取四元数
        q = msg.pose.pose.orientation
        
        # 2. 将四元数转换为 Euler 角 (Yaw)
        # 公式推导：Yaw 对应围绕 Z 轴的旋转
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # 初始化第一次的角度
        if self.last_trigger_yaw is None:
            self.last_trigger_yaw = current_yaw
            return

        # 3. 计算角度差 (巧妙处理 -PI 到 PI 的跨界问题)
        # 比如从 179度 转到 -179度，绝对差是 358，但实际只转了 2度
        angle_diff = current_yaw - self.last_trigger_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff)) 

        # 4. 判断是否达到触发条件
        if abs(angle_diff) >= self.trigger_interval_rad:
            self.get_logger().info(f"检测到旋转超过 {self.trigger_interval_deg}°！发送抓拍指令...")
            self.trigger_flag = True  # 升起触发旗帜
            self.last_trigger_yaw = current_yaw # 更新基准角度

    def image_callback(self, msg):
        # 如果没有触发需求，直接丢弃图像，节省算力
        if not self.trigger_flag:
            return
            
        # 一旦检测到触发旗帜，立刻保存当前这帧图像
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            img_name = os.path.join(self.save_dir, f"capture_{self.image_count:04d}.jpg")
            cv2.imwrite(img_name, cv_image)
            
            self.get_logger().info(f"✅ 成功抓拍并保存: {img_name}")
            self.image_count += 1
            self.trigger_flag = False  # 放下触发旗帜，等待下一次旋转达标
            
        except Exception as e:
            self.get_logger().error(f"图像转换或保存失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = AngleTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()