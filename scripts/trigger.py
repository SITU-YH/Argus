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
        
        # 自动创建存放目录，并清空历史残留图片
        self.num_streams = int(360.0 / self.trigger_interval_deg)
        self.base_dir = os.path.expanduser("~/argus_data")
        self.stream_dirs = []
        for i in range(self.num_streams):
            dir_name = os.path.join(self.base_dir, f"stream_{int(i * self.trigger_interval_deg)}")
            os.makedirs(dir_name, exist_ok=True)
            
            old_files = glob.glob(os.path.join(dir_name, "*.jpg"))
            for f in old_files:
                try:
                    os.remove(f)
                except OSError:
                    pass
            self.stream_dirs.append(dir_name)
            
        # ==========================================
        # 🌟 状态机大升级：真实物理积分与抓拍队列
        # ==========================================
        self.accumulated_yaw = 0.0         # 物理空间累计转过的绝对总角度
        self.last_odom_yaw = None          # 上一帧的原始角度
        self.expected_trigger_count = 0    # 根据真实转角，理论上应该触发的次数
        self.pending_captures = 0          # 等待相机执行的抓拍任务队列
        
        self.current_stream_idx = 0 
        self.image_counts = [0] * self.num_streams 
        self.lap_count = 0  
        self.bridge = CvBridge()

        # --- 订阅话题 ---
        self.odom_sub = self.create_subscription(Odometry, '/Odometry', self.odom_callback, 10)
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy
        qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.image_sub = self.create_subscription(Image, '/driver/hikvision/argus_camera/image_raw', self.image_callback, qos)

        self.get_logger().info(f"🚀 全向采集已启动！(已升级积分队列，消除幽灵触发)")
        self.get_logger().info(f"⚙️ 当前物理转速/视频帧率设定为: {self.video_fps:.1f} FPS (r/s)")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        current_yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.last_odom_yaw is None:
            self.last_odom_yaw = current_yaw
            return

        # 1. 计算极短时间内的微小角度增量 (完美过滤 -pi 到 pi 的突变)
        diff = current_yaw - self.last_odom_yaw
        diff = math.atan2(math.sin(diff), math.cos(diff)) 

        # ==========================================
        # 🚀 核心修复：消除掉帧导致的“时光倒流”错觉
        # 因为我们知道转台是一直往前转的（正向），
        # 如果算出来它突然“倒退”了超过 90 度，绝对是因为卡顿漏帧，导致真实旋转超过了 180 度！
        # 此时必须把它强行掰回到正向的真实增量。
        # ==========================================
        if diff < -math.pi / 2:
            diff += 2 * math.pi
        # ==========================================
        
        # 2. 积分：累加到物理总旋转角 (变成单调平滑增长)
        self.accumulated_yaw += diff
        self.last_odom_yaw = current_yaw

        # ==========================================
        # ⬇️ 请在这里插入这 4 行探针代码 ⬇️
        # ==========================================
        if not hasattr(self, 'debug_cnt'): self.debug_cnt = 0
        self.debug_cnt += 1
        if self.debug_cnt % 10 == 0:  # 约每 1 秒打印一次真实积分状态
            self.get_logger().info(f"🔍 [探针] 当前原始Yaw: {math.degrees(current_yaw):.1f}°, 真实累计已转: {math.degrees(self.accumulated_yaw):.1f}°")
        # ==========================================

        # 3. 除法：计算当前真实物理角度【应该】触发多少次
        # (例如转了 185度，185 / 90 = 2.05 -> 应该有 2 个任务)
        current_expected_triggers = int(abs(self.accumulated_yaw) / self.trigger_interval_rad)

        # 4. 如果发现跨越了新的 90° 边界，下发任务给相机！
        if current_expected_triggers > self.expected_trigger_count:
            # 严格计算新增的任务量（绝不重复触发）
            new_tasks = current_expected_triggers - self.expected_trigger_count
            self.pending_captures += new_tasks
            self.expected_trigger_count = current_expected_triggers
            
            self.get_logger().info(f"🎯 物理旋转真实跨越 {current_expected_triggers * self.trigger_interval_deg}° 边界！派发抓拍任务...")

    def image_callback(self, msg):
        # 如果队列里没有任务，直接无视图像，节省算力
        if self.pending_captures <= 0:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            target_dir = self.stream_dirs[self.current_stream_idx]
            frame_count = self.image_counts[self.current_stream_idx]
            
            img_name = os.path.join(target_dir, f"frame_{frame_count:05d}.jpg")
            cv2.imwrite(img_name, cv_image)
            
            # 把提示改为 info 级别，并且打上具体的方向，方便你核对！
            self.get_logger().info(f"📸 成功保存方向 {self.current_stream_idx * self.trigger_interval_deg}°: frame_{frame_count:05d}.jpg")
            
            self.image_counts[self.current_stream_idx] += 1
            
            # ==========================================
            # 只有严格走完 4 个方向，才会播报一圈
            # ==========================================
            if (self.current_stream_idx + 1) % self.num_streams == 0:
                self.lap_count += 1
                self.get_logger().info(f"🟢 =======================================")
                self.get_logger().info(f"🟢 完美闭环！已成功完成第 {self.lap_count} 圈全向数据采集！")
                self.get_logger().info(f"🟢 =======================================")
            
            # 切换到下一个夹子，并消耗掉 1 个任务
            self.current_stream_idx = (self.current_stream_idx + 1) % self.num_streams
            self.pending_captures -= 1 
            
        except Exception as e:
            self.get_logger().error(f"图像转换或保存失败: {e}")

    def generate_all_videos(self):
        self.get_logger().info("======================================")
        self.get_logger().info(f"🏁 收到停止指令。本次真实完成 {self.lap_count} 圈。开始按 {self.video_fps:.1f} FPS 合成独立视频流...")
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