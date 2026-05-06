#!/bin/bash

# 1. 加载 ROS 2 环境和小 PC 的工作空间
source /opt/ros/humble/setup.bash
source ~/argus_ws/install/setup.bash

# 2. 设置分布式通信环境变量 (确保与笔记本一致)
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0

echo "--------------------------------------------"
echo "Argus Sensor System Starting..."
echo "DOMAIN_ID: $ROS_DOMAIN_ID"
echo "--------------------------------------------"

# 3. 启动海康相机驱动 (后台运行)
# 使用 & 将进程放入后台，这样脚本可以继续往下跑
ros2 launch hikvision_ros2_driver standalone.launch.yaml camera_name:=argus_camera &
CAM_PID=$!
echo "✓ Hikvision Camera node started (PID: $CAM_PID)"

# 稍微等待 2 秒，避免启动瞬间 CPU 占用过高
sleep 2

# 4. 启动 Livox 雷达驱动 (后台运行)
# 这里使用的是你仓库里的 Mid-360 启动脚本
ros2 launch livox_ros_driver2 msg_MID360_launch.py &
LIDAR_PID=$!
echo "✓ Livox LiDAR node started (PID: $LIDAR_PID)"

echo "--------------------------------------------"
echo "All sensors are running!"
echo "Press [Ctrl+C] to stop all sensors."
echo "--------------------------------------------"

# 5. 捕获中断信号 (Ctrl+C)，确保退出时关闭所有后台进程
trap "echo 'Stopping sensors...'; kill $CAM_PID $LIDAR_PID; exit" SIGINT SIGTERM

# 保持脚本运行，直到你按下 Ctrl+C
wait