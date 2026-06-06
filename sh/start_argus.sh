#!/bin/bash
# 自动杀死已有 argus 进程
trap "pkill -SIGINT -f 'argus'; exit" SIGINT

echo "正在启动 Argus 系统..."

# 启动三个独立终端
gnome-terminal --tab --title="Argus-Drivers" -- bash -c "source install/setup.bash; ros2 launch argus drivers.launch.py; exec bash"
gnome-terminal --tab --title="Argus-SLAM" -- bash -c "source install/setup.bash; ros2 launch argus slam.launch.py; exec bash"
gnome-terminal --tab --title="Argus-Tools" -- bash -c "source install/setup.bash; ros2 launch argus tools.launch.py; exec bash"

echo "启动完成。在当前终端按 [Ctrl+C] 可关闭所有 Argus 相关节点。"
read -p ""