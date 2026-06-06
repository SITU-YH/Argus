#!/bin/bash

# --- 1. 定义清理函数 ---
cleanup() {
    echo -e "\n检测到关闭信号，正在停止所有 Argus 进程..."
    # 杀死所有包含 argus 关键词的子进程
    pkill -SIGINT -f "argus"
    # 等待几秒确保清理干净
    sleep 1
    echo "清理完毕，正在退出。"
    exit 0
}

# --- 2. 捕获 Ctrl+C 和 脚本退出信号 ---
trap cleanup SIGINT SIGTERM

# --- 3. 启动各个模块 ---
echo "正在启动 Argus 系统，该终端将作为‘总控台’..."
WS_PATH=~/argus_ws

# 驱动层
gnome-terminal --title="Drivers" -- bash -c "cd $WS_PATH; source install/setup.bash; ros2 launch argus drivers.launch.py; exec bash"
# SLAM层
gnome-terminal --title="FAST-LIO" -- bash -c "cd $WS_PATH; source install/setup.bash; ros2 launch argus mapping.launch.py; exec bash"
# 触发器
gnome-terminal --title="Trigger" -- bash -c "cd $WS_PATH; source install/setup.bash; python3 src/argus/scripts/trigger.py; exec bash"

# --- 4. 关键点：让主窗口驻留 ---
echo "=========================================="
echo "系统已启动。"
echo "按下 [Ctrl+C] 或回车键即可一键关闭所有节点。"
echo "=========================================="

# 等待用户输入，或者一直挂起
read -p "输入任意键停止..." 
cleanup