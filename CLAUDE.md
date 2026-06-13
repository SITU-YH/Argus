# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Multi-Sensor Deblurring** — 高速旋转平台上，利用 LiDAR + IMU + RGB 相机进行运动去模糊。

- **传感器**: Livox MID360（深度+里程计）、IMU（角速度/姿态）、海康工业RGB相机
- **技术路线**: 传统几何模型（Model-based）或深度学习（Learning-based）去模糊
- **采集需求**: 在特定旋转角度精准触发相机曝光，生成多方向清晰视频流
- **本仓库 (Argus)**: 全向视觉采集子系统，负责角度触发拍照 + 多 stream 存盘 + MP4 合成

### 技术栈

| 层 | 技术 |
|----|------|
| 语言 | C++17 / Python 3.10 |
| 框架 | ROS2 Humble |
| 深度学习 | PyTorch（去模糊算法侧，非本包） |
| 视觉 | OpenCV ≥ 4.x |
| 建图 | FAST-LIO（激光惯性里程计） |

---

## Argus — 全向采集子系统

ROS2 Humble 包，单节点 `angle_trigger_node`，订阅里程计跟踪偏航角，按角度间隔触发海康相机拍照。

### Build & Run

```bash
# 编译（在 ROS2 workspace 根目录，通常 ~/rmd_ws）
colcon build --packages-select argus
source install/setup.bash

# Lint
colcon test --packages-select argus

# 基础模式（LiDAR + 相机 + RViz，不触发拍照）
ros2 launch argus 1.launch.py

# 建图采集模式（LiDAR + 相机 + FAST-LIO + 角度触发 + RViz）
ros2 launch argus mapping_trigger.launch.py
```

编译系统: `ament_cmake`，需要 C++17（`std::filesystem`）。

### Data Flow

```
/Odometry (nav_msgs/Odometry) ──┐
                                  ├── angle_trigger_node ──► ~/argus_data/stream_*/
/driver/hikvision/argus_camera/   │
    image_raw (sensor_msgs/Image)─┘
```

外部依赖链:
- `livox_ros_driver2` → `/livox/lidar` + `/livox/imu`
- `FAST-LIO` → 消费 LiDAR + IMU，产出 `/Odometry`
- `hikvision_driver` → `/driver/hikvision/argus_camera/image_raw`

### angle_trigger_node 内部设计

全部逻辑在 [src/angle_trigger_node.cpp](src/angle_trigger_node.cpp)（~440行），`MultiThreadedExecutor` 驱动。

**线程模型**:
- `odom_cb_group_` + `image_cb_group_`：两个 MutuallyExclusive 回调组，里程计和图像回调在不同线程并发执行
- `save_thread_`：后台存图线程，旋转 + 写盘不阻塞主线程

**里程计回调 (`odom_cb`)**:
1. 四元数 → 偏航角，EMA 方向跟踪处理 ±π 翻转
2. 跳变检测（>3 rad 跳过，>2.5 rad 尝试修复）
3. 累计角度 `accumulated_` 跨越 `k × interval_deg` 时，**线性插值**估算精确触发时刻
4. 插值后的 `TriggerRequest` 入队 `pending_`

**图像回调 (`image_cb`)**:
1. 环形缓冲（最多 30 帧 / 2 秒窗口）
2. 消费 `pending_`：按时间戳最近匹配（<300ms 精确匹配，<1s 弱匹配兜底）
3. 匹配到的帧入队 `save_queue_`，通知存图线程

**后台存图 (`save_loop`)**:
- `cv::rotate` 在此线程执行（不阻塞回调）
- RGB→BGR 转换后 `cv::imwrite` 写 JPG
- 元数据按 stream 收集到内存，退出时统一写 `metadata.json`

**析构流程**:
1. 停止存图线程
2. `write_metadata()` — 每个 stream 写一条 JSON 数组
3. `generate_videos()` — 按时间戳排序帧，自动计算实际 FPS，`cv::VideoWriter` 合成 MP4（120s 超时保护）

**线程安全**:
| 资源 | 保护机制 |
|------|----------|
| `pending_` | `pending_mtx_`（里程计/图像双回调访问） |
| `save_queue_` | `queue_mtx_` + `condition_variable` |
| `stream_metadata_` | 仅存图线程写，析构时存图线程已停，无竞争 |

### 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `trigger_interval_deg` | 90.0 | 触发角度间隔，stream 数 = ceil(360/间隔) |
| `fps` | 10.0 | 视频合成参考帧率（会被真实时间戳覆盖） |
| `rotate_code` | 2 | OpenCV RotateFlags: 0=CW90°, 1=180°, 2=CCW90° |

### TF 树

```
livox_frame ──(static_transform)──> argus_camera_optical_frame
```

外参硬编码在两个 launch 文件中（x/y/z + qx/qy/qz/qw）。

### 输出结构

```
~/argus_data/
├── stream_0/           # 各角度图片 + metadata.json
├── stream_90/
├── stream_180/
├── stream_270/
├── video_0deg.mp4      # 按真实时间戳合成的视频
├── video_90deg.mp4
├── video_180deg.mp4
└── video_270deg.mp4
```

### 配置文件

| 文件 | 用途 |
|------|------|
| `config/MID360_config.json` | Livox 网络 IP/端口 |
| `config/camera_params.yaml` | 海康相机名、pixel_format、曝光、增益 |
| `config/camera_info.yaml` | 相机内参矩阵 K/D/R/P（plumb_bob 畸变模型） |
| `config/fast_lio_mid360.yaml` | FAST-LIO 建图参数 |
| `config/camera_lidar.rviz` | RViz 布局（相机+雷达视图） |
| `config/fastlio.rviz` | RViz 布局（建图视图） |
