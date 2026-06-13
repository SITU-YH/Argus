# Argus — 全向视觉采集系统

基于 ROS2 Humble 的全向（多角度）图像采集系统，配合 Livox MID360 激光雷达与海康工业相机，按偏航角触发拍照，用于高速旋转平台上的运动去模糊。

## 硬件

| 设备 | 型号 | 用途 |
|------|------|------|
| LiDAR | Livox MID360 | IMU 角速度 + 点云建图 |
| 相机 | 海康 MV-CB016-10GC-S-W | 图像采集 |
| 计算平台 | x86_64 (Linux) | ROS2 运行 |

## 架构

```
/livox/imu (200Hz) ──────────┐
                              ├── angle_trigger_node ── 纯积分触发
/driver/hikvision/            │    ├─ 固定主轴积分 (rotation_axis)
    argus_camera/image_raw ───┘    ├─ 反推插值 + 最近帧匹配
                                   ├─ BayerRG8 无损存 PNG
                                   ├─ 起停自动检测
                                   └─ 析构时合成 MP4
```

- `angle_trigger_node`：核心节点（单文件 ~500 行），订阅 Livox 原始 IMU 陀螺仪数据，对指定轴角速度做纯数学积分，在到达预设角度间隔时触发图像采集
- 原始 Bayer 数据无损保存为 PNG，省 3 倍带宽
- 图像旋转在后台存图线程中完成（非 Bayer 模式），不阻塞主线程
- 转台停转后自动收尾：写元数据 → 合成视频 → 退出

## 目录结构

```
Argus/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/
│   └── angle_trigger_node.cpp   # 核心节点
├── launch/
│   ├── 1.launch.py              # 基础启动：LiDAR + 相机 + RViz + TF
│   └── mapping_trigger.launch.py # 完整启动：基础 + FAST-LIO + 角度触发
├── config/
│   ├── MID360_config.json       # Livox MID360 网络配置
│   ├── camera_params.yaml       # 海康相机参数
│   ├── camera_info.yaml         # 相机内参标定
│   ├── fast_lio_mid360.yaml     # FAST-LIO 参数配置
│   ├── camera_lidar.rviz        # RViz 配置（相机+雷达视图）
│   └── fastlio.rviz             # RViz 配置（建图视图）
```

## 依赖

### ROS2 包
- `rclcpp`
- `sensor_msgs`（含 `image_encodings`）
- `cv_bridge`
- `tf2_ros`
- `rviz2`

### 外部驱动
- [`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2) — Livox LiDAR 驱动
- [`hikvision_driver_ros2`](https://github.com/SITU-YH/hikvision_driver_ros2) — 海康相机驱动
- [`FAST-LIO`](https://github.com/hku-mars/FAST_LIO) — 激光惯性里程计（可选，仅建图需要）

### 系统
- OpenCV ≥ 4.x
- C++17
- ROS2 Humble

## 编译

```bash
cd ~/rmd_ws
colcon build --packages-select argus
source install/setup.bash
```

## 启动

### 基础模式（LiDAR + 相机 + RViz 预览，不触发拍照）

```bash
ros2 launch argus 1.launch.py
```

启动内容：
- Livox MID360 驱动
- 海康相机驱动（延时 3 秒）
- TF 外参发布（LiDAR → 相机坐标系）
- RViz2（相机+雷达视图）

### 建图采集模式（全功能）

```bash
ros2 launch argus mapping_trigger.launch.py
```

启动内容：
- Livox MID360 驱动
- 海康相机驱动
- FAST-LIO 建图（延时 2 秒）
- **角度触发节点** — 按偏航角自动拍照（延时 4 秒）
- TF 外参发布
- RViz2（建图视图）

## 配置

### 触发参数

在 `mapping_trigger.launch.py` 的 `trigger_node` 中配置：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `fps` | 10.0 | 视频合成参考帧率（会被真实时间戳覆盖） |
| `trigger_interval_deg` | 90.0 | 触发角度间隔（度），stream 数 = ceil(360/间隔) |
| `rotation_axis` | 2 | 主旋转轴：0=X, 1=Y, 2=Z（根据雷达安装方向设定） |
| `rotate_code` | 2 | OpenCV 旋转码：0=顺时针90°, 1=180°, 2=逆时针90° |
| `auto_stop_timeout` | 3.0 | 转台停转多少秒后自动关闭节点，0=禁用 |

### 相机参数

编辑 `config/camera_params.yaml`：

```yaml
camera_name: "argus_camera"
frame_id: "argus_camera_optical_frame"
pixel_format: "BayerRG8"     # 原始 Bayer，省 3 倍带宽
exposure_time: 10000.0       # 曝光时间 (μs)
gain: 8.0                    # 增益 (dB)
```

支持的像素格式：`"Keep"`, `"RGB8"`, `"BGR8"`, `"Mono8"`, `"BayerRG8"`, `"BayerBG8"`, `"BayerGR8"`, `"BayerGB8"`。

### LiDAR 外参

在 launch 文件的 `static_tf_node` 中修改 LiDAR 到相机的外参（x, y, z, qx, qy, qz, qw）。

## 输出

采集的图像和数据保存在 `~/argus_data/`：

```
~/argus_data/
├── stream_360/     # 360° (物理 0°) 方向
│   ├── frame_00000.png          # Bayer → PNG 无损
│   ├── frame_00001.png
│   ├── ...
│   └── metadata.json
├── stream_90/      # 90° 方向
├── stream_180/     # 180° 方向
├── stream_270/     # 270° 方向
├── video_360deg.mp4
├── video_90deg.mp4
├── video_180deg.mp4
└── video_270deg.mp4
```

每个 stream 目录下有一个 `metadata.json` 记录全部帧信息：

```json
[
  {"frame":"frame_00000.png","yaw_deg":360.0,"trigger_ns":1718200000000000000,"encoding":"bayer_rggb8"},
  {"frame":"frame_00001.png","yaw_deg":720.0,"trigger_ns":1718200012000000000,"encoding":"bayer_rggb8"}
]
```

`encoding` 字段仅在 Bayer 模式时出现，用于指示下游 debayer 算法应该使用何种 Bayer 排列。

## 工作原理

### 角度触发与帧匹配

1. IMU 回调取指定轴角速度（`rotation_axis`），纯数学积分跟踪 `accumulated_`（累计旋转角度），无跳变风险
2. 当累计角度越过 `k × interval_deg` 阈值时，用**反推插值**估算设备实际到达该角度的精确时刻
3. 图像回调在环形缓冲（60 帧 / 2 秒窗口）中按时间戳查找最接近该时刻的图像帧
4. 匹配策略：Δt < 300ms 为强匹配；Δt > 300ms 且请求超过 1 秒为弱匹配兜底
5. 匹配到的帧放入后台存图队列，异步写入磁盘

### 存图

- Bayer 模式：跳过旋转和色彩转换，直接存 PNGLossless】
- RGB 模式：支持旋转 + RGB→BGR 转换，存 JPG

### 起停自动检测

节点启动后等待首次检测到转动（`has_rotated_`），之后 1Hz 定时器持续监测。当旋转轴转速低于死区（0.05 rad/s）超过 `auto_stop_timeout` 秒时，自动触发节点关闭 → 析构 → 写 metadata → 合成视频。

### 视频合成

节点退出时，每个 stream 按真实时间戳排序后合成 MP4。Bayer 帧在合成时自动 debayer 为彩色。播放帧率根据实际采集时间跨度自动计算，保证回放速度与采集速度一致。

## 坐标系

```
livox_frame ──(TF外参)──> argus_camera_optical_frame
```

## License

Apache-2.0
