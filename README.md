# Argus — 全向视觉采集系统

基于 ROS2 Humble 的全向（多角度）图像采集系统，配合 Livox MID360 激光雷达与海康工业相机，按偏航角触发拍照，用于环境感知与三维建图。

## 硬件

| 设备 | 型号 | 用途 |
|------|------|------|
| LiDAR | Livox MID360 | 里程计 + 点云建图 |
| 相机 | 海康工业相机 | 图像采集 |
| 计算平台 | x86_64 (Linux) | ROS2 运行 |

## 架构

```
IMU (/livox/imu) ──┐
                    ├── angle_trigger_node ── 按角度触发 ── 多 stream 存盘
图像 (/driver/.../  ──┘                                └── 退出时合成 MP4
      argus_camera/image_raw)
```

- `angle_trigger_node`：核心节点，订阅 Livox IMU 纯陀螺仪 Z 轴角速度积分，在到达预设角度间隔时（默认 90°）触发图像采集
- 每个角度对应一个独立 stream，图片保存至不同子目录
- 图像旋转在后台存图线程中完成，不阻塞主线程
- 节点退出时自动将每个 stream 的图片合成为 MP4 视频

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
- `sensor_msgs`
- `cv_bridge`
- `tf2_ros`
- `rviz2`

### 外部驱动
- [`livox_ros_driver2`](https://github.com/Livox-SDK/livox_ros_driver2) — Livox LiDAR 驱动
- [`hikvision_driver`](https://github.com/STYH-3524/hikvision_driver) — 海康相机驱动
- [`FAST-LIO`](https://github.com/hku-mars/FAST_LIO) — 激光惯性里程计

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

### 基础模式（LiDAR + 相机 + RViz 预览）

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
| `fps` | 10.0 | 相机帧率（用于视频合成） |
| `trigger_interval_deg` | 90.0 | 触发角度间隔（度） |
| `rotate_code` | 2 | OpenCV 旋转码：0=顺时针90°, 1=180°, 2=逆时针90° |

### 相机参数

编辑 `config/camera_params.yaml`：

```yaml
camera_name: "argus_camera"
frame_id: "argus_camera_optical_frame"
pixel_format: "RGB8"
exposure_time: 10000.0   # 曝光时间 (μs)
gain: 8.0                # 增益 (dB)
```

### LiDAR 外参

在 launch 文件的 `static_tf_node` 中修改 LiDAR 到相机的外参（x, y, z, qx, qy, qz, qw）。

## 输出

采集的图像和数据保存在 `~/argus_data/`：

```
~/argus_data/
├── stream_0/      # 0°  方向采集的图片
│   ├── frame_00000.jpg
│   ├── frame_00001.jpg
│   ├── ...
│   └── metadata.json   # 该 stream 全部帧的元数据
├── stream_90/     # 90° 方向采集的图片 + metadata.json
├── stream_180/    # 180°方向采集的图片 + metadata.json
├── stream_270/    # 270°方向采集的图片 + metadata.json
├── video_0deg.mp4
├── video_90deg.mp4
├── video_180deg.mp4
└── video_270deg.mp4
```

每个 stream 目录下有一个 `metadata.json` 记录全部帧信息：

```json
[
  {"frame":"frame_00000.jpg","yaw_deg":360.0,"trigger_ns":1718200000000000000},
  {"frame":"frame_00001.jpg","yaw_deg":720.0,"trigger_ns":1718200012000000000}
]
```

## 工作原理

### 角度触发与帧匹配

1. IMU 回调取 Z 轴角速度，纯数学积分跟踪 `accumulated_`（累计旋转角度），无跳变风险
2. 当累计角度越过 `k × interval_deg` 阈值时，用**反推插值**估算设备实际到达该角度的精确时刻
3. 图像回调在环形缓冲中按时间戳查找最接近该时刻的图像帧
4. 匹配到的帧放入后台存图队列，异步写入磁盘

### 视频合成

节点退出时，每个 stream 按真实时间戳排序后合成 MP4，播放帧率根据实际采集时间跨度自动计算，保证回放速度与采集速度一致。

## 坐标系

```
livox_frame ──(TF外参)──> argus_camera_optical_frame
```

## License

Apache-2.0
