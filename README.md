Lidar:
https://github.com/Livox-SDK/livox_ros_driver2

LAN1

Camera:
https://github.com/sjtu-cyberc3/hikvision_ros2_driver

LAN2



ros2 launch hikvision_ros2_driver standalone.launch.yaml camera_name:=argus_camera
// 调相机参数 f= 60cm
ros2 run rqt_reconfigure rqt_reconfigure

///home/styh/argus_ws/src/hikvision_ros2_driver/hikvision_ros2_driver/launch/component.launch.yaml 永久调整
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
