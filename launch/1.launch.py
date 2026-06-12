import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取 argus 包的绝对路径
    argus_share_dir = get_package_share_directory('argus')
    hik_driver_share = get_package_share_directory('hikvision_driver')

    # 1. 启动 Livox
    livox_config_path = os.path.join(argus_share_dir, 'config', 'MID360_config.json')
    
    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'cmdline_str': 'MID360',
            'user_config_path': livox_config_path, 
            'frame_id': 'livox_frame'
        }]
    )

    # 2. 启动海康相机 (使用 Python 驱动的新接口)
    hik_launch_path = os.path.join(hik_driver_share, 'launch', 'hik_camera.launch.py')
    my_camera_yaml = os.path.join(argus_share_dir, 'config', 'camera_params.yaml')
    
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hik_launch_path),
        launch_arguments={'config_file': my_camera_yaml}.items()
    )

    # 3. 启动 RViz2
    rviz_config_path = os.path.join(argus_share_dir, 'config', 'camera_lidar.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path], 
        output='screen'
    )

    # 4. 发布标定好的外参 TF (注意：如果 image 还是看不到，请检查这里的 parent/child 顺序)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_camera_tf',
        arguments=[
            '0.055899', '-0.514563', '0.013142',                    
            '0.202623', '0.191962', '-0.677195', '0.680809',        
            'livox_frame', 'argus_camera_optical_frame' # 父坐标系: livox_frame, 子坐标系: argus_camera_optical_frame
        ]
    )

    # 5. 相机延时启动策略 (保持你原来的设计)
    delayed_camera_node = TimerAction(
        period=3.0,
        actions=[camera_node]
    )

    # 注意：图像旋转已内迁至 angle_trigger_node，不再需要独立的 image_rotate_node

    return LaunchDescription([
        livox_node,
        rviz_node,
        static_tf_node,
        delayed_camera_node
    ])