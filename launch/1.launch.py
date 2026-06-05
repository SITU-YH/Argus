import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # =================================================================
    # 🌟 核心：获取包 (argus) 在系统中的绝对路径
    # =================================================================
    argus_share_dir = get_package_share_directory('argus')

    # 1. 启动 Livox
    # 指向你自己 argus 包里的 config 文件夹
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


   
    # 2. 启动海康相机
    hik_launch_path = os.path.join(argus_share_dir, 'launch', 'standalone.launch.yaml')
    
    camera_node = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(hik_launch_path),
        launch_arguments={'camera_name': 'argus_camera'}.items()
    )
    # 3. 启动 RViz2
    # 指向 argus 包里的 rviz 配置文件
    rviz_config_path = os.path.join(argus_share_dir, 'config', 'camera_lidar.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path], 
        output='screen'
    )

    # 4. 发布标定好的外参 TF (雷达 -> 相机)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_camera_tf',
        arguments=[
            '0.055899', '-0.514563', '0.013142',                    # 平移 X Y Z (米)
            '0.202623', '0.191962', '-0.677195', '0.680809',        # 旋转 qx qy qz qw
            'argus_camera', 'livox_frame'                           # 父坐标系 与 子坐标系
        ]
    )

    # 5. 运行内参发布 Python 脚本 
    camera_info_node = Node(
        package='argus',
        executable='camera_info.py',
        name='camera_info_publisher',
        output='screen'
    )

    # 6. 将相机节点用 TimerAction 包裹，延时 3 秒启动
    delayed_camera_node = TimerAction(
        period=3.0,
        actions=[camera_node]
    )

    return LaunchDescription([
        livox_node,
        rviz_node,
        static_tf_node,
        camera_info_node,    
        delayed_camera_node  
    ])