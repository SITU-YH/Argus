import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 获取 argus 包的绝对路径
    argus_share_dir = get_package_share_directory('argus')

    # ==========================================================
    # 1. 启动 Livox
    # ==========================================================
    livox_config_path = os.path.join(argus_share_dir, 'config', 'MID360_config.json')
    
    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'cmdline_str': 'MID360',
            'user_config_path': livox_config_path,
            'frame_id': 'livox_frame'
        }]
    )

    # ==========================================================
    # 2. 启动海康相机 (配置驱动模式)
    # ==========================================================
    hik_driver_share = get_package_share_directory('hikvision_driver')
    hik_launch_path = os.path.join(hik_driver_share, 'launch', 'hik_camera.launch.py')
    
    # 核心修改：指向 argus 包自己的 config 目录
    # 以后你想用不同的相机配置，只需在这个文件夹里放不同的 yaml 即可
    my_camera_yaml = os.path.join(argus_share_dir, 'config', 'camera_params.yaml')
    
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hik_launch_path),
        # 不再传递 camera_name，直接传配置文件路径
        # 驱动会自动读取 yaml 里的 camera_name 作为身份
        launch_arguments={'config_file': my_camera_yaml}.items()
    )

    # ==========================================================
    # 3. 启动 FAST-LIO
    # ==========================================================
    fast_lio_config_path = os.path.join(argus_share_dir, 'config', 'fast_lio_mid360.yaml')
    
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[fast_lio_config_path]
    )

    # 注意：图像旋转已内迁至 angle_trigger_node，不再需要独立的 image_rotate_node

    # ==========================================================
    # 4. 启动触发拍照节点 (内含旋转功能)
    # ==========================================================
    trigger_node = Node(
        package='argus',
        executable='angle_trigger_node',
        name='angle_trigger_node',
        output='screen',
        parameters=[{
            'fps': 10.0,
            'trigger_interval_deg': 90.0,
            'rotate_code': 2  # <--- 把旋转指令直接传给 trigger 节点！
        }]
    )

    # ==========================================================
    # 7. 启动 RViz2
    # ==========================================================
    rviz_config_path = os.path.join(argus_share_dir, 'config', 'fastlio.rviz') 
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # ==========================================================
    # 8. 发布标定好的外参 TF (雷达 -> 相机) 
    # ==========================================================
    # 请确保 frame_id 和这里的一致
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_camera_tf',
        arguments=[
            '0.055899', '-0.514563', '0.013142',
            '0.202623', '0.191962', '-0.677195', '0.680809',
            'livox_frame', 'argus_camera_optical_frame'  # 父：livox_frame, 子：argus_camera
        ]
    )

    # ==========================================================
    # 🌟 系统统筹：延时启动策略
    # ==========================================================
    delayed_fast_lio = TimerAction(
        period=2.0,
        actions=[fast_lio_node]
    )
    
    delayed_trigger = TimerAction(
        period=4.0,
        actions=[trigger_node]
    )

    return LaunchDescription([
        livox_node,
        camera_node,
        static_tf_node,
        delayed_fast_lio,
        delayed_trigger,
        rviz_node
        # ⚠️ 注意：camera_info_node 已被移除
    ])