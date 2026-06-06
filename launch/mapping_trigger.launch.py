import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 获取你自己 argus 包的绝对路径
    argus_share_dir = get_package_share_directory('argus')

    # ==========================================================
    # 1. 启动 Livox (完美接管：自定义配置 + FAST-LIO 专属格式)
    # ==========================================================
    livox_config_path = os.path.join(argus_share_dir, 'config', 'MID360_config.json')
    
    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,             # <--- 核心魔法：1 代表输出 CustomMsg 供 FAST-LIO 建图
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'cmdline_str': 'MID360',
            'user_config_path': livox_config_path, # <--- 完美指向你自己的配置文件！
            'frame_id': 'livox_frame'
        }]
    )

    # ==========================================================
    # 2. 启动海康相机 (调用你自己 launch 文件夹里的配置)
    # ==========================================================
    hik_launch_path = os.path.join(argus_share_dir, 'launch', 'hik_camera.launch.yaml')
    
    camera_node = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(hik_launch_path),
        launch_arguments={'camera_name': 'argus_camera'}.items()
    )

 
    # ==========================================================
    # 3. 启动 FAST-LIO (彻底接管：读取 argus/config 里的建图参数)
    # ==========================================================
    fast_lio_config_path = os.path.join(argus_share_dir, 'config', 'fast_lio_mid360.yaml')
    
    # 还可以顺便获取 Rviz 配置，如果你想连 FAST-LIO 的 Rviz 视角也接管的话
    # rviz_cfg_path = os.path.join(argus_share_dir, 'config', 'rviz_mapping.rviz')

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',    # FAST-LIO 的核心可执行节点名
        name='fastlio_mapping',
        output='screen',
        parameters=[fast_lio_config_path] # <--- 完美：强行喂给它你自己的参数配置！
    )
    # ==========================================================
    # 4. 启动你的 30° 角度触发拍照节点 
    # ==========================================================
    trigger_node = Node(
        package='argus',            
        executable='trigger.py',    
        name='angle_trigger_node',
        output='screen'
    )

  # ==========================================================
    # 5. 启动 RViz2 (修复了文件名拼写)
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
    # 6. 发布标定好的外参 TF (雷达 -> 相机) 
    # ==========================================================
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_camera_tf',
        arguments=[
            '0.055899', '-0.514563', '0.013142',                    # 平移 X Y Z 
            '0.202623', '0.191962', '-0.677195', '0.680809',        # 旋转 qx qy qz qw
            'argus_camera', 'livox_frame'                           # 父子坐标系
        ]
    )

    # ==========================================================
    # 7. 运行内参发布脚本 (解决 Camera 黑屏的核心！)
    # ==========================================================
    camera_info_node = Node(
        package='argus',
        executable='camera_info.py',
        name='camera_info_publisher',
        output='screen'
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
        rviz_node,
        camera_info_node         
    ])