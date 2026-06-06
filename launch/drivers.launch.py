from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    argus_share_dir = get_package_share_directory('argus')
    livox_config_path = os.path.join(argus_share_dir, 'config', 'MID360_config.json')
    hik_launch_path = os.path.join(argus_share_dir, 'launch', 'hik_camera.launch.yaml')

    return LaunchDescription([
        Node(
            package='livox_ros_driver2', executable='livox_ros_driver2_node', name='livox_lidar_publisher',
            output='screen',
            parameters=[{'xfer_format': 1, 'multi_topic': 0, 'data_src': 0, 'publish_freq': 10.0, 
                         'output_data_type': 0, 'cmdline_str': 'MID360', 'user_config_path': livox_config_path, 'frame_id': 'livox_frame'}]
        ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(hik_launch_path),
            launch_arguments={'camera_name': 'argus_camera'}.items()
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher', name='lidar_camera_tf',
            arguments=['0.055899', '-0.514563', '0.013142', '0.202623', '0.191962', '-0.677195', '0.680809', 'argus_camera', 'livox_frame']
        )
    ])