from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    argus_share_dir = get_package_share_directory('argus')
    fast_lio_config = os.path.join(argus_share_dir, 'config', 'fast_lio_mid360.yaml')

    fast_lio_node = Node(
        package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
        output='screen', parameters=[fast_lio_config]
    )

    return LaunchDescription([
        TimerAction(period=2.0, actions=[fast_lio_node]),
        Node(package='argus', executable='camera_info.py', name='camera_info_publisher', output='screen')
    ])