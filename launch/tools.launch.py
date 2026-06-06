from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    argus_share_dir = get_package_share_directory('argus')
    rviz_config = os.path.join(argus_share_dir, 'config', 'fastlio.rviz')

    trigger_node = Node(package='argus', executable='trigger.py', name='angle_trigger_node', output='screen')

    return LaunchDescription([
        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config], output='screen'),
        TimerAction(period=4.0, actions=[trigger_node])
    ])