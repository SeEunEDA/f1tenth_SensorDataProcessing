from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('f1tenth_ftg')
    cfg = os.path.join(pkg, 'config', 'ftg.yaml')

    return LaunchDescription([
        Node(
            package='f1tenth_ftg',
            executable='ftg_node',
            name='ftg_node',
            parameters=[cfg],
            output='screen',
        ),
    ])
