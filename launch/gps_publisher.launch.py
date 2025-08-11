from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the config file
    config = os.path.join(
        get_package_share_directory('navsatfix_pub'),
        'config',
        'gps_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='navsatfix_pub',
            executable='navsatfix_publisher',
            name='gps_publisher',
            parameters=[config],
            output='screen'
        )
    ])