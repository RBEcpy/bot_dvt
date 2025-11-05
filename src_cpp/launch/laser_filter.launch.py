# file: launch/laser_filter.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_name = 'src_cpp'  # ← đổi thành tên package của bạn
    config_path = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'laser_filter.yaml'
    )

    scan_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        output='screen',
        parameters=[config_path],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')
        ]
    )

    return LaunchDescription([scan_filter_node])
