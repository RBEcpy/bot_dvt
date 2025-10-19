# lidar_filtered.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg = 'src_cpp'  # đổi nếu package của bạn tên khác
    share = get_package_share_directory(pkg)

    use_sim_time = LaunchConfiguration('use_sim_time')
    input_scan  = LaunchConfiguration('input_scan')
    output_scan = LaunchConfiguration('output_scan')
    yaml_path   = LaunchConfiguration('yaml_path')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use /clock'
    )
    declare_input_scan = DeclareLaunchArgument(
        'input_scan', default_value='/scan', description='Input LaserScan'
    )
    declare_output_scan = DeclareLaunchArgument(
        'output_scan', default_value='/scan_filtered', description='Output LaserScan'
    )
    declare_yaml_path = DeclareLaunchArgument(
        'yaml_path',
        default_value=os.path.join(share, 'config', 'lidar_filters.yaml'),
        description='Params file (ROS 2 node schema)'
    )

    node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',                    
        parameters=[yaml_path],                
        remappings=[('scan', input_scan),
                    ('scan_filtered', output_scan)],
        output='screen'
    )

    ld = LaunchDescription()
    for a in (declare_use_sim_time, declare_input_scan, declare_output_scan, declare_yaml_path):
        ld.add_action(a)
    ld.add_action(node)
    return ld
