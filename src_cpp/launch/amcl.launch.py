from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time'
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([FindPackageShare('src_cpp'), 'maps', 'my_map.yaml']),
        description='Path to map.yaml'
    )
    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([FindPackageShare('src_cpp'), 'config', 'amcl.yaml']),
        description='Path to AMCL params YAML'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file     = LaunchConfiguration('map')
    params_file  = LaunchConfiguration('params_file')

    # --- Include Nav2's localization (runs map_server + amcl + lifecycle) ---
    nav2_loc_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'localization_launch.py'
    )
    amcl_inc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_loc_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(map_arg)
    ld.add_action(params_arg)
    ld.add_action(amcl_inc)
    return ld
