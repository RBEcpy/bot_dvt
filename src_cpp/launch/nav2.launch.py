#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    package_name = 'src_cpp'
    pkg_dir = get_package_share_directory(package_name)

    # ---- Launch args ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    use_amcl     = LaunchConfiguration('use_amcl')
    map_yaml     = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulated clock (Gazebo)'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
        description='Path to Nav2 params YAML'
    )

    declare_use_amcl = DeclareLaunchArgument(
        'use_amcl', default_value='true',
        description='true: run AMCL (needs map). false: assume SLAM/localization runs elsewhere.'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'my_map.yaml'),
        description='Path to a prebuilt map.yaml (used when use_amcl=true)'
    )

    # ---- (tuỳ chọn) Localization bằng AMCL khi bạn KHÔNG chạy localization trong SLAM ----
    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'localization_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml
        }.items(),
        condition=IfCondition(use_amcl)
    )

    # ---- Nav2 Navigation (planner, controller, costmaps, behaviors, etc.) ----
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    # ---- Relay: Nav2 /cmd_vel  →  ros2_control /diff_drive_controller/cmd_vel ----
    # Vì bạn không dùng joy/twist_mux, dùng relay cho gọn.
    relay_cmd = Node(
        package='topic_tools',
        executable='relay',
        name='relay_cmd_vel_to_controller',
        arguments=['/cmd_vel', '/diff_drive_controller/cmd_vel'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_use_amcl)
    ld.add_action(declare_map)
    ld.add_action(amcl)        # chạy nếu use_amcl:=true
    ld.add_action(navigation)  # luôn chạy Nav2
    ld.add_action(relay_cmd)   # nối tốc độ tới controller

    return ld
