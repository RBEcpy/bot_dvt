#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Packages & paths
    bringup_share  = get_package_share_directory('src_cpp')   # worlds/models
    bringup_prefix = get_package_prefix('src_cpp')
    plugin_prefix  = get_package_prefix('src_setup')          # .so

    worlds_dir = os.path.join(bringup_share, 'worlds')
    models_dir = os.path.join(bringup_share, 'models')

    # Args
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(worlds_dir, 'dynamic_world.sdf'),
        description='Path to SDF world file'
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose', default_value='2',
        description='Gazebo verbosity level (0..4)'
    )
    world   = LaunchConfiguration('world')
    verbose = LaunchConfiguration('verbose')

    # Env (resources & system plugins)
    set_res = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        TextSubstitution(text=f"{bringup_prefix}/share:{models_dir}:{worlds_dir}")
    )
    set_sys_plugins = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        TextSubstitution(text=f"{plugin_prefix}/lib")
    )

    # Include ros_gz_sim launcher; pass gz_args as a list of substitutions (no .perform)
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                TextSubstitution(text='-r -v '),  # auto-run + verbosity
                verbose,
                TextSubstitution(text=' '),
                world
            ]
        }.items()
    )

    return LaunchDescription([
        world_arg, verbose_arg,
        set_res, set_sys_plugins,
        gz,
    ])
