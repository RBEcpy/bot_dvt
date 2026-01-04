import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    package_name = 'src_cpp'
    package_dir = get_package_share_directory(package_name)

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map = LaunchConfiguration('map')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated clock if true'
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value='', # set map here
        description='Full path to map yaml file for AMCL'
    )

    # Nav2 params
    nav2_params_file = os.path.join(
        package_dir,
        'config',
        'nav2_params.yaml'
    )

    # --------- AMCL (Localization dùng map có sẵn) ---------
    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'localization_launch.py'
            )
        ]),
        launch_arguments={
            'map': map,
            'use_sim_time': use_sim_time
        }.items()
    )

    # --------- NAV2 NAVIGATION (bí não Nav2) ---------
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ]),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # --------- BUILD LAUNCH DESCRIPTION ---------
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_map)

    ld.add_action(amcl)
    ld.add_action(navigation)

    return ld
