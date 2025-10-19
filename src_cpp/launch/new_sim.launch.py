# launch_sim.launch.py  (tối giản + đủ để nạp plugin vật cản động)
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction

def generate_launch_description():
    pkg = 'src_cpp'
    share = get_package_share_directory(pkg)

    # ---- Launch args ----
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulated clock (Gazebo /clock)'
    )

    # ---- Paths ----
    xacro_file = os.path.join(share, 'description', 'robot.urdf.xacro')
    world_sdf  = os.path.join(share, 'worlds', 'playground.sdf')  # giữ như file gốc
    controllers_yaml = os.path.join(share, 'config', 'controller_gz_sim.yaml')

    # ---- ENV cho resource & plugin (để nạp model + .so) ----
    # lấy prefix cài đặt của 2 package
    cpp_prefix    = get_package_prefix('src_cpp')
    plugin_prefix = get_package_prefix('src_setup')

    set_res_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        f"{cpp_prefix}/share:{os.path.join(share,'models')}:{os.path.join(share,'worlds')}"
    )
    set_sys_plugins = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        f"{plugin_prefix}/lib"
    )

    # ---- robot_description from xacro ----
    robot_description_cmd = Command(['xacro ', xacro_file])
    params = {
        'robot_description': ParameterValue(robot_description_cmd, value_type=str),
        'use_sim_time': use_sim_time
    }

    node_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params],
        output='screen'
    )

    # ---- Start Gazebo with world (auto-run, verbose=4) ----
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_sdf}'
        }.items()
    )

    # ---- Spawn robot from /robot_description ----
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'minibot',
            '-allow_renaming', 'true',
            '-z', '0.1'
        ],
    )

    # ---- Spawners (controller_manager) ----
    spawner_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml
        ],
        output='screen'
    )

    spawner_diff = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--param-file', controllers_yaml
        ],
        output='screen'
    )
    delayed_spawners = TimerAction(period=2.5, actions=[spawner_jsb, spawner_diff])

    # ---- Bridges tối thiểu ----
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(set_res_path)       # <<< thêm từ obstacle_world
    ld.add_action(set_sys_plugins)    # <<< thêm từ obstacle_world
    ld.add_action(node_rsp)
    ld.add_action(gz)
    ld.add_action(spawn)
    ld.add_action(bridge)
    ld.add_action(delayed_spawners)

    return ld
