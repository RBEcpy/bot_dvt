import os
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import TimerAction
from launch.event_handlers import OnProcessStart
from launch.actions import IncludeLaunchDescription, GroupAction, RegisterEventHandler, DeclareLaunchArgument


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
    world_sdf  = os.path.join(share, 'worlds', 'map_base.sdf')
    controllers_yaml = os.path.join(share, 'config', 'controller_gz_sim.yaml')
    bridge_yaml = os.path.join(share, 'config', 'bridge_topics.yaml')
    twist_mux_params_file = os.path.join(share, 'config', 'twist_mux.yaml')

    cpp_prefix    = get_package_prefix('src_cpp')
    plugin_prefix = get_package_prefix('src_setup')
   
        #    <-- objectcle -->
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

    node_twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name='twist_mux',
        output='screen',
        parameters=[
            twist_mux_params_file,
            {'use_stamped': True},
        ],
        remappings=[
            ('cmd_vel_out', 
             'diff_drive_controller/cmd_vel'),
        ],
    )

    node_twist_stamper = Node(
        package="twist_stamper",
        executable="twist_stamper",
        name='twist_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', 'cmd_vel_smoothed'),
            ('cmd_vel_out', 'nav_vel'),
        ],
    )


    register_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action= spawn,
            on_start= [spawner_jsb],
        )
    )

    register_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action= spawner_jsb,
            on_start= [spawner_diff],
        )
    )

    # ---- Bridges ----
    def load_bridge_topics(path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f) or {}
        topics = data.get('topics', [])
        if not isinstance(topics, list):
            raise ValueError('`topics` trong bridge_topics.yaml phải là list các chuỗi')
        return [str(t).strip() for t in topics if t]

    bridge_topics = load_bridge_topics(bridge_yaml)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=bridge_topics
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(set_res_path)       # <<< thêm từ obstacle_world
    ld.add_action(set_sys_plugins)    # <<< thêm từ obstacle_world
    ld.add_action(node_rsp)
    ld.add_action(gz)
    ld.add_action(node_twist_mux)
    ld.add_action(node_twist_stamper)
    ld.add_action(spawn)
    ld.add_action(bridge)
    ld.add_action(register_joint_state_broadcaster_spawner)
    ld.add_action(register_diff_drive_controller_spawner)

    return ld
