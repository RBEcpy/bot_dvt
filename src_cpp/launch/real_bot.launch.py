import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command


def generate_launch_description():

    package_name = 'src_cpp'
    package_dir = get_package_share_directory(package_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='If true, use simulated clock'
    )

    declare_use_ros2_control = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='If true, use ros2_control'
    )

    declare_lidar_serial_port = DeclareLaunchArgument(
        'lidar_serial_port',
        default_value='/dev/ttyUSB0',
        description='Specifying usb port to connected lidar'
    )

    # Đường dẫn các file cần thiết
    robot_description_xacro_file = os.path.join(
        package_dir,
        'description',
        'robot.urdf.xacro'
    )

    robot_controllers_file_dir = os.path.join(
        package_dir,
        'config',
        'controller.yaml'
    )

    twist_mux_params_file = os.path.join(
        package_dir,
        'config',
        'twist_mux.yaml'
    )

    # robot_state_publisher setup
    robot_description_config = Command([
        'xacro ',
        robot_description_xacro_file,
        ' use_ros2_control:=',
        use_ros2_control
    ])

    params = {
        'robot_description': ParameterValue(robot_description_config, value_type=str),
        'use_sim_time': use_sim_time,
        'use_ros2_control': use_ros2_control
    }

    # --------- ROBOT STATE PUBLISHER ---------

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # --------- ROS2 CONTROL + CONTROLLERS ---------

    node_ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            params,
            robot_controllers_file_dir
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--param-file",
            robot_controllers_file_dir
        ],
    )

    # --------- TWIST MUX + TWIST STAMPER ---------

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
            ('cmd_vel_out', 'diff_drive_controller/cmd_vel'),
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

    # --------- EVENT HANDLERS (thứ tự start node) ---------

    register_node_ros2_control = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_robot_state_publisher,
            on_start=[node_ros2_control],
        )
    )

    register_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_ros2_control,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    register_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[diff_drive_controller_spawner],
        )
    )

    # --------- LIDAR DRIVER ---------

    node_rplidar_drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('sllidar_ros2'),
                'launch',
                'sllidar_a1_launch.py'
            )
        ]),
        launch_arguments={
            'serial_port': lidar_serial_port,
            'frame_id': 'lidar_frame'
        }.items()
    )

    # --------- BUILD LAUNCH DESCRIPTION ---------

    ld = LaunchDescription()

    # launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_ros2_control)
    ld.add_action(declare_lidar_serial_port)

    # event handlers
    ld.add_action(register_node_ros2_control)
    ld.add_action(register_joint_state_broadcaster_spawner)
    ld.add_action(register_diff_drive_controller_spawner)

    # nodes
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_twist_mux)
    ld.add_action(node_twist_stamper)
    ld.add_action(node_rplidar_drive)

    return ld
