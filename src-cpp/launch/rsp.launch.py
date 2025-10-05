from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = 'src-cpp'                      # đổi nếu package khác
    pkg_share = get_package_share_directory(pkg)

    # --- Args ---
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulated time'
    )

    # --- Paths (giữ đúng như trong sim.launch.py) ---
    urdf_xacro = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')

    # --- Expand xacro -> XML, rồi ép kiểu str để tránh ROS parse YAML ---
    robot_description_xml = Command([FindExecutable(name='xacro'),' ', urdf_xacro])
    robot_description = ParameterValue(robot_description_xml, value_type=str)

    # --- Nodes ---
    jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        jsp_gui,
        rsp,
        rviz
    ])
