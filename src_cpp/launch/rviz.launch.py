from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([rviz_node])
