from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = "src_cpp"  # đổi nếu gói của bạn tên khác
    share = get_package_share_directory(pkg)

    model_dir = os.path.join(share, 'save_model', 'stage_4_')
    model_file = 'latest.pth'

    return LaunchDescription([
        Node(
            package=pkg,
            executable='dqn_inference',  # hoặc 'dqn_inference' nếu bạn RENAME trong CMake
            name='dqn_inference',
            output='screen',
            parameters=[{
                # model
                'model_dir': model_dir,
                'model_file': model_file,

                # topics (nghe cả 2 kiểu goal nếu bạn đã áp dụng patch nghe 2 topic)
                'scan_topic': '/scan',
                'odom_topic': '/odom',
                'goal_topic': '/goal_pose',                    # Nav2 Goal
                'goal_topic_secondary': '/move_base_simple/goal',  # 2D Nav Goal (ROS1 style)

                # Nav2 controller_server + plugin namespace
                'controller_server_node': '/controller_server',
                'teb_plugin_ns': 'FollowPath',                 # khớp YAML ở trên

                # Map 2 “trọng số” của DQN vào 2 critic weights
                'teb_param_1': 'CostCritic.cost_weight',
                'teb_param_2': 'PathAlignCritic.cost_weight',

                # tốc độ vòng lặp, state…
                'rate': 5.0,
                'state_size': 28,
                'action_size': 500,
                'lidar_stride': 15
            }]
        )
    ])
