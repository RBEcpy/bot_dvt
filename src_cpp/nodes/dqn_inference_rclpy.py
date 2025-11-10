#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DQN inference node (ROS 2 Jazzy, rclpy)

Giữ nguyên ý chí code cũ:
- Load mô hình DQN (.pth) đã huấn luyện
- Subscribe: /scan, /odom, (/goal_pose mặc định cho Nav2; có thể dùng /move_base_simple/goal)
- Xây state 28 chiều (24 tia lidar lấy step stride + heading + distance + min_range + min_angle)
- Chọn action greedy (epsilon=0), ánh xạ sang (weight_obstacle, weight_optimaltime)
- Cập nhật TEB controller trong Nav2 qua ROS 2 parameter service
- Publish thông tin hành động ra topic /get_action (std_msgs/Float32MultiArray)

Những điểm nâng/cải tiến nhỏ để ổn định:
- Thêm QoS phù hợp cho /scan (SensorData-like)
- Kiểm tra tồn tại file .pth, log cảnh báo và chạy với trọng số ngẫu nhiên nếu thiếu
- Thêm timeout khi set parameters để tránh treo node
- Cho phép cấu hình topic goal, namespace plugin TEB và tên node controller_server
"""

import os
import math
import numpy as np
import torch
import torch.nn as nn

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType


# ---------------- Utils ----------------
def quaternion_to_yaw(x, y, z, w) -> float:
    """Tính yaw từ quaternion (ROS 2 không mặc định có tf_transformations)."""
    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def sanitize_state(state, expected_size=None):
    """Chuyển state thành np.float32 1D, thay NaN/Inf, pad/crop nếu cần."""
    if isinstance(state, (tuple, list)) and len(state) >= 1 and not isinstance(state[0], (int, float, np.floating)):
        state = state[0]

    if isinstance(state, (list, tuple)):
        a = np.array(state, dtype=np.float32)
    elif isinstance(state, np.ndarray):
        a = state.astype(np.float32).reshape(-1)
    else:
        a = np.array([state], dtype=np.float32)

    a = np.nan_to_num(a, nan=0.0, posinf=3.5, neginf=-3.5)

    if expected_size is not None:
        if a.shape[0] > expected_size:
            a = a[:expected_size]
        elif a.shape[0] < expected_size:
            pad = np.zeros(expected_size - a.shape[0], dtype=np.float32)
            a = np.concatenate([a, pad])
    return a


# ---------------- Model (giữ nguyên cấu trúc) ----------------
class DQN(nn.Module):
    def __init__(self, state_size, action_size):
        super().__init__()
        self.fc1 = nn.Linear(state_size, 128)
        self.fc2 = nn.Linear(128, 128)
        self.fc3 = nn.Linear(128, action_size)

        nn.init.kaiming_uniform_(self.fc1.weight, nonlinearity='relu')
        nn.init.kaiming_uniform_(self.fc2.weight, nonlinearity='relu')
        nn.init.xavier_uniform_(self.fc3.weight)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        return self.fc3(x)


# ---------------- ROS2 Node ----------------
class DQNInferenceNode(Node):
    def __init__(self):
        super().__init__('dqn_inference_rclpy')

        # ------- Declare parameters (đặt default hợp lý) -------
        self.declare_parameter('stage', '4')  # giữ cùng semantics
        self.declare_parameter('model_dir', '')  # #HASHTAG:MODEL_DIR (đường dẫn thư mục chứa .pth)
        self.declare_parameter('model_file', 'latest.pth')  # #HASHTAG:MODEL_FILE
        self.declare_parameter('state_size', 28)
        self.declare_parameter('action_size', 500)
        self.declare_parameter('lidar_stride', 15)
        self.declare_parameter('min_range_threshold', 0.13)
        self.declare_parameter('rate', 5.0)

        # Nav2 / TEB params
        self.declare_parameter('controller_server_node', '/controller_server')  # #HASHTAG:CONTROLLER_SERVER_NODE
        self.declare_parameter('teb_plugin_ns', 'FollowPath')  # #HASHTAG:TEB_PLUGIN_NS (tên plugin trong nav2_params.yaml)
        self.declare_parameter('goal_topic', '/goal_pose')  # Nav2 RViz default; có thể đổi thành /move_base_simple/goal

        # ------- Read parameters -------
        self.stage = self.get_parameter('stage').get_parameter_value().string_value
        self.model_dir_override = self.get_parameter('model_dir').get_parameter_value().string_value
        self.model_filename = self.get_parameter('model_file').get_parameter_value().string_value
        self.state_size = int(self.get_parameter('state_size').get_parameter_value().integer_value or 28)
        self.action_size = int(self.get_parameter('action_size').get_parameter_value().integer_value or 500)
        self.lidar_sample_stride = int(self.get_parameter('lidar_stride').get_parameter_value().integer_value or 15)
        self.min_range_threshold = float(self.get_parameter('min_range_threshold').value)
        self.rate_hz = float(self.get_parameter('rate').value)
        self.controller_server_node = self.get_parameter('controller_server_node').value
        self.teb_plugin_ns = self.get_parameter('teb_plugin_ns').value
        self.goal_topic = self.get_parameter('goal_topic').value

        # ------- Resolve model path -------
        if self.model_dir_override:
            self.model_dir = self.model_dir_override
        else:
            here = os.path.dirname(os.path.realpath(__file__))
            # Giữ logic cũ: thay 'nodes' -> 'save_model/stage_<stage>_'
            candidate = here.replace('nodes', f'save_model/stage_{self.stage}_')
            self.model_dir = candidate
        self.model_path = os.path.join(self.model_dir, self.model_filename)
        self.get_logger().info(f"[DQN_INF] Looking for model at: {self.model_path}")

        # ------- Device + model -------
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = DQN(self.state_size, self.action_size).to(self.device)
        self._load_model(self.model_path)

        # ------- Action mapping như cũ -------
        self.w_obstacle_list = list(range(5, 105, 5))   # 5..100, step 5 (len 20)
        self.w_optimaltime_list = list(range(1, 51, 2)) # 1..49, step 2 (len 25)
        total_actions = len(self.w_obstacle_list) * len(self.w_optimaltime_list)
        if self.action_size > total_actions:
            self.get_logger().warn(
                f"[DQN_INF] action_size={self.action_size} lớn hơn bảng mapping ({total_actions}). "
                f"Sẽ clamp theo bảng mapping."
            )

        # ------- QoS cho /scan (ổn định hơn) -------
        qos_scan = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ------- Subscribers / Publishers -------
        self.scan = None
        self.odom = None
        self.goal = None

        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_scan)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # Cho phép dùng topic goal của Nav2 (RViz2) hoặc topic cũ nếu người dùng đổi param
        self.create_subscription(PoseStamped, self.goal_topic, self._goal_cb, 10)

        self.pub_get_action = self.create_publisher(Float32MultiArray, 'get_action', 5)

        # ------- Service client to set parameters on controller_server -------
        # (Sẽ tạo client on-demand để tránh lỗi nếu node kia chưa có ngay)
        self.setparam_client = None

        # ------- Loop timer -------
        period = 1.0 / max(0.1, self.rate_hz)
        self.timer = self.create_timer(period, self._loop)

        self.get_logger().info("[DQN_INF] Inference node started. Waiting for /scan, /odom and goal topic...")

    # -------------- Model loader --------------
    def _load_model(self, path: str):
        if os.path.isfile(path):
            try:
                state_dict = torch.load(path, map_location=self.device)
                self.model.load_state_dict(state_dict)
                self.model.eval()
                self.get_logger().info(f"[DQN_INF] Loaded model from {path}")
            except Exception as e:
                self.get_logger().warn(f"[DQN_INF] Failed to load model: {e}")
        else:
            self.get_logger().warn(f"[DQN_INF] Model file not found: {path}. Running with random weights (not recommended).")

    # -------------- Callbacks --------------
    def _scan_cb(self, msg: LaserScan):
        self.scan = msg

    def _odom_cb(self, msg: Odometry):
        self.odom = msg

    def _goal_cb(self, msg: PoseStamped):
        self.goal = msg

    # -------------- State builder --------------
    def _build_state(self):
        """
        Trả về vector state 28 phần tử:
        - Lidar sampled theo stride
        - heading = góc giữa yaw robot và góc tới goal (wrap -pi..pi)
        - current_distance = khoảng cách robot->goal
        - obstacle_min_range, obstacle_angle
        """
        if self.scan is None or self.odom is None or self.goal is None:
            return None

        scan = self.scan
        ranges = np.array(scan.ranges, dtype=np.float32)

        # sample mỗi self.lidar_sample_stride
        sampled = []
        stride = max(1, int(self.lidar_sample_stride))
        for i in range(0, len(ranges), stride):
            v = float(ranges[i])
            if math.isinf(v):
                sampled.append(3.5)
            elif math.isnan(v):
                sampled.append(0.0)
            else:
                sampled.append(v)
        sampled = np.array(sampled, dtype=np.float32)

        # angles cho các tia sampled
        angles_all = np.linspace(scan.angle_min, scan.angle_max, len(ranges), dtype=np.float32)
        angles = angles_all[::stride]
        if len(angles) != len(sampled):
            angles = np.linspace(scan.angle_min, scan.angle_max, len(sampled), dtype=np.float32)

        # yaw robot
        q = self.odom.pose.pose.orientation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # goal & robot position
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y

        goal_angle = math.atan2(gy - py, gx - px)
        heading = goal_angle - yaw
        # wrap to [-pi, pi]
        if heading > math.pi:
            heading -= 2.0 * math.pi
        elif heading < -math.pi:
            heading += 2.0 * math.pi
        heading = float(round(heading, 2))

        if sampled.size > 0:
            obstacle_min_range = float(np.min(sampled))
            obstacle_angle = float(angles[int(np.argmin(sampled))])
        else:
            obstacle_min_range = 0.0
            obstacle_angle = 0.0

        current_distance = float(round(math.hypot(gx - px, gy - py), 2))

        # Lidar part + 4 extras = state_size
        lidar_needed = max(0, self.state_size - 4)
        lidar_part = sanitize_state(sampled.tolist(), expected_size=lidar_needed)

        state = np.concatenate([
            lidar_part,
            np.array([heading, current_distance, round(obstacle_min_range, 2), obstacle_angle], dtype=np.float32)
        ])
        return state

    # -------------- Action selection & TEB update --------------
    def _select_and_apply_action(self, state):
        if state is None:
            return None, None

        s = sanitize_state(state, expected_size=self.state_size)
        s_tensor = torch.from_numpy(s).unsqueeze(0).float().to(self.device)
        with torch.no_grad():
            q_values = self.model(s_tensor).cpu().numpy()[0]

        action_idx = int(np.argmax(q_values))
        q_max = float(np.max(q_values))

        # Map sang (w_obstacle, w_optimaltime)
        w_obs_len = len(self.w_obstacle_list)
        w_opt_len = len(self.w_optimaltime_list)

        obs_idx = max(0, min(w_obs_len - 1, action_idx // w_opt_len))
        opt_idx = max(0, min(w_opt_len - 1, action_idx % w_opt_len))

        w_obstacle = float(self.w_obstacle_list[obs_idx])
        w_optimaltime = float(self.w_optimaltime_list[opt_idx])

        # Áp vào Nav2 TEB qua set_parameters
        self._set_teb_params(w_obstacle, w_optimaltime)

        # Publish debug/info
        arr = Float32MultiArray()
        arr.data = [float(action_idx), float(q_max), float(w_obstacle), float(w_optimaltime)]
        self.pub_get_action.publish(arr)

        return action_idx, (w_obstacle, w_optimaltime)

    def _set_teb_params(self, w_obs: float, w_opt: float):
        """Gọi service set_parameters trên controller_server để set params runtime."""
        node_name = self.controller_server_node.rstrip('/')  # e.g. /controller_server
        full_service = f"{node_name}/set_parameters"

        if self.setparam_client is None:
            self.setparam_client = self.create_client(SetParameters, full_service)

        if not self.setparam_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn(f"[DQN_INF] Service not available: {full_service}")
            return

        # Tham số của plugin TEB có dạng: <teb_plugin_ns>.weight_obstacle
        # Ví dụ nếu teb_plugin_ns = "FollowPath", param là "FollowPath.weight_obstacle"
        def make_param(name: str, value: float) -> Parameter:
            p = Parameter()
            p.name = f"{self.teb_plugin_ns}.{name}"
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
            return p

        req = SetParameters.Request()
        req.parameters = [
            make_param('weight_obstacle', w_obs),
            make_param('weight_optimaltime', w_opt),
        ]

        future = self.setparam_client.call_async(req)
        # Chờ ngắn để biết kết quả, tránh treo vòng lặp
        rclpy.task.Future
        done = self._spin_until_future_complete(future, timeout=0.2)
        if not done:
            self.get_logger().warn("[DQN_INF] set_parameters timeout (non-fatal).")

    def _spin_until_future_complete(self, future, timeout=0.2) -> bool:
        """Quay event loop có kiểm soát để chờ future hoàn thành, trả về True/False."""
        start = self.get_clock().now()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.01)
            if (self.get_clock().now() - start).nanoseconds * 1e-9 > timeout:
                return False
        return True

    # -------------- Loop --------------
    def _loop(self):
        state = self._build_state()
        if state is not None:
            action_idx, params = self._select_and_apply_action(state)
            if action_idx is not None:
                # giảm spam log: 1 dòng mỗi ~5s tương đương ROS1 log_throttle
                # Ở đây chỉ log thưa (mỗi N vòng), đơn giản:
                if int(self.get_clock().now().nanoseconds / 1e9) % 5 == 0:
                    self.get_logger().info(f"[DQN_INF] action={action_idx} params={params}")


def main():
    rclpy.init()
    node = DQNInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
