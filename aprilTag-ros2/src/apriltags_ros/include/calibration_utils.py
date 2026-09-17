"""手眼标定公共工具：参数读取、位姿<->矩阵、矩阵求逆、采集数据存取。

约定：T_A2B 表示把 B 系下的点变换到 A 系，p_A = T_A2B @ p_B。
"""
import json
import os

import numpy as np
import yaml

PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RECORD_DIR = os.path.join(PACKAGE_DIR, 'calibration_record')
T_CAM2TARGET_FILE = os.path.join(RECORD_DIR, 'matrix_cam2target.json')
T_BASE2END_FILE = os.path.join(RECORD_DIR, 'matrix_base2end.json')


def get_params(yaml_name='camare_params.yaml'):
    """包外 src/ 优先，其次 config/、包目录；可用 APRILTAGS_ROS_PARAMS 覆盖。"""
    env_path = os.environ.get('APRILTAGS_ROS_PARAMS')
    candidates = [env_path] if env_path else []
    candidates += [os.path.normpath(os.path.join(PACKAGE_DIR, sub, yaml_name))
                   for sub in (os.pardir, 'config', '')]
    for path in candidates:
        if os.path.exists(path):
            with open(path) as f:
                return yaml.safe_load(f)
    raise FileNotFoundError(f"找不到 {yaml_name}，已查找: {candidates}")


def quat_to_matrix(w, x, y, z):
    norm = np.sqrt(w**2 + x**2 + y**2 + z**2)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array([
        [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
    ])


def pose_to_matrix(position, orientation):
    if isinstance(position, list):
        x, y, z = position
    else:
        x, y, z = position.x, position.y, position.z
    if isinstance(orientation, list):
        qw, qx, qy, qz = orientation
    else:
        qw, qx, qy, qz = orientation.w, orientation.x, orientation.y, orientation.z
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(qw, qx, qy, qz)
    T[:3, 3] = [x, y, z]
    return T


def pose_msg_to_matrix(pose_msg):
    if pose_msg is None:
        return None
    return pose_to_matrix(pose_msg.pose.position, pose_msg.pose.orientation)


def invert_homogeneous(T):
    T = np.asarray(T, dtype=float)
    R_inv = T[:3, :3].T
    T_inv = np.eye(4)
    T_inv[:3, :3] = R_inv
    T_inv[:3, 3] = -R_inv @ T[:3, 3]
    return T_inv


def split_rotation_translation(T_list):
    return [T[:3, :3] for T in T_list], [T[:3, 3] for T in T_list]


def build_homogeneous(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def reset_records():
    """每次启动标定节点时清空采集数据。"""
    os.makedirs(RECORD_DIR, exist_ok=True)
    for filename in (T_CAM2TARGET_FILE, T_BASE2END_FILE):
        open(filename, 'w').close()


def load_json_records(filename):
    try:
        with open(filename) as f:
            data = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return []
    return [np.array(m, dtype=float) for m in data]


def append_json_record(filename, matrix):
    try:
        with open(filename) as f:
            records = json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        records = []
    records.append(np.asarray(matrix).tolist())
    with open(filename, 'w') as f:
        json.dump(records, f, indent=4)
    return len(records)


def format_matrix(T, title=''):
    lines = [title] if title else []
    for row in np.asarray(T):
        lines.append("[ " + ", ".join(f"{v: .6f}" for v in row) + " ],")
    return "\n".join(lines)
