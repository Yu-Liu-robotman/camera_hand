#!/usr/bin/env python3
"""手眼标定启动。

detect_model: 'hand_calibration_input'  眼在手上（相机在末端，码在台面）
              'hand_calibration_output' 眼在手外（相机固定，码在末端）
"""
import os

import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

MODES = {
    'hand_calibration_input': ('end2camera_calibration.py', 'end2CamCalibration'),
    'hand_calibration_output': ('base2camera_calibration.py', 'base2CamCalibration'),
}


def load_params(yaml_name='camare_params.yaml'):
    # 刻意不 import apriltags_ros：ros2 CLI 跑在系统 python 下，这个包只装在 conda 环境里
    candidates = [os.environ.get('APRILTAGS_ROS_PARAMS')]
    path = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        candidates += [os.path.join(path, yaml_name),
                       os.path.join(path, 'src', yaml_name)]
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    for candidate in candidates:
        if candidate and os.path.exists(candidate):
            with open(candidate) as f:
                return yaml.safe_load(f), candidate
    raise RuntimeError(f"找不到 {yaml_name}，已查找: {candidates}")


def generate_launch_description():
    params, path = load_params()
    print(f'[CameraTagRun] 参数: {path}')

    detect_model = params['detect_model']
    if detect_model not in MODES:
        raise RuntimeError(f"detect_model='{detect_model}' 不是标定模式，可选 {list(MODES)}")
    executable, node_name = MODES[detect_model]
    marker_name = params['marker_name']

    camera_args = {
        'camera_name': params['camera_name'],
        'rgb_camera.color_profile': params['camera_profile'],
        'enable_depth': 'false',
        'align_depth.enable': 'false',
    }
    if str(params.get('camera_serial_no', '')).strip():
        camera_args['serial_no'] = f" '{params['camera_serial_no']}' "

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'
            ]),
            launch_arguments=camera_args.items()
        ),
        Node(
            package='aruco_ros',
            executable='single',
            name=marker_name,
            parameters=[{
                'marker_id': params['marker_id'],
                'marker_size': params['marker_size'],
                'image_is_rectified': True,
                'camera_frame': 'camera_optical',
                'marker_frame': f"{marker_name}_ID{params['marker_id']}",
                'corner_refinement': 'LINES',
                'marker_dictionary': marker_name,
                'error_correction_rate': 0.1,
            }],
            remappings=[
                ('/image', f"/camera/{params['camera_name']}/color/image_raw"),
                ('/camera_info', f"/camera/{params['camera_name']}/color/camera_info"),
            ]
        ),
        Node(
            package='apriltags_ros',
            executable=executable,
            name=node_name,
            output='screen',
            emulate_tty=True,
        ),
    ])
