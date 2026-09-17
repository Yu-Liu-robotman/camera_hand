#!/usr/bin/env python3
"""眼在手外标定：相机固定，aruco 码随末端运动。

图像窗口里按 G 采集一组，移动机械臂后重复，按 Q 求解并退出。
输出 T_base2camera（相机系 -> 基座系）。
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image

from apriltags_ros.include.calibration_utils import (
    T_BASE2END_FILE, T_CAM2TARGET_FILE, append_json_record, build_homogeneous,
    format_matrix, get_params, invert_homogeneous, load_json_records,
    pose_msg_to_matrix, reset_records, split_rotation_translation)

params = get_params()


class Base2CameraCalibration(Node):

    def __init__(self):
        super().__init__('base2CamCalibration')
        name, mid = params['marker_name'], params['marker_id']
        self.marker_pose = None
        self.end_pose = None

        self.create_subscription(
            PoseStamped, f'/{name}/ID{mid}/pose', self.marker_callback, 1)
        self.create_subscription(
            PoseStamped, params['forward_kinematics_topic'], self.end_callback, 1)

        self.bridge = CvBridge()
        if params['image_topic_visual']:
            self.create_subscription(
                Image, f'/{name}/ID{mid}/result', self.image_callback, 1)
        else:
            self.get_logger().warn('image_topic_visual=false，按 G/Q 无效')

        reset_records()
        self.get_logger().info(f"等待 /{name}/ID{mid} 与 "
                               f"{params['forward_kinematics_topic']}；G 采集，Q 求解")

    def marker_callback(self, msg):
        self.marker_pose = msg

    def end_callback(self, msg):
        self.end_pose = msg

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Aruco Result', cv2.resize(image, None, fx=0.5, fy=0.5))
        key = cv2.waitKey(1) & 0xFF
        if key == ord('g'):
            self.record()
        elif key == ord('q'):
            self.finish()

    def record(self):
        T_cam2target = pose_msg_to_matrix(self.marker_pose)
        T_base2end = pose_msg_to_matrix(self.end_pose)
        if T_cam2target is None:
            self.get_logger().warn('未检测到 aruco 码，本次不记录')
            return
        if T_base2end is None:
            self.get_logger().warn('未收到末端位姿，本次不记录')
            return
        append_json_record(T_CAM2TARGET_FILE, T_cam2target)
        count = append_json_record(T_BASE2END_FILE, T_base2end)
        self.get_logger().info(f'已记录第 {count} 组')
        self.marker_pose = None
        self.end_pose = None

    def compute(self):
        target = load_json_records(T_CAM2TARGET_FILE)
        ends = load_json_records(T_BASE2END_FILE)
        if min(len(target), len(ends)) < 3:
            self.get_logger().warn(f'数据不足（{min(len(target), len(ends))} 组，至少 3 组）')
            return None
        R_target2cam, t_target2cam = split_rotation_translation(target)
        # 眼在手外：喂末端位姿的逆，得到 cam2base
        R_base2gripper, t_base2gripper = split_rotation_translation(
            [invert_homogeneous(T) for T in ends])
        try:
            R, t = cv2.calibrateHandEye(R_base2gripper, t_base2gripper,
                                        R_target2cam, t_target2cam,
                                        method=cv2.CALIB_HAND_EYE_DANIILIDIS)
        except cv2.error as e:
            self.get_logger().error(f'求解失败（位姿太单调？）: {e}')
            return None
        return build_homogeneous(R, t)

    def finish(self):
        T = self.compute()
        if T is not None:
            self.get_logger().info('T_base2camera:\n' + format_matrix(T))
        self.exit_program()

    def exit_program(self):
        cv2.destroyAllWindows()
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Base2CameraCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.exit_program()
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
