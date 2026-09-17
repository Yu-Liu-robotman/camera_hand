#!/usr/bin/env python3
"""用 URDF 算手臂正解，发布到 forward_kinematics_topic（PoseStamped）。
关节角来自 tlibot SDK：按参数 arm 取 ARM_JOINTS 里 arm_left_* 或 arm_right_* 的条目。
"""
import os

import pinocchio as pin
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from apriltags_ros.include.calibration_utils import get_params
from read_arm_joints import ARM_JOINTS, ArmJointReader

params = get_params()
MODEL_DIR = os.path.dirname(os.path.abspath(__file__))


class Base2EndPose(Node):

    def __init__(self):
        super().__init__('base2endPose')
        self.declare_parameter('arm', params.get('fk_arm', 'left'))
        self.declare_parameter('topic', params.get('forward_kinematics_topic', '/base2end/pose'))
        arm = self.get_parameter('arm').value
        topic = self.get_parameter('topic').value

        urdf = os.path.join(MODEL_DIR, params.get('fk_urdf', 'z1_v1.0/urdf/z1_v1.urdf'))
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()

        # 只取选中臂的关节，手臂以外的一律不进链路
        self.joints = [n for n, _ in ARM_JOINTS if n.startswith(f'arm_{arm}_')]
        missing = [n for n in self.joints if not self.model.existJointName(n)]
        if missing:
            raise RuntimeError(f'{os.path.basename(urdf)} 里没有这些关节: {missing}')

        self.root_link = params.get('fk_root_link') or 'arm_base_link'
        self.root_frame_id = self.model.getFrameId(self.root_link)
        self.end_link = params.get('fk_end_link') or f'arm_{arm}_wrist_yaw_link'
        self.end_frame_id = self.model.getFrameId(self.end_link)
        self.reader = ArmJointReader(params['domain_id'], params.get('net') or None)
        self.pub = self.create_publisher(PoseStamped, topic, 1)
        hz = float(params.get('fk_hz', 50.0))
        self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(f'{arm} 臂 {len(self.joints)} 个关节  '
                               f'{self.root_link} -> {self.end_link}，发布 {topic} @ {hz}Hz')

    def tick(self):
        state = self.reader.read()
        if state is None:
            return
        v = pin.utils.zero(self.model.nv)
        for name in self.joints:
            v[self.model.idx_vs[self.model.getJointId(name)]] = state[name][0]
        q = pin.integrate(self.model, pin.neutral(self.model), v)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        M = self.data.oMf[self.root_frame_id].inverse() * self.data.oMf[self.end_frame_id]

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.root_link
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = M.translation
        (msg.pose.orientation.x, msg.pose.orientation.y,
         msg.pose.orientation.z, msg.pose.orientation.w) = pin.Quaternion(M.rotation).coeffs()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = Base2EndPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
