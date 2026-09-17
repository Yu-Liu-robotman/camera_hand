# 仅读取手臂关节位置，不下发任何指令
# python read_arm_joints.py [--hz 5] [--domain 20] [--net enp4s0] [--topic rt/robotstate]
import argparse
import sys
import time

from tlibot_sdkpy.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from tlibot_sdkpy.idl.msg.dds import RobotState

from apriltags_ros.include.calibration_utils import get_params

params = get_params()

# 关节名 -> 电机 ID (23dof 实机)
ARM_JOINTS = [
    ("arm_left_shoulder_pitch_joint", 12),
    ("arm_left_shoulder_roll_joint", 13),
    ("arm_left_shoulder_yaw_joint", 14),
    ("arm_left_elbow_joint", 15),
    ("arm_left_wrist_roll_joint", 16),
    ("arm_left_wrist_pitch_joint", 17),
    ("arm_left_wrist_yaw_joint", 18),
    
    ("arm_right_shoulder_pitch_joint", 19),
    ("arm_right_shoulder_roll_joint", 20),
    ("arm_right_shoulder_yaw_joint", 21),
    ("arm_right_elbow_joint", 22),
    ("arm_right_wrist_roll_joint", 23),
    ("arm_right_wrist_pitch_joint", 24),
    ("arm_right_wrist_yaw_joint", 25),
]

class ArmJointReader:
    def __init__(self, domain_id: int = 20, net: str = None, topic: str = "rt/robotstate"):
        self.joints = ARM_JOINTS
        ChannelFactoryInitialize(domain_id, net)

        self.latest = None
        self.subscriber = ChannelSubscriber(topic, RobotState)
        self.subscriber.Init(self._handler, 10)

    def _handler(self, msg: RobotState):
        self.latest = msg

    def read(self):
        """返回 {关节名: (pos, vel, tau)}，尚未收到数据时返回 None"""
        msg = self.latest
        if msg is None:
            return None
        out = {}
        for name, mid in self.joints:
            ms = msg.motor_state[mid]
            out[name] = (ms.q, ms.dq, ms.tau)
        return out


def main():
    parser = argparse.ArgumentParser(description="仅读取手臂关节位置")
    parser.add_argument("--hz", type=float, default=50.0, help="刷新频率")
    parser.add_argument("--domain", type=int, default=params['domain_id'], help="DDS domain id")
    parser.add_argument("--net", default=params.get('net') or None, help="网卡名, 默认自动选择")
    parser.add_argument("--topic", default="rt/robotstate", help="lowstate 话题")
    args = parser.parse_args()
    period = 1.0 / args.hz

    reader = ArmJointReader(args.domain, args.net, args.topic)
    print(f"domain={args.domain}  net={args.net or 'auto'}  topic={args.topic}  ctrl-C 退出\n")
    print(f"{'joint':<34}{'pos(rad)':>12}{'vel(rad/s)':>13}{'tau(Nm)':>12}")

    printed = 0
    try:
        while True:
            state = reader.read()
            if state is None:
                time.sleep(period)
                continue
            if printed:
                sys.stdout.write(f"\033[{printed}F")  # 光标回退，原地刷新
            printed = 0
            for name, (pos, vel, tau) in state.items():
                print(f"{name:<34}{pos:>12.4f}{vel:>13.4f}{tau:>12.4f}")
                printed += 1
            sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        reader.subscriber.Close()

if __name__ == "__main__":
    main()
