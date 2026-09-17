#!/usr/bin/env python3
"""虚拟机器人：不接实机，在同一个 DDS 话题上按同样的消息格式发假关节角。

read_arm_joints.py / base2end_pose.py 无需任何改动即可拿到数据。

  python3 src/robot_model/sim_robot_state.py                            # 手臂关节正弦摆动
  python3 src/robot_model/sim_robot_state.py --angles 0.2,0,0,0,0,0,0   # 静态保持(rad)
  python3 src/robot_model/sim_robot_state.py --sweep arm_left_elbow_joint --amp 0.5
"""
import argparse
import math
import time

from tlibot_sdkpy.core.channel import ChannelPublisher, ChannelFactoryInitialize
from tlibot_sdkpy.idl.default import default_RobotState
from tlibot_sdkpy.idl.msg.dds import RobotState

from apriltags_ros.include.calibration_utils import get_params
from read_arm_joints import ARM_JOINTS

params = get_params()


def main():
    parser = argparse.ArgumentParser(description="虚拟机器人状态发布")
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--domain", type=int, default=params['domain_id'])
    parser.add_argument("--net", default=params.get('net') or None)
    parser.add_argument("--topic", default="rt/robotstate")
    parser.add_argument("--angles", default=None, help="静态角度(rad)，逗号分隔，按 ARM_JOINTS 顺序")
    parser.add_argument("--sweep", default=None, help="只让这个关节正弦摆动")
    parser.add_argument("--amp", type=float, default=0.3, help="摆动幅度(rad)")
    parser.add_argument("--freq", type=float, default=0.2, help="摆动频率(Hz)")
    args = parser.parse_args()

    static = [float(x) for x in args.angles.split(',')] if args.angles else None

    ChannelFactoryInitialize(args.domain, args.net)
    pub = ChannelPublisher(args.topic, RobotState)
    pub.Init()

    msg = default_RobotState()
    print(f"虚拟机器人: domain={args.domain} net={args.net or 'auto'} "
          f"topic={args.topic} {args.hz}Hz")
    print(f"  关节: {' '.join(n for n, _ in ARM_JOINTS)}")
    if static is not None:
        print(f"  静态角度 {static}")
    elif args.sweep:
        print(f"  {args.sweep} 摆动 ±{args.amp}rad @{args.freq}Hz")
    else:
        print(f"  全部手臂关节摆动 ±{args.amp}rad @{args.freq}Hz")

    period = 1.0 / args.hz
    t0 = time.time()
    try:
        while True:
            t = time.time() - t0
            for idx, (name, mid) in enumerate(ARM_JOINTS):
                if static is not None:
                    q = static[idx] if idx < len(static) else 0.0
                    dq = 0.0
                elif args.sweep in (None, name):
                    phase = 2 * math.pi * args.freq * t
                    q = args.amp * math.sin(phase)
                    dq = args.amp * 2 * math.pi * args.freq * math.cos(phase)
                else:
                    q = dq = 0.0
                msg.motor_state[mid].q = q
                msg.motor_state[mid].dq = dq
            pub.Write(msg)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        pub.Close()


if __name__ == '__main__':
    main()
