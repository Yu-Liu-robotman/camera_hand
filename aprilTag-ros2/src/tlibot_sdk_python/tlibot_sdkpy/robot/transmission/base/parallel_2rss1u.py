import numpy as np

from .config import Parallel2RSS1UConfig
from .types import *
from .transmission import ParallelTransmission

class Parallel2RSS1UTransmission(ParallelTransmission):
    def __init__(self, cfg: Parallel2RSS1UConfig):
        self.config = cfg
        # base 坐标系定义在pitch 轴和 roll 轴的投影交点且位于pitch轴上。
        self.h0 = cfg.roll_axis_center.to_array()

        # 下方电机（motor_a）固定点
        self.ra_a = cfg.motor_center_a.to_array()
        # 上方电机（motor_b）固定点
        self.ra_b = cfg.motor_center_b.to_array()

        self.rb_a = cfg.crank_zero_a.to_array()
        self.rb_b = cfg.crank_zero_b.to_array()

        self.lbar_a = np.linalg.norm(self.rb_a - self.ra_a)
        self.lbar_b = np.linalg.norm(self.rb_b - self.ra_b)

        # 支架 俯仰 倾斜角 (rad)，正值：前俯
        self.delta_offset = cfg.motor_sagittal_tilt
        self.R_off = self.RyPlot(self.delta_offset)    # 局部系 -> 世界系 的恒定旋转
        self.axis_world = self.R_off @ np.array([1.0, 0.0, 0.0])  # 电机真实转轴(世界系下)

        self.rc_a = cfg.base_attach_a.to_array()
        self.rc_b = cfg.base_attach_b.to_array()
        self.hc_a = self.rc_a - self.h0
        self.hc_b = self.rc_b - self.h0

        self.lrod_a = np.linalg.norm(self.rb_a - self.rc_a)
        self.lrod_b = np.linalg.norm(self.rb_b - self.rc_b)

        self.re_a = np.array([cfg.base_attach_a.x + 0.1, cfg.base_attach_a.y, cfg.base_attach_a.z])
        self.re_b = np.array([cfg.base_attach_b.x + 0.1, cfg.base_attach_b.y, cfg.base_attach_b.z])
        self.he_a = self.re_a - self.h0
        self.he_b = self.re_b - self.h0

        self.rab_a_y = np.array([0, self.lbar_a, 0])
        self.rab_b_y = np.array([0, -self.lbar_b, 0])

        rab_a_local = self.R_off.T @ (self.rb_a - self.ra_a)
        rab_b_local = self.R_off.T @ (self.rb_b - self.ra_b)

        self.m_a = np.arctan2(rab_a_local[2], rab_a_local[1])
        self.m_b = np.arctan2(-rab_b_local[2], -rab_b_local[1])

        self.pitch_range = cfg.pitch_range
        self.roll_range = cfg.roll_range

        self._last_pitch = 0.0
        self._last_roll = 0.0

    def RyPlot(self, pitch):
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])
        return Ry

    def RxPlot(self, roll):
        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
        return Rx

    def xrot(self, pitch, roll):
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                       [0, 1, 0],
                       [-np.sin(pitch), 0, np.cos(pitch)]])

        Rx = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])

        return Ry @ Rx

    def ik(self, pitch, roll):
        error_state = 0

        rh = self.RyPlot(pitch) @ self.h0
        rc_a = rh + self.xrot(pitch, roll) @ self.hc_a
        ra_a = self.ra_a

        rac_a = self.R_off.T @ (rc_a - ra_a)
        c_a = -(self.lrod_a**2 - self.lbar_a**2 - rac_a[0]**2 - rac_a[1]**2 - rac_a[2]**2) / (2 * self.lbar_a)

        b_a = rac_a[1]
        a_a = rac_a[2]

        jjj = a_a * a_a * c_a * c_a - (b_a**2 + a_a**2) * (c_a**2 - b_a**2)
        if jjj < .0:
            print("1 Warning: Argument <0 ,jjj:  ", jjj)
            jjj = .0001
            error_state = 1

        ss = (a_a * c_a + np.sqrt(jjj)) / (b_a**2 + a_a**2)
        if ss < -1:
            print("1 Warning:  ,ss:  ", ss)
            ss = -1
            error_state = 2
        elif ss > 1:
            print("1 Warning:  ,ss:  ", ss)
            ss = 1
            error_state = 3

        motor_a_angle = np.arcsin(ss) - self.m_a

        rc_b = rh + self.xrot(pitch, roll) @ self.hc_b
        ra_b = self.ra_b
        rac_b = self.R_off.T @ (rc_b - ra_b)
        c_b = (self.lrod_b**2 - rac_b[0]**2 - rac_b[1]**2 - self.lbar_b**2 - rac_b[2]**2) / (2 * self.lbar_b)

        b_b = rac_b[1]
        a_b = rac_b[2]
        jjj = a_b**2 * c_b**2 - (b_b**2 + a_b**2) * (c_b**2 - b_b**2)
        if jjj < .0:
            jjj = .0
            print("2Warning: Argument <0 ,jjj:  ", jjj)
            error_state = 4

        ss = (a_b * c_b - np.sqrt(jjj)) / (b_b**2 + a_b**2)
        if ss < -1:
            print("1 Warning:  ,ss:  ", ss)
            ss = -1
            error_state = 5
        elif ss > 1:
            print("1 Warning:  ,ss:  ", ss)
            ss = 1
            error_state = 6

        motor_b_angle = np.arcsin(ss) - self.m_b

        # motor_a_angle 为下方(连接左边铰链点)电机角度，motor_b_angle 为上方(连接右边铰链点)电机角度
        return motor_a_angle, motor_b_angle, error_state

    def Jac(self, pitch, roll, motor_a_angle, motor_b_angle):

        Jx = np.zeros((2, 6))
        Jtheta = np.zeros((2, 2))
        G = np.array([[0, 0, 0, np.cos(pitch), 0, -np.sin(pitch)],
                      [0, 0, 0, 0, 1, 0]]).transpose()

        s1_a = self.axis_world
        s1_b = self.axis_world

        rb_a = self.ra_a + self.R_off @ (self.RxPlot(motor_a_angle + self.m_a) @ self.rab_a_y)
        rh = self.RyPlot(pitch) @ self.h0

        rc_a = rh + self.xrot(pitch, roll) @ self.hc_a
        ra_a = self.ra_a

        rbar_a = rb_a - ra_a
        rrod_a = rc_a - rb_a

        rb_b = self.ra_b + self.R_off @ (self.RxPlot(motor_b_angle + self.m_b) @ self.rab_b_y)
        rc_b = rh + self.xrot(pitch, roll) @ self.hc_b
        ra_b = self.ra_b

        rbar_b = rb_b - ra_b
        rrod_b = rc_b - rb_b

        Jtheta = np.array([[s1_a @ np.cross(rbar_a, rrod_a), 0],
                           [0, s1_b @ np.cross(rbar_b, rrod_b)]])

        Jx[0, 0:3] = rrod_a.transpose()
        Jx[0, 3:6] = (np.cross(rc_a, rrod_a)).transpose()

        Jx[1, 0:3] = rrod_b.transpose()
        Jx[1, 3:6] = (np.cross(rc_b, rrod_b)).transpose()

        Jc = np.linalg.inv(Jtheta) @ (Jx @ G)

        rank = np.linalg.matrix_rank(Jc)
        error_state = 0
        if rank != 2:
            print("jac Warning:  , rank:  Jc:  ", rank, Jc)
            error_state = 1
        return Jc, error_state

    def _joint2motor_raw(self, pitch, roll, pitch_vel, roll_vel):
        """
        将脚掌姿态（角度+角速度）转换为两个电机的角度和角速度

        参数:
            pitch (float): 脚掌俯仰角 (rad)
            roll  (float): 脚掌横滚角 (rad)
            pitch_vel (float): 脚掌俯仰角速度 (rad/s)
            roll_vel  (float): 脚掌横滚角速度 (rad/s)

        返回:
            motor_a_angle (float): motor_a（连接左边铰链点）的角度 (rad)
            motor_b_angle (float): motor_b（连接右边铰链点）的角度 (rad)
            motor_a_vel   (float): motor_a的角速度 (rad/s)
            motor_b_vel   (float): motor_b的角速度 (rad/s)
            error_state  (int)   : 0=正常, >0=出错
        """
        # 1. 位置逆解：脚掌姿态 -> 电机角度
        motor_a_angle, motor_b_angle, error_state = self.ik(pitch, roll)

        if error_state != 0:
            return np.nan, np.nan, np.nan, np.nan, error_state

        # 2. 计算雅可比矩阵
        Jc, jac_error = self.Jac(pitch, roll, motor_a_angle, motor_b_angle)
        if jac_error != 0:
            error_state = jac_error  # 记录雅可比奇异警告，但仍可计算（可能不准）

        # 3. 速度映射：脚掌角速度（[roll_vel, pitch_vel]）-> 电机角速度
        # 注意：Jac() 的列顺序为 [roll, pitch]，因此输入向量必须为 [roll_vel, pitch_vel]
        foot_vel = np.array([roll_vel, pitch_vel])
        motor_vel = Jc @ foot_vel  # motor_vel = [motor_a速度, motor_b速度]

        motor_a_vel = motor_vel[0]
        motor_b_vel = motor_vel[1]

        return motor_a_angle, motor_b_angle, motor_a_vel, motor_b_vel, error_state

    def _motor2joint_raw(self, motor_a_angle, motor_b_angle, motor_a_vel, motor_b_vel):
        """
        将电机的角度和角速度转换为脚掌的姿态和角速度（joint2motor 的反方向）

        参数:
            motor_a_angle (float): motor_a（连接左边铰链点）角度 (rad)
            motor_b_angle (float): motor_b（连接右边铰链点）角度 (rad)
            motor_a_vel   (float): motor_a角速度 (rad/s)
            motor_b_vel   (float): motor_b角速度 (rad/s)

        返回:
            pitch (float): 脚掌俯仰角 (rad)
            roll  (float): 脚掌横滚角 (rad)
            pitch_vel (float): 脚掌俯仰角速度 (rad/s)
            roll_vel  (float): 脚掌横滚角速度 (rad/s)
            error_state (int) : 0=正常, >0=出错
        """
        # 1. 正运动学：电机角度 -> 脚掌姿态（用0作为初始猜测）
        pitch, roll, error_state = self.fw(self._last_pitch, self._last_roll, motor_a_angle, motor_b_angle)
        if error_state != 0:
            return np.nan, np.nan, np.nan, np.nan, error_state
        if error_state == 0:
            self._last_pitch = pitch
            self._last_roll = roll
        # 2. 计算当前姿态下的雅可比矩阵
        Jc, jac_error = self.Jac(pitch, roll, motor_a_angle, motor_b_angle)
        if jac_error != 0:
            error_state = jac_error
            return pitch, roll, np.nan, np.nan, error_state

        # 3. 速度映射：电机速度 -> 脚掌角速度
        motor_vel = np.array([motor_a_vel, motor_b_vel])
        try:
            Jc_inv = np.linalg.inv(Jc)
        except np.linalg.LinAlgError:
            print("motor2joint: 雅可比矩阵不可逆")
            return pitch, roll, np.nan, np.nan, 2

        foot_vel = Jc_inv @ motor_vel  # foot_vel = [roll_vel, pitch_vel]
        roll_vel = foot_vel[0]
        pitch_vel = foot_vel[1]

        return pitch, roll, pitch_vel, roll_vel, error_state

    def motor2joint_torque(self, pitch, roll, motor_a_angle, motor_b_angle, motor_torque):
        """
        力矩正解：已知两个电机的输出力矩，求解脚掌（末端）等效力矩

        原理（虚功/功率对偶）：
            motorVel = Jc @ jointVel
            由虚功原理 motorTorque^T @ motorVel = jointTorque^T @ jointVel 对任意 jointVel 成立
            => jointTorque = Jc^T @ motorTorque
        即：速度用 Jc 传递（joint->motor），力矩用 Jc^T 反向传递（motor->joint）。
        该关系只在理想无摩擦、无质量（准静态）假设下成立。

        参数:
            pitch, roll: 当前脚掌姿态 (rad)
            motor_a_angle, motor_b_angle: 当前两个电机角度 (rad)，用于计算雅可比
            motor_torque: np.array([motor_a_torque, motor_b_torque]) 电机力矩 (N·m)

        返回:
            joint_torque: np.array([roll_torque, pitch_torque]) 脚掌等效力矩 (N·m)
            error_state: 0=正常, 1=雅可比奇异警告
        """
        Jc, error_state = self.Jac(pitch, roll, motor_a_angle, motor_b_angle)
        joint_torque = Jc.transpose() @ np.asarray(motor_torque)
        return joint_torque, error_state

    def joint2motor_torque(self, pitch, roll, motor_a_angle, motor_b_angle, joint_torque):
        """
        力矩逆解：已知期望的脚掌（末端）力矩，求解两个电机需要输出的力矩

        原理：对 motor2joint_torque 的关系求逆
            jointTorque = Jc^T @ motorTorque
            => motorTorque = (Jc^T)^-1 @ jointTorque = inv(Jc).T @ jointTorque

        参数:
            pitch, roll: 当前脚掌姿态 (rad)
            motor_a_angle, motor_b_angle: 当前两个电机角度 (rad)，用于计算雅可比
            joint_torque: np.array([roll_torque, pitch_torque]) 期望的脚掌力矩 (N·m)
                          注意顺序与 Jac() 的列顺序一致，为 [roll, pitch]

        返回:
            motor_torque: np.array([motor_a_torque, motor_b_torque]) 电机需要输出的力矩 (N·m)
            error_state: 0=正常, 1=雅可比奇异（此时无法可靠求逆，结果不可信）
        """
        Jc, error_state = self.Jac(pitch, roll, motor_a_angle, motor_b_angle)
        try:
            motor_torque = np.linalg.inv(Jc.transpose()) @ np.asarray(joint_torque)
        except np.linalg.LinAlgError:
            print("joint2motor_torque Warning: Jc is singular, cannot invert")
            return np.array([np.nan, np.nan]), 1
        return motor_torque, error_state

    def joint2motor(self, joint_pitch: JointState, joint_roll: JointState) -> tuple[ActuatorState, ActuatorState]:
        """关节空间 -> 电机空间 (逆解)，实现 ParallelTransmission 抽象接口

        同时完成位置/速度/力矩的正向传递：
            - 位置、速度：脚掌姿态 -> 电机角度/角速度
            - 力矩：脚掌等效力矩(joint_pitch.effort, joint_roll.effort) -> 电机力矩

        返回:
            (motor_a, motor_b): 与 motor2joint 的入参顺序一致
        """
        motor_a_angle, motor_b_angle, motor_a_vel, motor_b_vel, error_state = self._joint2motor_raw(
            joint_pitch.pos, joint_roll.pos, joint_pitch.vel, joint_roll.vel
        )
        if error_state != 0:
            print(f"joint2motor: 解算异常，error_state={error_state}")

        motor_a_effort = np.nan
        motor_b_effort = np.nan
        if error_state == 0:
            joint_torque = np.array([joint_roll.effort, joint_pitch.effort])
            motor_torque, torque_error = self.joint2motor_torque(
                joint_pitch.pos, joint_roll.pos, motor_a_angle, motor_b_angle, joint_torque
            )
            if torque_error != 0:
                print(f"joint2motor: 力矩解算异常，error_state={torque_error}")
            motor_a_effort, motor_b_effort = motor_torque[0], motor_torque[1]

        return (
            ActuatorState(pos=float(motor_a_angle), vel=float(motor_a_vel), effort=float(motor_a_effort)),
            ActuatorState(pos=float(motor_b_angle), vel=float(motor_b_vel), effort=float(motor_b_effort)),
        )

    def motor2joint(self, motor_a: ActuatorState, motor_b: ActuatorState) -> tuple[JointState, JointState]:
        """电机空间 -> 关节空间 (正解)，实现 ParallelTransmission 抽象接口

        同时完成位置/速度/力矩的正向传递：
            - 位置、速度：电机角度/角速度 -> 脚掌姿态
            - 力矩：电机力矩(motor_a.effort, motor_b.effort) -> 脚掌等效力矩
        """
        pitch, roll, pitch_vel, roll_vel, error_state = self._motor2joint_raw(
            motor_a.pos, motor_b.pos, motor_a.vel, motor_b.vel
        )
        if error_state != 0:
            print(f"motor2joint: 解算异常，error_state={error_state}")

        pitch_effort = np.nan
        roll_effort = np.nan
        if error_state == 0:
            motor_torque = np.array([motor_a.effort, motor_b.effort])
            joint_torque, torque_error = self.motor2joint_torque(
                pitch, roll, motor_a.pos, motor_b.pos, motor_torque
            )
            if torque_error != 0:
                print(f"motor2joint: 力矩解算异常，error_state={torque_error}")
            roll_effort, pitch_effort = joint_torque[0], joint_torque[1]

        return (
            JointState(pos=float(pitch), vel=float(pitch_vel), effort=float(pitch_effort)),
            JointState(pos=float(roll), vel=float(roll_vel), effort=float(roll_effort)),
        )

    def fw(self, pitchRef, rollRef, motor_a_angle, motor_b_angle):
        epsilon = 1e-6
        max_iterations = 100
        dt = 1.0
        pitch = pitchRef
        roll = rollRef
        error_state = 0

        for i in range(max_iterations):

            if pitch > self.pitch_range[1]:
                pitch = self.pitch_range[1]
            elif pitch < self.pitch_range[0]:
                pitch = self.pitch_range[0]

            if roll > self.roll_range[1]:
                roll = self.roll_range[1]
            elif roll < self.roll_range[0]:
                roll = self.roll_range[0]

            m_a, m_b, _ = self.ik(pitch, roll)
            if (np.isnan(m_a) or np.isnan(m_b)):
                error_state = 1
                return 0, 0, error_state

            error = np.array([m_a - motor_a_angle, m_b - motor_b_angle])

            if np.linalg.norm(error) < epsilon:
                print("逆运动学收敛！")
                print(i)
                return pitch, roll, error_state

            Jc, _ = self.Jac(pitch, roll, m_a, m_b)

            try:
                JcInv = np.linalg.inv(Jc)
            except np.linalg.LinAlgError:
                print(" jc inv is error ")
                error_state = 1
                return 0, 0, error_state

            # v = JcInv @ error
            # pitch -= v[1] * dt
            # roll -= v[0] * dt

            v = JcInv @ error
            step = v.copy()
            max_step = np.radians(5.0)  # 每次迭代最多允许5度变化，可调
            norm = np.linalg.norm(step)
            if norm > max_step:
                step = step * (max_step / norm)
            pitch -= step[1] * dt
            roll -= step[0] * dt

        else:
            print("逆运动学未在最大迭代次数内收敛。")

        error_state = 1
        return 0, 0, error_state