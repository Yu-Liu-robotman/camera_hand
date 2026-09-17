import numpy as np

from .types import *
from .config import CrankDiffTransConfig
from .transmission import ParallelTransmission


class CrankDifferentialTransmission(ParallelTransmission):
    def __init__(self, cfg: CrankDiffTransConfig):
        self.config = cfg
        # 计算roll减速比的线性拟合系数, 获得的系数用于根据roll角度动态计算 减速比
        self.coef_roll_reduce_roll = np.polyfit(cfg.roll_reduce_range_angle, cfg.roll_reduce_range, 1)
        
        self.roll_reduce_range_motordiff = [cfg.roll_reduce_range[0] * cfg.roll_reduce_range_angle[0] * 2.0,
                                            cfg.roll_reduce_range[1] * cfg.roll_reduce_range_angle[1] * 2.0]
        
        # 计算电机角度差与roll角度的比例的线性拟合系数, 获得的系数用于根据电机角度差动态计算 减速比
        self.coef_roll_reduce_motordiff = np.polyfit(self.roll_reduce_range_motordiff, cfg.roll_reduce_range, 1)
        

    def joint2motor(self, joint_pitch: JointState, joint_roll: JointState) -> tuple[ActuatorState, ActuatorState]:
        """关节空间 -> 电机空间 (逆解)"""
        m1 = ActuatorState()
        m2 = ActuatorState()
        
        # 现根据roll角度，计算roll的动态减速比
        roll_reduce = self.dynamic_roll_reduce(joint_roll.pos)
        
        # 1 根据pitch和roll角度，算出电机1 2 的角度
        m1.pos = (joint_pitch.pos - self.config.pitch_offset) * self.config.reduce_pitch - (joint_roll.pos - self.config.roll_offset) * roll_reduce
        m2.pos = (joint_pitch.pos - self.config.pitch_offset) * self.config.reduce_pitch + (joint_roll.pos - self.config.roll_offset) * roll_reduce
        
        m1.vel = joint_pitch.vel * self.config.reduce_pitch - joint_roll.vel * roll_reduce
        m2.vel = joint_pitch.vel * self.config.reduce_pitch + joint_roll.vel * roll_reduce
        
        m1.effort = (joint_pitch.effort / self.config.reduce_pitch - joint_roll.effort / roll_reduce) / 2.0
        m2.effort = (joint_pitch.effort / self.config.reduce_pitch + joint_roll.effort / roll_reduce) / 2.0
        
        return m1, m2
    
    
    def motor2joint(self, m1: ActuatorState, m2: ActuatorState) -> tuple[JointState, JointState]:
        """电机空间 -> 关节空间 (正解)"""
        joint_pitch = JointState()
        joint_roll = JointState()
        
        # 1 根据电机1 2 的角度，算出pitch
        joint_pitch.pos = (m1.pos + m2.pos) / (2.0 * self.config.reduce_pitch) + self.config.pitch_offset
        motordiff = m2.pos - m1.pos
        roll_reduce = self.dynamic_roll_reduce_motordiff(motordiff)
                
        joint_roll.pos = motordiff / (2.0 * roll_reduce) + self.config.roll_offset
        
        joint_pitch.vel = (m1.vel + m2.vel) / (2.0 * self.config.reduce_pitch)
        joint_roll.vel = (m2.vel - m1.vel) / (2.0 * roll_reduce)
        
        joint_pitch.effort = (m1.effort + m2.effort) * self.config.reduce_pitch
        joint_roll.effort = (m2.effort - m1.effort) * roll_reduce
                
        return joint_pitch, joint_roll
    
    def dynamic_roll_reduce(self, joint_roll: float) -> float:
        """根据roll角度，动态计算电机角度差与roll角度的比例"""
        # 线性插值
        roll_min, roll_max = self.config.roll_reduce_range_angle
        reduce_min, reduce_max = self.config.roll_reduce_range
        if joint_roll <= roll_min:
            return reduce_min
        elif joint_roll >= roll_max:
            return reduce_max
        else:
            # 线性插值计算减速比
            return joint_roll * self.coef_roll_reduce_roll[0] + self.coef_roll_reduce_roll[1]
        
    def dynamic_roll_reduce_motordiff(self, motordiff: float) -> float:
        """根据电机角度差，动态计算电机角度差与roll角度的比例
        """
        absmotordiff = abs(motordiff)
        # 线性插值
        motordiff_min, motordiff_max = self.roll_reduce_range_motordiff
        reduce_min, reduce_max = self.config.roll_reduce_range
        if absmotordiff <= motordiff_min:
            return reduce_min
        elif absmotordiff >= motordiff_max:
            return reduce_max
        else:
            # 线性插值计算减速比
            return absmotordiff * self.coef_roll_reduce_motordiff[0] + self.coef_roll_reduce_motordiff[1]
        
    