from .types import *   
from abc import ABC, abstractmethod

class ParallelTransmission(ABC):
    @abstractmethod
    def joint2motor(self, joint_pitch: JointState, joint_roll: JointState) -> tuple[ActuatorState, ActuatorState]:
        """关节空间 -> 电机空间 (逆解)"""
        ...
    
    @abstractmethod
    def motor2joint(self, motor_a: ActuatorState, motor_b: ActuatorState) -> tuple[JointState, JointState]:
        """电机空间 -> 关节空间 (正解)"""
        ...
        
