import numpy as np
from dataclasses import dataclass


@dataclass(slots=True)
class CrankDiffTransConfig:
    reduce_pitch: float   # 电机角度平均值与pitch角度的比例
    # reduce_roll: float    # 电机角度差与roll角度的比例
    roll_reduce_range: tuple[float, float] # roll 减速比范围，对应于roll 角度范围
    roll_reduce_range_angle: tuple[float, float] # roll 角度范围，对应于roll 减速比范围
    
    pitch_offset: float   # 电机0度时，pitch轴的角度偏移，单位为弧度
    roll_offset: float    # 电机0度时，roll轴的角度偏移，单位为弧度
    
    pitch_range: tuple[float, float] # [-30, 60]
    roll_range: tuple[float, float] #[-20, 20]
    
    
@dataclass(slots=True)
class Vector3:
    x: float
    y: float
    z: float
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
    
    @classmethod
    def from_array(cls, arr: np.ndarray) -> "Vector3":
        return cls(x=float(arr[0]), y=float(arr[1]), z=float(arr[2]))

@dataclass(slots=True)
class Parallel2RSS1UConfig:
    """ 2RSS1U 并联关节配置
    
    坐标系约定：
    - X: 前后方向 (向前为正)
    - Y: 左右方向 (向左为正)  
    - Z: 上下方向 (向上为正)
    
    原点：
    1: 万向节(十字轴)中心
    2: pitch轴和roll轴的投影交点且位于pitch轴上。    
    
    
    pitch 和roll位于0度的时候 各个关键点的位置，成为该模块的参数
    """    
    roll_axis_center: Vector3 # roll轴中心位置
    
    motor_center_a: Vector3 # 电机a中心位置， 连接左边铰链点的电机
    motor_center_b: Vector3 # 电机b中心位置， 连接右边铰链点的电机
    
    
    crank_zero_a: Vector3 # 在pitch和roll为0度时 电机a曲柄的位置
    crank_zero_b: Vector3 # 在pitch和roll为0度时 电机b曲柄的位置
    motor_sagittal_tilt : float # 电机在sagittal平面上的倾斜角度，单位为弧度
    
    base_attach_a: Vector3 # 在pitch和roll为0度时，电机a对应连杆在 PR 基座（如脚掌）连接点的位置
    base_attach_b: Vector3 # 在pitch和roll为0度时，电机b对应连杆在 PR 基座（如脚掌）连接点的位置
    
    pitch_range: tuple[float, float] # pitch 关节的运动范围，单位为弧度
    roll_range: tuple[float, float] # roll 关节的运动范围，单位为弧度