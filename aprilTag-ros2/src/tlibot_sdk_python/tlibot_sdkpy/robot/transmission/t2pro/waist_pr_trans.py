from ..base.config import CrankDiffTransConfig
from ..base.differential import CrankDifferentialTransmission

from ..base.types import *
import numpy as np

def make_t2pro_waist_pr_config() -> CrankDiffTransConfig:
    return CrankDiffTransConfig(
        reduce_pitch=1.0,   # 电机角度平均值与pitch角度的比例
        roll_reduce_range=(2.41, 3.0), # roll 减速比范围，对应于roll 角度范围
        # roll_reduce_range=(2.5, 2.5), # roll 减速比范围，对应于roll 角度范围
        roll_reduce_range_angle=(np.radians(0), np.radians(20)), # roll 角度范围，对应于roll 减速比
        
        pitch_offset=np.radians(0),   # 电机0度时，pitch轴的角度偏移，单位为弧度
        roll_offset=np.radians(0.0),    # 电机0度时，roll轴
        
        pitch_range=[np.radians(-30), np.radians(60)], # [-30, 60]
        roll_range=[np.radians(-20), np.radians(20)] #[-20, 20]
    )
    
T2PRO_WAIST_PR_CONFIG = make_t2pro_waist_pr_config()

class T2proWaistPRTransmission(CrankDifferentialTransmission):
    def __init__(self):
        super().__init__(T2PRO_WAIST_PR_CONFIG)
    