from ..base.config import Parallel2RSS1UConfig, Vector3
from ..base.parallel_2rss1u import Parallel2RSS1UTransmission

from ..base.types import *
import numpy as np

def make_t1mini_ankle_pr_config() -> Parallel2RSS1UConfig:
    return Parallel2RSS1UConfig(
        roll_axis_center=Vector3(0.0, 0.0, 0.0),
        motor_center_a=Vector3(0.02728, 0.0, 0.16266),
        motor_center_b=Vector3(0.03142, 0.0, 0.22652),
        
        crank_zero_a=Vector3(0.02728, 0.01999, 0.16148),
        crank_zero_b=Vector3(0.03142, -0.01994, 0.22734),
        
        motor_sagittal_tilt=np.radians(3.71), 
        
        base_attach_a=Vector3(0.025, 0.018, 0.0),
        base_attach_b=Vector3(0.025, -0.018, 0.0),
        
        pitch_range=(np.radians(-90), np.radians(90)), 
        roll_range=(np.radians(-90), np.radians(90)),
    )
    
T1MINI_ANKLE_PR_CONFIG = make_t1mini_ankle_pr_config()

class T1miniAnklePRTransmission(Parallel2RSS1UTransmission):    
    def __init__(self):
        super().__init__(T1MINI_ANKLE_PR_CONFIG)
    
