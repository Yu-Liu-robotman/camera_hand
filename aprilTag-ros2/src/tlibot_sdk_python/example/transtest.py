from tlibot_sdkpy.robot.transmission.t2pro.waist_pr_trans import T2proWaistPRTransmission
from tlibot_sdkpy.robot.transmission.base.types import JointState, ActuatorState

import numpy as np

if __name__ == '__main__':
   # print("Press Enter to continue...")

    trans = T2proWaistPRTransmission()
     
    j = trans.motor2joint(ActuatorState(pos=np.radians(20), vel=0.0, effort=0.0),
                          ActuatorState(pos=np.radians(20), vel=np.radians(10), effort=5.0))
    
    print(f"j: pitch={j[0].as_degrees()}, roll={j[1].as_degrees()}")
    
    m = trans.joint2motor(j[0], j[1])
    
    motors = trans.joint2motor(JointState(pos=np.radians(30), vel=0.0, effort=0.0),
                               JointState(pos=np.radians(10), vel=0.0, effort=0.0))
    
    print(f"motors: m1={motors[0].as_degrees()}, m2={motors[1].as_degrees()}")
        
    
    out_j = trans.motor2joint(motors[0], motors[1])
    
    print (f"out_j: pitch={out_j[0].as_degrees()}, roll={out_j[1].as_degrees()}")
    
    