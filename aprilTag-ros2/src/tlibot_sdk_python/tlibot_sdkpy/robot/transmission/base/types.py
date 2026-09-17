from dataclasses import dataclass
import numpy as np

@dataclass(slots=True)
class JointState:
    pos: float = 0.0
    vel: float = 0.0
    effort: float = 0.0

    def as_degrees(self) -> "JointState":
        return JointState(
            pos=np.degrees(self.pos),
            vel=np.degrees(self.vel),
            effort=self.effort
        )
        
@dataclass(slots=True)
class ActuatorState:
    pos: float = 0.0
    vel: float = 0.0
    effort: float = 0.0
    
    def as_degrees(self) -> "ActuatorState":
        return ActuatorState(
            pos=np.degrees(self.pos),
            vel=np.degrees(self.vel),
            effort=self.effort
        )