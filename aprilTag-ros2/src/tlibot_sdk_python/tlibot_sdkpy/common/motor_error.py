from enum import IntEnum, IntFlag
from dataclasses import dataclass


class MotorError(IntFlag):
    OK = 0
    
    DRIVER_OVER_VOLTAGE      = 1 << 1
    DRIVER_UNDER_VOLTAGE     = 1 << 4
    MOS_OVER_CURRENT         = 1 << 5
    VEL_FEEDBACK_ERROR       = 1 << 6
    MCU_INTERNAL_ERROR       = 1 << 7
    DRIVER_CURRENT_ERROR     = 1 << 8
    
    MOTOR_OVER_TEMPERATURE   = 1 << 9
    DRV_OVER_TEMPERATURE     = 1 << 10
    MOTOR_ERROR_TEMPERATURE   = 1 << 11
    DRV_ERROR_TEMPERATURE     = 1 << 12
    
    ENCODER_ERROR            = 1 << 13
    EEROM_ERROR              = 1 << 14

    PARAM_SET_ERROR          = 1 << 15
    

ERROR_INFO = {
    MotorError.DRIVER_OVER_VOLTAGE: "Driver over voltage",
    MotorError.DRIVER_UNDER_VOLTAGE: "Driver under voltage",
    MotorError.MOS_OVER_CURRENT: "MOS over current",
    MotorError.VEL_FEEDBACK_ERROR: "Velocity feedback error",
    MotorError.MCU_INTERNAL_ERROR: "MCU internal error",
    MotorError.DRIVER_CURRENT_ERROR: "Driver current error",
    MotorError.MOTOR_OVER_TEMPERATURE: "Motor over temperature",
    MotorError.DRV_OVER_TEMPERATURE: "Driver over temperature",
    MotorError.MOTOR_ERROR_TEMPERATURE: "Motor error temperature",
    MotorError.DRV_ERROR_TEMPERATURE: "Driver error temperature",            
    MotorError.ENCODER_ERROR: "Encoder communication failed",
    MotorError.EEROM_ERROR: "EEROM error",
    MotorError.PARAM_SET_ERROR: "Driver Parameter set error",
}

def error_infos(mask: int) -> list[str]:
    errs = []

    for err, name in ERROR_INFO.items():
        if mask & err:
            errs.append(name)

    return errs