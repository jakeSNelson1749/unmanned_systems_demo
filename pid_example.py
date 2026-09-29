""""
Recipe for PID
- set position 
- calculate error
- feed error into PID
- send command to Quadcopter
- repeat
"""
import numpy as np

from unmanned_systems_basics.quadcopter import Quadcopter

def compute_error(actual: float, desired:float) -> float:
    return desired - actual
def pid_calculate(current_error:float, previous_error:float, dt:float, min_val:float, max_val:float) -> float:
    """_summary_

    Args:
        current_error (float): _description_
        previous_error (float): _description_

    Returns:
        float: _description_
        
        
        # Proportional Error
        # Integral Error
        # Derivative Error
    """
    
    kp:float = 0.5
    ki:float = 0.0
    kd:float = 0.0
    
    prop_error:float = kp*current_error
    integral_error:float = ki*((previous_error+current_error)/2)* dt
    deriv_error: float = kd*((current_error - previous_error)/2) *dt
    
    command = (prop_error + integral_error + deriv_error)
    
    return np.clip(command, min_val, max_val)

# COORDINATE FRAME IS NED

vehicle = Quadcopter("udp:127.0.0.1:14551")
state = vehicle.update()

desired_x_ned = 0.0
current_x = 0.0

prev_error = 0.0

# compute error
error = compute_error(actual=current_x, desired=desired_x_ned)

print("error",error)

while True:
    error = compute_error(actual=current_x, desired=desired_x_ned)
    command_vel_x = pid_calculate(
        current_error=error,
        previous_error=prev_error,
        dt=0.05,
        min_val=-3.0,
        max_val=3.0
    )
    
    print("command: ",command_vel_x)
    #command_drone()
    vehicle.command_velocity_ned(
        vn_m_s=command_vel_x, 
        ve_m_s=0, 
        vd_m_s=0
    )
    state = vehicle.update()
    current_x = state.north_m
    print("current_x", current_x)
