# CGCalibrate.py
#   Created by: Adrien Hobelman
#   For Capstone Team ODIN
#   Date: 10/22/2025
#       Initial Creation. This set of functions is the logic to utilize CGMove() functions for determining the CG

import numpy as np
import CGLogic as cg
# import Sensor here for real
from globals.globalVars import G


def Calibrate():
    XYCal()
    ZCal()


def XYCal(tol=(0.06*np.pi/180)):
    # ADD MOTOR SHAKE HERE TO SETTLE GYRO BEARING #
    # Sensor Grab Function Here
    
    state_x = {'i_sum': 0.0, 'prev_err': 0.0}
    state_y = {'i_sum': 0.0, 'prev_err': 0.0}
    dt = 1  # Due to this process being iterative and not time based, dt is set to 1 for the derivative time delta
    iter = 0
    while abs(G.sensor.roll) > tol or abs(G.sensor.pitch) > tol:
        iter += 1
        ux, state_x = pid_controller(G.sensor.pitch, state_x, dt, G.statics.Kx_CG)
        uy, state_y = pid_controller(G.sensor.roll, state_y, dt, G.statics.Ky_CG)
        
        dr = np.vstack([ux, -uy, 0.0])
        cg.CG(dr)
        
        # ADD MOTOR SHAKE HERE TO SETTLE GYRO BEARING #
        #YPRgrabTest
        # Sensor Grab Function Here
    #print('This run took ', iter, ' PID attempts to calibrate.\n\n')
    G.disp.r_CG = np.vstack([0, 0, 0])  # X and Y calibrated to 0s, Z = 0 is a placeholder until ZCal()
    return


def ZCal():
    dr_CG = np.vstack([1, 0, 0])    # Sets dr_CG to move X forward 1 mm to compute Z-Axis CG component
    cg.CG(dr_CG)

    # ADD MOTOR SHAKE HERE TO SETTLE GYRO BEARING #
    # Update sensor data for pitch to compute Z
    # Sensor Grab Function Here
    
    r_CG_z = 1 / np.tan(G.sensor.pitch)  # Might need to be tangent, not cotangent, not sure on how the VN200s will report angle to gravity vector.
    G.disp.r_CG = np.vstack([1, 0, r_CG_z])
    cg.CG(np.vstack([-1, 0, 0]))    # Moving CG back to <0, 0, Z>
    return


def pid_controller(err, state, dt, K):
    Kp, Ki, Kd = K.flatten()
    i = state.get('i_sum', 0.0) + Ki * err * dt
    d = Kd * (err - state.get('prev_err', 0.0)) / dt
    u = Kp * err + i + d
    state['i_sum'] = i
    state['prev_err'] = err
    return u, state


# To simulate grabbing YPR from the VN200s for simulation:
def YPRgrabTest():
    x, y, z = G.testVals.r_CG_real.flatten()
    
    G.sensor.roll  = -np.arctan2(-y, -z)       # left offset → negative roll
    G.sensor.pitch = -np.arctan2(x, -z)      # forward     → negative pitch
    G.sensor.yaw   = 0.0                    # Without second sensor measurement (camera), not determinable
