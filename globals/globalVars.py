import numpy as np
from . import Jacobian_init as Ja

class Globals:
    class testVals:
        # These values are all for test simulations, these are not to be used in the code for actual functions, only to be referenced
        def __init__(self):
            self.r_CG_real = np.vstack([3.163, -0.248, -0.921])

    class sensor:
        def __init__(self):
            
            # VN200 Outputs that should be updated once an update function is run
            self.yaw   = 0.0
            self.pitch = 0.0
            self.roll  = 0.0
            self.quat  = np.vstack([0, 0, 0, 0])

    class disp:
        def __init__(self):
            
            # Current Linear Actuator Positions
            self.s_linAct = np.vstack([0, 0, 0, 0])
            
            # Current r_CG (Initializes as 0s until Calibrate() in CGCalibrate.py is run)
            self.r_CG     = np.vstack([0, 0, 0])

    class statics:
        def __init__(self):
            
            # The following static values are for linear actuator limits and their Jacobian + nullspace vector
            self.J, self.v = Ja.Jacobian()
            self.lows  = np.vstack([0, 0, 0, 0])
            self.highs = np.vstack([50, 250, 250, 250])
            
            # PID weights for CG calibration
            self.Kx_CG = np.vstack([0.9, 0.0001, 0.05])
            self.Ky_CG = np.vstack([0.9, 0.0001, 0.05])

    def __init__(self):
        # Initialization of sub-classes within Globals()
        self.testVals = Globals.testVals()
        self.sensor   = Globals.sensor()
        self.disp     = Globals.disp()
        self.statics  = Globals.statics()

G = Globals()