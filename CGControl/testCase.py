import sys, pathlib
if __package__ is None or __package__ == "":
    sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))

from globals.globalVars import G
import numpy as np
from CGControl import CGCalibrate as cal
from CGControl import CGLogic as cg

cal.Calibrate()

print('Final s_1: \n', G.disp.s_linAct)
print('r_CG output from calibration: \n', G.disp.r_CG, '\n')
print('Simulated real r_CG for comparison: \n', G.testVals.r_CG_real)

# Moving CG to -0.1 mm in the Z
dr = np.vstack([0, 0, -0.1]) - G.disp.r_CG
cg.CG(dr)

#analytical()

def analytical():
    print('\n')
    
    s_1 = G.disp.s_linAct   # Store the current solution
    G.disp.s_linAct = np.vstack([0, 0, 0, 0])   # Reset linear actuators to 0s
    G.disp.r_CG = np.vstack([3.163, -0.248, -0.921])    # Reset r_CG to original r_CG_real

    cg.linAct(s_1)  # Apply solution from calibration

    print('r_CG from analytical solution: \n', G.disp.r_CG)