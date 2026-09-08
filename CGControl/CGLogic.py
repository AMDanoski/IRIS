# CGMove.py
#   Created by: Adrien Hobelman
#   For Capstone Team ODIN
#   Date: 10/16/2025
#
#   Date: 10/22/2025 Update 1
#       - Separated Jacobian and null motion vector derivation logic into Jacobian()
#       - Added a basic function for taking a desired s_1 and computing the new r_CG_1 given a known r_CG_0

import numpy as np
from scipy.linalg import null_space
from globals.globalVars import G

# Call CG() with inputs from globalVars:
#   
#   dr_CG = 3x1 np.array() column matrix of requested CG translation
#       dr_CG(0) = dx
#       dr_CG(y) = dy
#       dr_CG(2) = dz
#   s_0 = 4x1 np.array() column matrix of current linear actuator positions
#       s_0(0) = Bottom Actuator
#       s_0(1) = Rear Top Actuator
#       s_0(2) = Forward Left Top Actuator
#       s_0(3) = Forward Right Top Actuator
#
# 
# Ensure to update globalVars with:
#
#   ds = 4x1 np.array() column matrix of valid linear actuator movement for solution
#       ds(0) = Bottom Actuator
#       ds(1) = Rear Top Actuator
#       ds(2) = Forward Left Top Actuator
#       ds(3) = Forward Right Top Actuator
#   s_1 = 4x1 np.array() column matrix of new linear actuator positions after ds is applied
#       s_1(0) = Bottom Actuator
#       s_1(1) = Rear Top Actuator
#       s_1(2) = Forward Left Top Actuator
#       s_1(3) = Forward Right Top Actuator
#
# 
# This function takes a dr_CG and s_0 to compute a ds and s_1 utilizing the pseudoinverse of the Jacobian of the linear system.
# It has a built in check for impossible solutions with a linear actuator selector. Null motion injection is also employed to 
# ensure a minimum MMOI for the Chassis at that specific r_CG solution. This is hardcoded for our specific linear actuator placement.

def CG(dr_CG_0):
    # bounds selector flags
    a = b = c = d = 1
    J = G.statics.J

    # Ensures no infinite looping in the event of impossible solution
    max_iters = np.size(G.disp.s_linAct)
    for _ in range(max_iters):
        active_flags = np.array([a, b, c, d], dtype=bool)   # Sets active linear actuators in solution
        active_idxs  = np.nonzero(active_flags)[0]          # Active linear actuator indexes for 0 vector return on line 65

        # Slice active columns
        J_active = J[:, active_idxs]

        # Pseudoinverse gives
        J_pinv = np.linalg.pinv(J_active, rcond=1e-12)  # adjust rcond to taste

        # ds for active actuators
        ds_active = J_pinv @ dr_CG_0

        # expand to full (4 x 1) for output
        ds_full = np.zeros((4,1))
        ds_full[active_idxs, 0] = ds_active.ravel()

        # New total displacements for bounds checks
        s_1 = G.disp.s_linAct + ds_full

        # Null Motion Injection
        ds_null, s_1_null = null_motion_injection(ds_full, s_1)
        
        ds_null = two_decimal_round(ds_null)
        s_1_null = two_decimal_round(s_1_null)  # Rounded to 2 decimals for floating point error removal

        # Checks for bounds violations and sets flag for removal of invalid linear actuator/s from next iteration
        viol = errorCheck(s_1_null)

        dr_CG_null = np.asarray(J @ ds_null).ravel()    # 1-D output representing null minimized achieved dr from solution
        dr_CG_req = np.asarray(dr_CG_0).ravel()         # 1-D output representing dr from input for comparison

        # Checks that the current valid solution returns the input requested solution
        if np.allclose(dr_CG_null, dr_CG_req, atol=1e-3, rtol=1e-2) and not np.any(viol):
            G.disp.s_linAct = s_1_null
            G.disp.r_CG = G.disp.r_CG + dr_CG_0         # Update global vars with valid solution
            G.testVals.r_CG_real = G.testVals.r_CG_real + dr_CG_0       # Updates simulation data for comparison
            
            # ********************* RESERVED FOR TRANSLATION LOGIC TO COMMAND MOTORS HERE *********************
            # ds_null for motor
            return

        # Disables any actuators that violate bounds
        if viol[0]: a = 0
        if viol[1]: b = 0
        if viol[2]: c = 0
        if viol[3]: d = 0

    # If the solution is invalid, return 0s
    print("No Solution Available For dr_CG: \n", dr_CG_0)
    return

def linAct(s_1):
    # bounds check
    viol = errorCheck(s_1)
    
    if np.any(viol) == True:
        print("Invalid s_1 input, try again.")
        return
    
    ds = s_1 - G.disp.s_linAct      # Computes the delta for each actuator
    dr_CG = G.statics.J @ ds        # Computes the relative change in CG

    G.disp.s_linAct = s_1               # Stores new displacements as global var
    G.disp.r_CG = G.disp.r_CG + dr_CG   # Computes the new CG for output and updates global var
    G.testVals.r_CG_real = G.testVals.r_CG_real + dr_CG     # Purely for simulation, computes the new real CG for comparison later

    # ********************* RESERVED FOR TRANSLATION LOGIC TO COMMAND MOTORS HERE *********************
    # ds for motor
    
    return
    
def errorCheck(s_1):
    # Checks for bounds violations and sets flag for removal of invalid linear actuator from next iteration
    viol_low  = (s_1 < G.statics.lows).ravel()
    viol_high = (s_1 > G.statics.highs).ravel()
    viol = np.logical_or(viol_low, viol_high)
    return viol

def two_decimal_round(val):
    return np.round(val*100) / 100


def null_motion_injection(ds, s_0):
    v = G.statics.v
    t = np.zeros([4, 1])    # For s_0 minimization: 0 = s_0 + t*v

    for i in range(np.size(s_0)):   # Computes t multipliers for each linear actuator
        t[i] = s_0[i] / v[i]

    s_1 = s_0 - np.min(t)*v     # Takes the minimum t and multiplies all null space vector elements to compute minim s and ds
    ds_1 = ds - np.min(t)*v
    return ds_1, s_1    # Returns minimized ds and s