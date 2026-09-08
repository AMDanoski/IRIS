# ODIN
This is the repository for ODIN code

-----

How To Use Global Vars

Under globals/globalVars you can declare variables that are update-able across all scripts. To use them in a script the following header needs to be included:

from globals.globalVars import G

As long as __init__.py is in the same folder as the script, it can access the globalVars. If you are using it in a custom test script, you need to include an additional few lines to set the system path assuming your test script is in a folder:

import sys, pathlib
if __package__ is None or __package__ == "":
    sys.path.append(str(pathlib.Path(__file__).resolve().parents[1]))

Check CGControl/testCase.py for an example of the code integrated. Notice very few of my function call require explicit inputs and outputs outside of local logic. All of the variables are stored in global to keep them updated for use anywhere else in the code without needing to treat them as inputs.

Use the globalVars like a data structure in MATLAB. For instance, to read r_CG for the current CG position, you would call G.disp.r_CG which extracts from the global class, the disp class, the r_CG value that is stored. You can also update these values whenever necessary like a normal variable. So, G.disp.r_CG = np.array([1,0,0]) would update r_CG in the Global() class to 1,0,0 for anyone else to then import and use.

Any variables you would like to add need to be added in globals/globalVars. Use the same structure as globalVars for the values to bbe readable. If you would like to add a new class under class Global, it is as follows:

class Globals:
    class newClassName:
        # These values are all for test simulations, these are not to be used in the code for actual functions, only to be referenced
        def __init__(self):
            self.var = numbers go here

    ...

    def __init__(self):
        self.newClassName = Globals.newClassName()

With this structure you can add your own addressable classes. This would be readable from another script as G.newClassName.var as whatever you assigned. If you want to update them in a script:

G.newClassName.var = 15 would update var to 15 and be readable as such by any other script.\

-----

ALL AXES ARE AS FOLLOWS:

- +X is in the camera direction
- +Y is to the left, port side of the chassis
- +Z is upwards from center of chassis pivot

Note: All values are measured in mm for CG control


Linear Actuator Vector:

s_n = np.array([[BOTTOM]
              [REAR TOP]
              [+Y FRONT TOP]
              [-Y FRONT TOP]])


MIN and MAX stroke are:

s_n(0) = 0 - 50
s_n(1) = 0 - 250
s_n(2) = 0 - 250
s_n(3) = 0 - 250


r_CG (or dr_CG) vector:

r_CG = np.array([[+X]
                 [+Y]
                 [+Z]])


For CG Logic and Movement:
Layer cake methodology is used.

Top Layer is the Calibration for the CG in CGCalibrate.py.
Middle Layer is CGMove.py for computing the amount to move the motors given a change in r_CG or desired new s vector
Bottom Layer should be the motor control functions, give it a ds and move the motors on that ds vector.

For CG calibration, the X and Y is first calibrated using the VN200s to zero out roll and pitch angles to the gravity vector. Once this is accomplished it can be reasonably assumed that X and Y for the r_CG are 0. To determine the Z position, the r_CG is moved 1 mm in the +X direction. The tangent of the new pitch angle is equal to the Z in r_CG. The ZCal() then moves the CG to [0; 0; Z]. We'll keep the bottom actuator at the bottom of its stroke (max -Z), in order to keep the chassis stable along the gravity vector during gimbal and servo spin up in case any out-of-phase rotational acceleration is incurred during spin up.

-----

FILE NAME: DATE : FUNCTION: PERSON TO BLAME IF SOMETHING GOES WRONG.


OAK D S-2, ODIN ML : 10/15/2025: John P. Anderson


 
TrackerV1          : 10/17/2025: John P. Anderson




MotorControl Functions  : 10/22/2025: Andrew J. Reynolds

