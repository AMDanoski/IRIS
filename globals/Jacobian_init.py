import numpy as np
from scipy.linalg import null_space

def Jacobian():
    m_1     = 1.06      # Mass of bottom mass in kg
    m_2     = 0.9       # Mass of rear top mass in kg
    m_3     = 0.9       # Mass of forward left top mass in kg
    m_4     = 0.9       # Mass of forward right top mass in kg
    m_total     = 10.0  # Mass of total system with linear actuator masses in kg (Currently Placeholder Until Built and Weighed)

    phi   = 60.0 * (np.pi/180.0)    # Angle of upper linear actuators from +Z
    theta = 15.0 * (np.pi/180.0)    # Angle of forward upper linear actuators from +X

    # Bottom Linear Actuator Unit Vector
    u1 = np.vstack([
        0.0,
        0.0,
        -1.0
        ])

    # Upper Rear Linear Actuator Unit Vector
    u2 = np.vstack([
        -np.sin(phi),
        0.0,
        np.cos(phi)
    ])

    # Upper Forward Left Linear Actuator Unit Vector
    u3 = np.vstack([
        np.cos(theta)*np.sin(phi),
        np.sin(theta)*np.sin(phi),
        np.cos(phi)
    ])

    # Upper Forward Right Linear Actuator Unit Vector
    u4 = np.vstack([
        np.cos(theta)*np.sin(phi),
        -np.sin(theta)*np.sin(phi),
        np.cos(phi)
    ])

    # J should be 3x4: columns correspond to actuators
    J = (1.0/m_total) * np.hstack([
        m_1     * u1,
        m_2     * u2,
        m_3     * u3,
        m_4     * u4
    ])

    v = null_space(J)   # Null Motion Vector for J
    
    return J, v