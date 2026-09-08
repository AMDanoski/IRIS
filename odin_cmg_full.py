#!/usr/bin/env python3
# ==============================================================
#  ODIN CMG CONTROLLER — PID + Anti-Chatter + Gated Nullspace Bias
#  + Singularity-Robust Damped LS (SR-DLS) + Weighted pinv + α-scaling
#  (with direction-aware debug output)
# ==============================================================

import numpy as np
import matplotlib.pyplot as plt

# ==============================================================
#                      CONFIGURATION SECTION
# ==============================================================

# --- Spacecraft Inertia Tensor (kg·m²)
I_body = np.array([
    [0.239,       -9.689e-4,  -0.010],
    [-9.689e-4,    0.315,       4.905e-5],
    [-0.010,       4.905e-5,    0.435]
], float)
Iinv = np.linalg.inv(I_body)

# --- Wheel properties
wheel_MOI  = 5.5e-5
wheel_rpm  = 4500.0
omega_spin = wheel_rpm * 2*np.pi/60.0
h_vec      = np.ones(4) * (wheel_MOI * omega_spin)  # [N·m·s]

# --- PID Gains (attitude error & rate error)
# --- PID Gains (attitude error & rate error)
Kp = np.diag([1.6, 1.6, 1.6])          # was 3.0
Kd = np.diag([3.2, 3.2, 3.2])          # was 1.8  (≈ +45%)
Ki = np.diag([0.02, 0.02, 0.015])       # was 0.25/0.25/0.15  (slow integral)

Iterm_limit = np.deg2rad(np.array([10,10,10]))  # anti-windup clamp (input:deg output:rad)

# Enable/disable integral action
USE_I = True

# --- Limits and timing
gimbal_rate_limit = np.deg2rad(np.array([540, 540, 540, 540]))   # |γ̇| limit [input:deg/s output:rad/s]
dt_default = 0.01                                                # 100 Hz

# --- SR-DLS / pinv knobs
SRDLS_ENABLE = True
SRDLS_SIGMA_C = 0.05          # crossover for λ(σ_min)
SRDLS_LAM_MAX_FACTOR = 1.0    # λ_max = factor * σ_max_nom
PINV_WEIGHTED = True          # use W=diag(1/γ̇_max^2)

# --- Allocator anti-chatter tuning (primary knobs)
CONTINUITY_BIAS   = 0.45     # μ: stick to previous γ̇ (0.1–0.4 typical)

# --- Additional smoothers (secondary knobs)
GIMBAL_SLEW_LIMIT    = np.deg2rad(4000.0)  # max per-step Δγ̇ [rad/s^2] * dt
GIMBAL_RATE_DEADBAND = np.deg2rad(1.25)     # small rates below this go to 0 (set 0 to disable)
CMD_SMOOTH_ALPHA     = None                # optional EMA on γ̇ (e.g., 0.2); None=off
TAU_CMD_ALPHA        = None                # EMA on task-space τ_cmd used by allocator (None=off)

# --- GATED Null-space bias (prevents γ windmilling in low-torque only)
NULLSPACE_BIAS_ENABLE = True
NULLSPACE_GAIN        = 0.005              # k_b in rad/s toward gamma_home (scale by gate)
gamma_home_deg        = np.array([45, 135, 225, 315])  # preferred posture
gamma_home            = np.deg2rad(gamma_home_deg)
NULLSPACE_TAU_THRESH  = 0.25               # [N·m] bias fades out as |tau_cmd| rises to this
LAMBDA_ADAPT = True
LAMBDA_BASE  = 0.01

# --- Demo Yaw Maneuver
DEMO_TOTAL_DEG = 300.0
DEMO_VMAX_DEG  = 60.0
DEMO_ACC_DEG   = 120.0

# ==============================================================

# ------------------- Math Helpers -------------------
def hat(v):
    x,y,z = v
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]],float)

def q_mul(q1,q2):
    w1,x1,y1,z1=q1; w2,x2,y2,z2=q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ],float)

def q_conj(q): return np.array([q[0],-q[1],-q[2],-q[3]],float)
def q_inv(q):  return q_conj(q)/np.dot(q,q)
def q_norm(q): return q/np.linalg.norm(q)

def compose_body_target(q_cur, q_target_body):
    """Convert a body-frame target quaternion into an absolute I→B target."""
    return q_norm(q_mul(np.asarray(q_cur,float), np.asarray(q_target_body,float)))

def small_angle_vec(qe):
    if qe[0] < 0: qe = -qe
    return 2*qe[1:4]

def omega_from_q_qdot(q,qdot):
    q0,qv=q[0],q[1:4]
    G=np.vstack([-qv,q0*np.eye(3)+hat(qv)]).T
    return 2*(G@qdot)

def qdot_from_q_omega(q,w):
    wx,wy,wz=w
    Om=np.array([
        [0,-wx,-wy,-wz],
        [wx, 0, wz,-wy],
        [wy,-wz, 0, wx],
        [wz, wy,-wx, 0]
    ],float)
    return 0.5*(Om@q)

def quat_to_euler(q):
    w, x, y, z = q
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)]
    ], float)
    yaw   = np.arctan2(R[1,0], R[0,0])
    pitch = -np.arcsin(np.clip(R[2,0], -1.0, 1.0))
    roll  = np.arctan2(R[2,1], R[2,2])
    return np.array([yaw, pitch, roll], float)

def euler_to_quat(yaw, pitch, roll):
    cy, sy = np.cos(yaw/2),   np.sin(yaw/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cr, sr = np.cos(roll/2),  np.sin(roll/2)
    # Z (yaw) – Y (pitch) – X (roll) intrinsic
    w = cr*cp*cy + sr*sp*sy
    x = sr*cp*cy - cr*sp*sy
    y = cr*sp*cy + sr*cp*sy 
    z = cr*cp*sy - sr*sp*cy
    return q_norm(np.array([w,x,y,z], float))

def wrap_to_pi(x):
    return (x + np.pi) % (2*np.pi) - np.pi

def unit(v, eps=1e-12):
    n = float(np.linalg.norm(v))
    return v / (n + eps)

def ang_deg(a, b, eps=1e-12):
    ua, ub = unit(a, eps), unit(b, eps)
    c = np.clip(np.dot(ua, ub), -1.0, 1.0)
    return np.rad2deg(np.arccos(c))

# ------------------- Geometry -------------------
def Rx(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[1, 0, 0],
                     [0, c,-s],
                     [0, s, c]], float)

def Rz(th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[ c,-s, 0],
                     [ s, c, 0],
                     [ 0, 0, 1]], float)

# Body axes convention: x forward, y left, z up (right-handed)

# Base (γ-independent) rotations for each CMG:
R_BASE = [
    Rx(np.deg2rad(-45.0)) @ Rz(np.deg2rad(180.0)),  # CMG 1
    Rx(np.deg2rad(+45.0)) @ Rz(0.0),                # CMG 2
    Rx(np.deg2rad(-45.0)) @ Rz(np.deg2rad(180.0)),  # CMG 3
    Rx(np.deg2rad(+45.0)) @ Rz(0.0)                 # CMG 4
]

# In the local gimbal frame:
# - The gimbal axis is êz (the axis of Rz)
# - The wheel spin axis is êx
e_x = np.array([1.0, 0.0, 0.0])
e_z = np.array([0.0, 0.0, 1.0])

def spin_axis_body(i, gamma_i):
    return R_BASE[i] @ (Rz(gamma_i) @ e_x)

def gimbal_axis_body(i):
    return R_BASE[i] @ e_z

def cmg_jacobian(gamma):
    J = np.zeros((3, 4), float)
    for i in range(4):
        s_i   = gimbal_axis_body(i)
        spin  = spin_axis_body(i, gamma[i])
        J[:, i] = h_vec[i] * np.cross(s_i, spin)
    return J

# ------------------- PID Controller Core -------------------
_int_error = np.zeros(3)

def integrator_update(e_rot, dt):
    """Accumulate and clamp integral state so that Ki * int_error <= Iterm_limit (per axis)."""
    global _int_error
    _int_error = _int_error + e_rot * dt
    # Clamp based on Ki diagonal so that the CONTRIBUTION stays within Iterm_limit
    ki_diag = np.diag(Ki).astype(float)
    # Avoid divide-by-zero if any Ki diag is 0
    ki_safe = np.where(ki_diag > 1e-12, ki_diag, 1e-12)
    max_int_err = Iterm_limit / ki_safe
    _int_error = np.clip(_int_error, -max_int_err, max_int_err)
    return _int_error

def pid_torque(q_des, q_cur, w_des, w_cur, dt):
    # ensure same hemisphere before computing the error
    q_err = q_mul(q_conj(q_cur), q_des)   # body-frame error
    # force shortest-arc
    if q_err[0] < 0.0:
        q_err = -q_err
    e_rot = 2.0 * q_err[1:4]
    e_w   = (w_des - w_cur)               # body angular-rate error

    # your gains (diag or full matrices)
    tau_p = Kp @ e_rot
    tau_d = Kd @ e_w
    tau_i = Ki @ integrator_update(e_rot, dt) if USE_I else 0.0

    return tau_p + tau_d + tau_i

# ------------------- SR-DLS helpers -------------------
_smax_nom = None

def _lambda_adaptive(smin, smax):
    if not SRDLS_ENABLE:
        return 0.0
    global _smax_nom
    if _smax_nom is None:
        _smax_nom = max(smax, 1e-9)
    lam_max = SRDLS_LAM_MAX_FACTOR * _smax_nom
    sc = SRDLS_SIGMA_C
    return lam_max * (sc*sc) / (smin*smin + sc*sc)

def _pinv_damped_weighted(J, lam, W=None):
    if W is None:
        A = J @ J.T + (lam*lam) * np.eye(3)
        return J.T @ np.linalg.solve(A, np.eye(3))
    A = J @ W @ J.T + (lam*lam) * np.eye(3)
    return W @ J.T @ np.linalg.solve(A, np.eye(3))

def _bounded_ls(J, tau, lo, hi, lam=0.0, g_prev=None, mu=0.0):
    n = J.shape[1]; g = np.zeros(n) if g_prev is None else g_prev.copy()
    F = np.ones(n,bool)
    for _ in range(12):
        JF = J[:,F]
        rhs = tau - (J[:,~F]@g[~F]) if (~F).any() else tau
        A = JF.T@JF; b = JF.T@rhs
        if lam>0: A += (lam**2)*np.eye(A.shape[0])
        if mu>0 and g_prev is not None and JF.shape[1]>0:
            A += mu*np.eye(A.shape[0]); b += mu*g_prev[F]
        g[F] = np.linalg.solve(A,b) if A.size else np.zeros(0)
        violated = np.zeros(n,bool)
        for i in np.where(F)[0]:
            if g[i] < lo[i]: g[i] = lo[i]; violated[i] = True
            elif g[i] > hi[i]: g[i] = hi[i]; violated[i] = True
        if not violated.any(): break
        F[violated] = False
    return g

def _apply_slew_limit(gdot, gdot_prev, dt):
    if gdot_prev is None: return gdot
    max_delta = GIMBAL_SLEW_LIMIT * dt
    return np.clip(gdot, gdot_prev - max_delta, gdot_prev + max_delta)

def _apply_deadband(gdot):
    if GIMBAL_RATE_DEADBAND <= 0.0:
        return gdot
    mag = np.abs(gdot)
    gdot = gdot.copy()
    gdot[mag < GIMBAL_RATE_DEADBAND] = 0.0
    return gdot

# persistent state
_gdot_prev = None
_tau_used_prev = None

def cmg_gimbal_rates(q_cur, qdot_cur, q_des, qdot_des, gamma, dt):
    global _gdot_prev, _tau_used_prev, _smax_nom

    # 1) Task-space PID torque
    w_cur = omega_from_q_qdot(q_cur, qdot_cur)
    w_des = omega_from_q_qdot(q_des, qdot_des)
    tau_cmd_raw = pid_torque(q_des, q_cur, w_des, w_cur, dt)

    # 2) Optional EMA on commanded torque
    if TAU_CMD_ALPHA is not None:
        a = float(TAU_CMD_ALPHA)
        if _tau_used_prev is None:
            _tau_used_prev = tau_cmd_raw.copy()
        tau_cmd_used = a * _tau_used_prev + (1.0 - a) * tau_cmd_raw
        _tau_used_prev = tau_cmd_used.copy()
    else:
        tau_cmd_used = tau_cmd_raw

    # 3) Jacobian, SVD, torque budget
    J = cmg_jacobian(gamma)
    U, S, Vt = np.linalg.svd(J, full_matrices=False)
    smax = S[0] if S.size else 0.0
    smin = S[-1] if S.size else 0.0
    if _smax_nom is None:
        _smax_nom = max(smax, 1e-9)

    # 4) α feasibility scaling
    tau_budget = smax * np.linalg.norm(gimbal_rate_limit, 2)
    nrm = float(np.linalg.norm(tau_cmd_used))
    alpha = 1.0 if nrm <= 1e-12 else min(1.0, tau_budget / (nrm + 1e-12))
    tau_cmd_used = alpha * tau_cmd_used
    nrm = float(np.linalg.norm(tau_cmd_used))

    # 5) Adaptive damping λ and continuity penalty μ
    lam = _lambda_adaptive(smin, smax)
    mu = CONTINUITY_BIAS

    # 6) LS bounds and SVD seed
    lo, hi = -gimbal_rate_limit, gimbal_rate_limit
    S_inv = np.diag([1/s if s > 1e-6 else 0.0 for s in S])
    gdot_seed = Vt.T @ (S_inv @ (U.T @ tau_cmd_used))

    # 7) Active-set SR-DLS solve in rate space
    gdot = _bounded_ls(J, tau_cmd_used, lo, hi, lam=lam, g_prev=gdot_seed, mu=mu)
    taua = J @ gdot

    # 8) High-torque fallback
    tau_frac = nrm / (tau_budget + 1e-12) if tau_budget > 0 else 0.0
    if (tau_frac > 0.8) and (np.linalg.norm(taua) < 0.2 * np.linalg.norm(tau_cmd_used) + 1e-12):
        g_align = J.T @ tau_cmd_used
        gdot_fb = np.clip(gimbal_rate_limit * np.sign(g_align), -gimbal_rate_limit, gimbal_rate_limit)
        gdot = gdot_fb
        taua = J @ gdot

    # 9) Weighted damped pinv + nullspace projector
    W = np.diag(1.0 / (gimbal_rate_limit**2)) if PINV_WEIGHTED else None
    J_pinv = _pinv_damped_weighted(J, lam, W=W)
    N = np.eye(4) - J_pinv @ J

    # 10) GATED null-space bias toward gamma_home
    if NULLSPACE_BIAS_ENABLE:
        tau_mag = np.linalg.norm(tau_cmd_used)
        gate = np.clip((NULLSPACE_TAU_THRESH - tau_mag) / NULLSPACE_TAU_THRESH, 0.0, 1.0)
        if gate > 0.0:
            err_gamma = wrap_to_pi(gamma - gamma_home)
            z = -NULLSPACE_GAIN * gate * err_gamma
            gdot += N @ z
            taua = J @ gdot

    # 11) Singularity-robust assist
    s_thresh = 0.15
    K_ns = 0.50
    if smin < s_thresh and tau_frac > 0.2:
        gain = K_ns * (0.2 + 0.8 * tau_frac)
        assist = N @ (J.T @ tau_cmd_used)
        a_norm = np.linalg.norm(assist) + 1e-12
        gdot += gain * (assist / a_norm) * (s_thresh - smin) / s_thresh
        taua = J @ gdot

    # 12) Rate slew limit, deadband, optional EMA on γ̇
    gdot = _apply_slew_limit(gdot, _gdot_prev, dt)
    gdot = _apply_deadband(gdot)
    if CMD_SMOOTH_ALPHA is not None:
        a = float(CMD_SMOOTH_ALPHA)
        if _gdot_prev is None:
            _gdot_prev = gdot.copy()
        gdot = a * _gdot_prev + (1.0 - a) * gdot
    _gdot_prev = gdot.copy()

    # --- New: direction & angle diagnostics ---
    yaw_axis = np.array([0.0, 0.0, 1.0])
    tau_cmd_dir = unit(tau_cmd_used)
    tau_act_dir = unit(taua)
    ang_cmd_vs_act = ang_deg(tau_cmd_used, taua)
    ang_cmd_vs_yaw = ang_deg(tau_cmd_used, yaw_axis)
    ang_act_vs_yaw = ang_deg(taua,        yaw_axis)
    sat_count = int(np.sum(np.isclose(np.abs(gdot), gimbal_rate_limit, rtol=0, atol=1e-9)))

    dbg = {
        "tau_cmd": tau_cmd_used,
        "tau_act": taua,
        "tau_cmd_dir": tau_cmd_dir,
        "tau_act_dir": tau_act_dir,
        "ang_cmd_vs_act_deg": ang_cmd_vs_act,
        "ang_cmd_vs_yaw_deg": ang_cmd_vs_yaw,
        "ang_act_vs_yaw_deg": ang_act_vs_yaw,
        "sat_count": sat_count,
        "smin": smin,
        "smax": smax,
        "lambda": lam,
        "alpha": alpha,
        "J": J
    }

    return gdot, tau_cmd_raw, tau_cmd_used, J, dbg


# ==============================================================
#  Public interface for hardware integration
# ==============================================================

def odin_cmg_step(q_cur, qdot_cur, q_target_body, gamma):
    """
    Public interface (hardware-facing):
      - q_cur:          current attitude, I→B quaternion
      - qdot_cur:       current quaternion rate (compatible with body-rate convention)
      - q_target_body:  desired rotation expressed in the BODY frame (relative target)
      - gamma:          current gimbal angles (rad)

    Assumptions:
      - Target is ALWAYS given in the BODY frame.
      - Uses internal dt_default for integral/smoothing/slew math.
    """
    # Use fixed internal time step for controller math
    dt = dt_default

    q_cur   = np.asarray(q_cur, float)
    qdot_cur= np.asarray(qdot_cur, float)
    gamma   = np.asarray(gamma, float)

    # Convert body-frame target to absolute I→B target
    q_des_use = compose_body_target(q_cur, q_target_body)

    # For a body-frame pose target, use zero desired body rates
    w_des = np.zeros(3)
    qdot_des_use = qdot_from_q_omega(q_des_use, w_des)

    # Call allocator
    gdot_cmd, tau_raw, tau_cmd, J, dbg = cmg_gimbal_rates(
        q_cur, qdot_cur, q_des_use, qdot_des_use, gamma, dt
    )
    tau_act = J @ gdot_cmd
    dbg["tau_act"] = tau_act
    dbg["tau_act_dir"] = unit(tau_act)
    return gdot_cmd, tau_cmd, tau_act, dbg


# ------------------- Demo Simulation -------------------
def run_demo(
    euler_deg=(90.0, 0.0, 0.0),   # Desired BODY-FRAME rotation (yaw, pitch, roll) [deg]
    T=20.0,
    dt=dt_default,
    q0=None,
    w0=None,
    gamma0_deg=(30, 120, 210, 300),
    smooth_s=2.0,                 # seconds to ramp 0→target
    plot=True,
    print_hz=10
):
    """
    BODY-frame target simulation that converges to a fixed absolute attitude.
    - We lock a reference attitude q_ref at t=0.
    - The absolute desired is q_des_abs(t) = q_ref ⊗ slerp(I, q_target_body_final, a(t)).
    - To call CMG step (which composes q_cur ⊗ q_cmdB internally), we pass:
          q_cmdB(t) = q_cur^{-1} ⊗ q_des_abs(t)
      so that q_cur ⊗ q_cmdB = q_des_abs (fixed as t→∞).
    """

    # ---- Initial state ----
    q = q_norm(np.array(q0, float)) if q0 is not None else q_norm(np.array([1,0,0,0], float))  # I→B
    w = np.array(w0, float) if w0 is not None else np.zeros(3, float)
    gamma = np.deg2rad(np.array(gamma0_deg, float))
    q_ref = q.copy()  # lock the reference attitude at command start

    # ---- Time base ----
    t = np.arange(0.0, T + dt, dt)

    # ---- Final BODY-frame target quaternion from input Euler ----
    yaw_b, pitch_b, roll_b = np.deg2rad(np.array(euler_deg, float))
    q_target_body_final = euler_to_quat(yaw_b, pitch_b, roll_b)  # BODY-frame

    # Logs
    eul_log, eul_des_log, w_log = [], [], []
    tau_used_log, tau_act_log = [], []
    gdot_log, gam_log = [], []
    smin_log, smax_log, alpha_log, lam_log, sat_count_log = [], [], [], [], []

    # Print cadence
    print_dt = 1.0 / max(1, int(print_hz))
    next_print_t = 0.0

    for tt in t:
        # --- Smooth from identity (BODY) to final body target ---
        if smooth_s > 0.0 and tt < smooth_s:
            a = tt / smooth_s
            a = a*a*(3 - 2*a)  # smoothstep
            qa = np.array([1.0, 0.0, 0.0, 0.0])      # identity (BODY)
            dot = np.clip(np.dot(qa, q_target_body_final), -1.0, 1.0)
            qtb = q_target_body_final if dot >= 0 else -q_target_body_final
            if abs(dot) > 0.9995:
                q_target_body_s = q_norm((1-a)*qa + a*qtb)
            else:
                th = np.arccos(abs(dot))
                q_target_body_s = q_norm(
                    (np.sin((1-a)*th)/np.sin(th))*qa + (np.sin(a*th)/np.sin(th))*qtb
                )
        else:
            q_target_body_s = q_target_body_final

        # --- Fixed absolute desired attitude relative to q_ref ---
        q_des_abs = compose_body_target(q_ref, q_target_body_s)   # I→B (fixed as a→1)

        # --- Convert absolute target to a per-step BODY-frame command for odin_cmg_step ---
        q_cmd_body = q_mul(q_inv(q), q_des_abs)  # so odin_cmg_step's q_des_use = q ⊗ q_cmd_body = q_des_abs

        # Current quaternion derivative from current body rate
        qdot_cur = qdot_from_q_omega(q, w)

        # --- Controller/allocator (your updated body-target interface) ---
        # If your odin_cmg_step signature is (q_cur, qdot_cur, q_target_body, gamma):
        gdot, tau_cmd, tau_act, dbg = odin_cmg_step(q, qdot_cur, q_cmd_body, gamma)

        # If instead it still expects absolute q_des/qdot_des & dt, use this:
        # w_des = np.zeros(3)
        # qdot_des_abs = qdot_from_q_omega(q_des_abs, w_des)
        # gdot, tau_cmd, tau_act, dbg = odin_cmg_step(q, qdot_cur, q_des_abs, qdot_des_abs, gamma, dt)

        # --- Rigid body + gimbal integration ---
        wdot = Iinv @ (tau_act - np.cross(w, I_body @ w))
        w += wdot * dt
        q += qdot_from_q_omega(q, w) * dt
        q  = q_norm(q)
        gamma += gdot * dt

        # Logs
        eul_log.append(np.rad2deg(quat_to_euler(q)))
        eul_des_log.append(np.rad2deg(quat_to_euler(q_des_abs)))  # absolute desired (for plotting)
        w_log.append(np.rad2deg(w))
        tau_used_log.append(tau_cmd)
        tau_act_log.append(tau_act)
        gdot_log.append(np.rad2deg(gdot))
        gam_log.append(np.rad2deg(gamma))
        smin_log.append(dbg["smin"]); smax_log.append(dbg["smax"])
        alpha_log.append(dbg["alpha"]); lam_log.append(dbg["lambda"])
        sat_count_log.append(dbg["sat_count"])

        # Console print
        if tt >= next_print_t:
            next_print_t += print_dt
            ypr  = np.rad2deg(quat_to_euler(q))
            yprd = np.rad2deg(quat_to_euler(q_des_abs))
            tc = tau_cmd; ta = tau_act
            print(
              f"t={tt:5.2f}s | yaw={ypr[0]:7.2f}° (des {yprd[0]:7.2f}°) "
              f"pitch={ypr[1]:7.2f}° (des {yprd[1]:7.2f}°) "
              f"roll={ypr[2]:7.2f}° (des {yprd[2]:7.2f}°) | "
              f"τ_cmd=[{tc[0]:+.3f},{tc[1]:+.3f},{tc[2]:+.3f}] "
              f"τ_act=[{ta[0]:+.3f},{ta[1]:+.3f},{ta[2]:+.3f}] | "
              f"smin={dbg['smin']:.3e} smax={dbg['smax']:.3e} λ={dbg['lambda']:.4f} α={dbg['alpha']:.2f} "
              f"sat={dbg['sat_count']}/4"
            )

    # → arrays
    eul_log       = np.asarray(eul_log)
    eul_des_log   = np.asarray(eul_des_log)
    w_log         = np.asarray(w_log)
    tau_used_log  = np.asarray(tau_used_log)
    tau_act_log   = np.asarray(tau_act_log)
    gdot_log      = np.asarray(gdot_log)
    gam_log       = np.asarray(gam_log)
    smin_log      = np.asarray(smin_log)
    smax_log      = np.asarray(smax_log)
    alpha_log     = np.asarray(alpha_log)
    lam_log       = np.asarray(lam_log)
    sat_count_log = np.asarray(sat_count_log)

    # ---- Plots ----
    if plot:
        fig1, ax1 = plt.subplots(3,1, figsize=(10,8), sharex=True)
        fig1.suptitle("Euler Angles: Desired vs Actual", fontsize=14, fontweight="bold")
        labels = ["Yaw [deg]","Pitch [deg]","Roll [deg]"]
        for i in range(3):
            ax1[i].plot(t, eul_des_log[:,i], '--', label='Desired (abs)')
            ax1[i].plot(t, eul_log[:,i], label='Actual')
            ax1[i].set_ylabel(labels[i]); ax1[i].legend(); ax1[i].grid(True)
        ax1[-1].set_xlabel("Time [s]")

        fig2, ax2 = plt.subplots(3,1, figsize=(10,8), sharex=True)
        fig2.suptitle("Body Torque: Command vs Actual", fontsize=14, fontweight="bold")
        for i,lbl in enumerate(["τx [N·m]","τy [N·m]","τz [N·m]"]):
            ax2[i].plot(t, tau_used_log[:,i], '--', label='Command (used)')
            ax2[i].plot(t, tau_act_log[:,i],  label='Actual (J·γ̇)')
            ax2[i].set_ylabel(lbl); ax2[i].legend(); ax2[i].grid(True)
        ax2[-1].set_xlabel("Time [s]")

        fig3, ax3 = plt.subplots(1,1, figsize=(10,4))
        fig3.suptitle("Gimbal Angles", fontsize=14, fontweight="bold")
        for i in range(4):
            ax3.plot(t, ((gam_log[:,i]+180)%360)-180, label=f"γ{i+1}")
        ax3.set_ylabel("Angle [deg]"); ax3.set_xlabel("Time [s]")
        ax3.legend(); ax3.grid(True)

        plt.tight_layout()
        plt.show()

    return {
        "t": t,
        "euler_deg": eul_log,
        "euler_des_deg": eul_des_log,
        "w_deg_s": w_log,
        "tau_used": tau_used_log,
        "tau_act": tau_act_log,
        "gdot_deg_s": gdot_log,
        "gamma_deg": gam_log,
        "smin": smin_log,
        "smax": smax_log,
        "alpha": alpha_log,
        "lambda": lam_log,
        "sat_count": sat_count_log
    }


# Run the demo if executed directly
if __name__=="__main__":
    # Example: 90° yaw step, with plots
    run_demo(euler_deg=(45,45,45), smooth_s=2.0, plot=True)
