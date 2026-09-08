#!/usr/bin/env python3
from scipy.spatial.transform import Rotation as R
import depthai as dai
import numpy as np
import time
from pathlib import Path
from datetime import datetime
import threading

# === Fixed model path (relative to this script ===)
# Assumes model and this script live in the same directory
SCRIPT_DIR = Path(__file__).resolve().parent
NN_ARCHIVE_PATH = SCRIPT_DIR / "TennisballDetectorV3.rvc2.tar.xz"
timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

lock = threading.Lock()

CameraData = {
    "T": 0,
    "FPS": 0,
    "Qx": 0,
    "Qy": 0,
    "Qz": 0,
    "Qw": 0,
    "QdotX": 0,
    "QdotY": 0,
    "QdotZ": 0,
    "QdotW": 0,
    "X": 0,
    "Y": 0,
    "Z": 0,
}


class TrackerState:
    """
    Holds persistent timing/orientation state for FPS, ω, and q̇ estimation.
    """
    def __init__(self):
        self.last_tick = None   # previous monotonic() time for FPS
        self.fps = 0.0
        self.prev_q = None      # previous quaternion [x,y,z,w]
        self.prev_t = None      # timestamp of previous quaternion


def TrackerWorker(run_flag):
    # Initialize tracker
    tracklets_q, start_time, pipeline = StartTracker()
    state = TrackerState()
    
    print(f"[{timestamp}] Tracker started.")
 
    global CameraData

    while run_flag["run"]:
        with lock:
            tempVar = QueeryTracker(tracklets_q, start_time, state)                
        if tempVar:
            CameraData = tempVar
            #print(f"[T={CameraData['T']:.3f}s] FPS={CameraData['FPS']:.2f} | "
            #      f"Q=({CameraData['Qx']:.3f}, {CameraData['Qy']:.3f}, {CameraData['Qz']:.3f}, {CameraData['Qw']:.3f}) | "
            #      f"Qdot=({CameraData['QdotX']:.3f}, {CameraData['QdotY']:.3f}, {CameraData['QdotZ']:.3f}, {CameraData['QdotW']:.3f}) | "
            #      f"XYZ=({CameraData['X']:.1f}, {CameraData['Y']:.1f}, {CameraData['Z']:.1f})")
        
    print("Stopping tracker pipeline...")
    StopTracker(pipeline)
    print("TrackerWorker: stopped cleanly.")


def vector_to_quaternion(vec):
    """Convert direction vector -> quaternion rotating [0,0,1] to vec."""
    vec = np.asarray(vec, dtype=np.float64)
    norm = np.linalg.norm(vec)
    if norm < 1e-6:
        raise ValueError("Direction vector too small")
    vec /= norm
    forward = np.array([0.0, 0.0, 1.0])
    rotation, _ = R.align_vectors([vec], [forward])
    q = rotation.as_quat()  # (x,y,z,w)
    return tuple((q / np.linalg.norm(q)).tolist())


def _update_fps(now, state: TrackerState):
    """Smooth FPS estimator using exponential moving average."""
    if state.last_tick is None:
        state.last_tick = now
        return state.fps
    dt = now - state.last_tick
    state.last_tick = now
    if dt <= 0:
        return state.fps
    inst = 1.0 / dt
    state.fps = inst if state.fps == 0.0 else 0.85 * state.fps + 0.15 * inst
    return state.fps


def _quat_conj(q):
    x, y, z, w = q
    return np.array([-x, -y, -z, w], dtype=np.float64)


def _quat_mul(p, r):
    px, py, pz, pw = p
    rx, ry, rz, rw = r
    return np.array([
        pw*rx + rw*px + py*rz - pz*ry,
        pw*ry + rw*py + pz*rx - px*rz,
        pw*rz + rw*pz + px*ry - py*rx,
        pw*rw - (px*rx + py*ry + pz*rz)
    ], dtype=np.float64)


def _unit(q):
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    return q if n == 0 else q / n


def _omega_and_qdot(prev_q, q, dt, world_left=True):
    """
    Compute angular velocity ω (rad/s) and quaternion derivative q̇.
    prev_q, q: unit quaternions [x,y,z,w]
    dt: time difference (s)
    world_left=True → q̇ = 0.5 [0,ω] ⊗ q  (ω in world frame)
    """
    if dt <= 0:
        return np.zeros(3), np.zeros(4)

    if np.dot(prev_q, q) < 0.0:
        q = -q  # keep same hemisphere

    dq = _quat_mul(q, _quat_conj(prev_q))  # relative rotation prev→current
    vx, vy, vz, w = dq
    v = np.array([vx, vy, vz], dtype=np.float64)
    nv = np.linalg.norm(v)

    # Robust axis-angle
    if nv < 1e-12:
        theta = 0.0
        axis = np.array([0.0, 0.0, 0.0])
    else:
        theta = 2.0 * np.arctan2(nv, w)
        axis = v / nv

    omega = (theta / dt) * axis  # rad/s

    # Quaternion derivative from ω
    x, y, z, wq = q
    wx, wy, wz = omega
    if world_left:
        qdot = 0.5 * np.array([
            + wq*wx + y*wz - z*wy,
            + wq*wy + z*wx - x*wz,
            + wq*wz + x*wy - y*wx,
            - (x*wx + y*wy + z*wz)
        ])
    else:
        qdot = 0.5 * np.array([
            + wq*wx + z*wy - y*wz,
            + wq*wy + x*wz - z*wx,
            + wq*wz + y*wx - x*wy,
            - (x*wx + y*wy + z*wz)
        ])
    return omega, qdot


def StartTracker():
    """
    Run the headless tennis ball tracker and yield data:
    FPS, T (seconds since start), Quaternion (Qx, Qy, Qz, Qw), Position (X, Y, Z)
    """
    pipeline = dai.Pipeline()

    camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

    stereo = pipeline.create(dai.node.StereoDepth)
    monoLeft.requestOutput((1280, 720)).link(stereo.left)
    monoRight.requestOutput((1280, 720)).link(stereo.right)

    nn_archive = dai.NNArchive(str(NN_ARCHIVE_PATH))
    spatialNN = pipeline.create(dai.node.SpatialDetectionNetwork).build(camRgb, stereo, nn_archive)
    spatialNN.setConfidenceThreshold(0.2)
    spatialNN.setDepthLowerThreshold(100)
    spatialNN.setDepthUpperThreshold(5000)

    tracker = pipeline.create(dai.node.ObjectTracker)
    tracker.setDetectionLabelsToTrack([0])
    tracker.setMaxObjectsToTrack(1)
    tracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
    tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)
    tracker.setTrackletBirthThreshold(15)
    tracker.setTrackletMaxLifespan(20)

    spatialNN.passthrough.link(tracker.inputTrackerFrame)
    spatialNN.passthrough.link(tracker.inputDetectionFrame)
    spatialNN.out.link(tracker.inputDetections)
    tracklets_q = tracker.out.createOutputQueue()
    start_time = time.monotonic()
    pipeline.start()
    return tracklets_q, start_time, pipeline
        

def QueeryTracker(tracklets_q, start_time, state: TrackerState):
    """
    Query the DepthAI tracker queue and return:
      - Time since start (T)
      - FPS
      - Quaternion (Qx,Qy,Qz,Qw)
      - Quaternion derivative (QdotX...QdotW)
      - Angular velocity (OmegaX...OmegaZ)
      - Spatial coordinates (X,Y,Z)
    """
    track = tracklets_q.get()  # blocking
    now = time.monotonic()
    fps = _update_fps(now, state)

    out = {
        "T": round(now - start_time, 6),
        "FPS": round(fps, 2),
        "Qx": 0.0, "Qy": 0.0, "Qz": 0.0, "Qw": 1.0,
        "QdotX": 0.0, "QdotY": 0.0, "QdotZ": 0.0, "QdotW": 0.0,
        "X": float("nan"), "Y": float("nan"), "Z": float("nan"),
    }

    if len(track.tracklets) == 0:
        return out

    t = track.tracklets[0]
    x = float(t.spatialCoordinates.x)
    y = float(t.spatialCoordinates.y)
    z = float(t.spatialCoordinates.z)

    try:
        qx, qy, qz, qw = vector_to_quaternion([x, -y, z])
    except ValueError:
        qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

    q = _unit(np.array([qx, qy, qz, qw], dtype=np.float64))

    if state.prev_q is None:
        omega = np.zeros(3)
        qdot = np.zeros(4)
    else:
        dt = now - state.prev_t
        omega, qdot = _omega_and_qdot(state.prev_q, q, dt, world_left=True)

    # update persistent state
    state.prev_q = q
    state.prev_t = now

    out.update({
        "Qx": float(q[0]), "Qy": float(q[1]), "Qz": float(q[2]), "Qw": float(q[3]),
        "QdotX": float(qdot[0]), "QdotY": float(qdot[1]), "QdotZ": float(qdot[2]), "QdotW": float(qdot[3]),
        "X": x, "Y": y, "Z": z
    })
    return out


def StopTracker(pipeline):
    pipeline.stop()