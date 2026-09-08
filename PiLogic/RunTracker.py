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
            "X": 0,
            "Y": 0,
            "Z": 0,
            "Counter": 0
        }

def TrackerWorker(run_flag):
    # Initialize tracker
    tracklets_q, last_time, pipeline = StartTracker()
    counter = 0
    
    print(f"[{timestamp}] Tracker started.")
 
    global CameraData

    while run_flag["run"]:
        with lock:
            tempVar = QueeryTracker(last_time, counter, tracklets_q)                
        if tempVar:
            CameraData = tempVar
            #print(f"[T={CameraData['T']:.3f}s] FPS={CameraData['FPS']:.2f} | "
                     # f"Q=({CameraData['Qx']:.3f}, {CameraData['Qy']:.3f}, {CameraData['Qz']:.3f}, {CameraData['Qw']:.3f}) | "
                      #f"XYZ=({CameraData['X']:.1f}, {CameraData['Y']:.1f}, {CameraData['Z']:.1f})")
        
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
    return tuple(rotation.as_quat())  # (qx, qy, qz, qw)

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
        

def QueeryTracker(last_time, counter, tracklets_q):
    track = tracklets_q.get()
    counter += 1
    now = time.monotonic()
    elapsed = now - last_time
    fps = 0.0
    if elapsed > 1.0:
        fps = counter / elapsed
        counter = 0
    
    for t in track.tracklets:
        x = t.spatialCoordinates.x
        y = t.spatialCoordinates.y
        z = t.spatialCoordinates.z

        try:
            qx, qy, qz, qw = vector_to_quaternion([x, -y, z])
        except ValueError:
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0

        return {
            "T": now,
            "FPS": round(fps, 2),
            "Qx": qx,
            "Qy": qy,
            "Qz": qz,
            "Qw": qw,
            "X": x,
            "Y": y,
            "Z": z,
            "Counter": counter
        }

def StopTracker(pipeline):
    pipeline.stop()
 