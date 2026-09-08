#!/usr/bin/env python3
from scipy.spatial.transform import Rotation as R
import depthai as dai
import numpy as np
import time
from pathlib import Path


# === Fixed model path (relative to this script ===)
# Assumes model and this script live in the same directory
SCRIPT_DIR = Path(__file__).resolve().parent
NN_ARCHIVE_PATH = SCRIPT_DIR / "tennisball_detectionV3_rvc2.tar.xz"


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


def quaternion_to_euler(qx, qy, qz, qw, order='zxy'):
    """Quaternion -> Euler angles (deg). Default order = 'zxy'."""
    r = R.from_quat([qx, qy, qz, qw])
    roll, pitch, yaw = r.as_euler(order, degrees=True)
    return roll, pitch, yaw


def run_tracker(run_flag):
    """
    Run the headless tennis ball tracker.

    Parameters
    ----------
    run_flag : dict
        Mutable dictionary controlling runtime: {"run": True}.
        Set run_flag["run"] = False externally to stop.
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
    counter = 0
    fps = 0.0

    pipeline.start()
    print(f"Tracker started using model: {NN_ARCHIVE_PATH}")
    print("Set run_flag['run'] = False or Ctrl+C to stop.\n")

    try:
        while pipeline.isRunning() and run_flag.get("run", True):
            track = tracklets_q.get()
            counter += 1
            now = time.monotonic()
            if (now - start_time) > 1:
                fps = counter / (now - start_time)
                counter = 0
                start_time = now

            for t in track.tracklets:
                try:
                    qx, qy, qz, qw = vector_to_quaternion([
                        t.spatialCoordinates.x,
                        -t.spatialCoordinates.y,
                        t.spatialCoordinates.z
                    ])
                except ValueError:
                    qx, qy, qz, qw = 0, 0, 0, 1

                roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
                timestamp = time.strftime("%H:%M:%S", time.localtime())

                print(f"[{timestamp}] FPS={fps:.2f} | "
                      f"Quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f}) | "
                      f"YPR=({yaw:.2f},{pitch:.2f},{roll:.2f})")

    except KeyboardInterrupt:
        run_flag["run"] = False
        print("Interrupted by user.")
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        print("Tracker stopped cleanly.")
