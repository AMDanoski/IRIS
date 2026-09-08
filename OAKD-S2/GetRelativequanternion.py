#!/usr/bin/env python3
import cv2
import depthai as dai
import time
import numpy as np
import math

def vector_to_quaternion(x, y, z):
    """
    Placeholder function that now computes yaw/pitch directly from [x, y, z].
    Returns (qx, qy, qz, qw) where we store yaw/pitch in qx/qy for compatibility.
    """
    # Avoid div-by-zero
    if abs(z) < 1e-6:
        z = 1e-6
    
    # Yaw = left/right angle
    yaw = math.atan2(x, z)
    # Pitch = up/down angle (note: y sign is camera-specific, usually down=+)
    pitch = math.atan2(y, z)

    # Store yaw/pitch in qx, qy (qx=pitch, qy=yaw for consistency)
    return (pitch, yaw, 0.0, 1.0)


def quaternion_to_euler(qx, qy, qz, qw):
    """
    Interprets qx=Pitch (rad), qy=Yaw (rad).
    Returns Roll, Pitch, Yaw in degrees.
    Roll is always 0 since it's not meaningful for single vector directions.
    """
    roll = 0.0
    pitch = math.degrees(qx)
    yaw = math.degrees(qy)
    return roll, pitch, yaw


fullFrameTracking = False

# Build pipeline
pipeline = dai.Pipeline()

# ---------- Camera Nodes ----------
camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)

# RGB output size (match NN input)
#cam_out = camRgb.requestOutput((512, 384), dai.ImgFrame.Type.BGR888p)

# ---------- Stereo Depth ----------
stereo = pipeline.create(dai.node.StereoDepth)
leftOutput = monoLeft.requestOutput((640, 400))
rightOutput = monoRight.requestOutput((640, 400))
leftOutput.link(stereo.left)
rightOutput.link(stereo.right)

# ---------- Neural Network ----------
nn_archive = dai.NNArchive(r"C:\Users\jpajh\OneDrive - Embry-Riddle Aeronautical University\Documents\ODIN\OAKD-S2\tennisball_detection_model.rvc2.tar.xz") #replace with local path
spatialDetectionNetwork = (
    pipeline.create(dai.node.SpatialDetectionNetwork)
    .build(camRgb, stereo, nn_archive)
)


spatialDetectionNetwork.setConfidenceThreshold(0.7)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)
labelMap = spatialDetectionNetwork.getClasses()

# ---------- Object Tracker ----------
objectTracker = pipeline.create(dai.node.ObjectTracker)
objectTracker.setDetectionLabelsToTrack([0])  # Adjust for your tennis ball class index
objectTracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

# ---------- Linking ----------
if fullFrameTracking:
    camRgb.requestFullResolutionOutput().link(objectTracker.inputTrackerFrame)
    objectTracker.inputTrackerFrame.setBlocking(False)
    objectTracker.inputTrackerFrame.setMaxSize(1)
else:
    spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
spatialDetectionNetwork.out.link(objectTracker.inputDetections)

# ---------- Outputs ----------
preview = objectTracker.passthroughTrackerFrame.createOutputQueue()
tracklets = objectTracker.out.createOutputQueue()

# ---------- Runtime ----------
startTime = time.monotonic()
counter = 0
fps = 0
color = (255, 255, 255)

pipeline.start()

while pipeline.isRunning():
    imgFrame = preview.get()
    track = tracklets.get()

    counter += 1
    current_time = time.monotonic()
    if (current_time - startTime) > 1:
        fps = counter / (current_time - startTime)
        counter = 0
        startTime = current_time

    frame = imgFrame.getCvFrame()
    trackletsData = track.tracklets

    for t in trackletsData:
        roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
        x1 = int(roi.topLeft().x)
        y1 = int(roi.topLeft().y)
        x2 = int(roi.bottomRight().x)
        y2 = int(roi.bottomRight().y)

        try:
            label = labelMap[t.label]
        except:
            label = t.label

        # Print to terminal
        print(f"ID: {t.id} | X: {t.spatialCoordinates.x:.1f} mm | "
            f"Y: {t.spatialCoordinates.y:.1f} mm | "
            f"Z: {t.spatialCoordinates.z:.1f} mm")
        
        qx, qy, qz, qw = vector_to_quaternion(
        t.spatialCoordinates.x,
        -t.spatialCoordinates.y,   #This needs to be inverted to match the camera coordinate system
        t.spatialCoordinates.z
        )

        print(f"Quaternion: (qx={qx:.3f}, qy={qy:.3f}, qz={qz:.3f}, qw={qw:.3f})")

        roll, pitch, yaw = quaternion_to_euler(qx, qy, qz, qw)
        print(f"Euler: Roll={roll:.2f}°, Pitch={pitch:.2f}°, Yaw={yaw:.2f}°")


        # Draw on frame as before
        cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

        cv2.putText(frame, f"X: {int(t.spatialCoordinates.x)} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.putText(frame, f"Y: {int(t.spatialCoordinates.y)} mm", (x1 + 10, y1 + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.putText(frame, f"Z: {int(t.spatialCoordinates.z)} mm", (x1 + 10, y1 + 95), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)


    cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, color)
    cv2.imshow("tracker", frame)

    if cv2.waitKey(1) == ord('q'):
        break
