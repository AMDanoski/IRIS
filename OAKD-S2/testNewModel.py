#!/usr/bin/env python3
from scipy.spatial.transform import Rotation as R
import cv2
import depthai as dai
import time
import numpy as np
import math

def vector_to_quaternion(vec):
    """
    Converts a direction vector into a quaternion that rotates the forward vector [0, 0, 1]
    to the given vector using scipy's Rotation module.

    Returns: (qx, qy, qz, qw)
    """
    # Normalize input vector
    vec = np.asarray(vec, dtype=np.float64)
    norm = np.linalg.norm(vec)
    if norm < 1e-6:
        raise ValueError("Direction vector too small")

    vec /= norm
    forward = np.array([0, 0, 1])   #Projecting in Zbody by default

    # Compute rotation between forward and target vector
    rotation, rmsd = R.align_vectors([vec], [forward])
    quat = rotation.as_quat()  # Returns in [x, y, z, w] format

    return tuple(quat)  # (qx, qy, qz, qw)


def quaternion_to_euler(qx, qy, qz, qw, order='zxy'):
    r = R.from_quat([qx, qy, qz, qw])  # scipy uses [x, y, z, w]
    euler = r.as_euler(order, degrees=True)
    return tuple(euler)  # returns in (roll, pitch, yaw) based on 'order'




fullFrameTracking = False

# Build pipeline
pipeline = dai.Pipeline()

# ---------- Camera Nodes ----------
camRgb = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
# RGB output size (match NN input)

# ---------- Stereo Depth ----------
stereo = pipeline.create(dai.node.StereoDepth)
leftOutput = monoLeft.requestOutput((1280, 720))
rightOutput = monoRight.requestOutput((1280, 720))
leftOutput.link(stereo.left)
rightOutput.link(stereo.right)

# ---------- Neural Network ----------
nn_archive = dai.NNArchive(r"C:\Users\schmis10\Downloads\tennisball_detectorV3.rvc2.tar.xz") #replace with local path
spatialDetectionNetwork = (
    pipeline.create(dai.node.SpatialDetectionNetwork)
    .build(camRgb, stereo, nn_archive)
)


spatialDetectionNetwork.setConfidenceThreshold(0.2)
spatialDetectionNetwork.input.setBlocking(False)
spatialDetectionNetwork.setBoundingBoxScaleFactor(0.3)
spatialDetectionNetwork.setDepthLowerThreshold(100)
spatialDetectionNetwork.setDepthUpperThreshold(5000)
labelMap = spatialDetectionNetwork.getClasses()

# ---------- Object Tracker ----------
objectTracker = pipeline.create(dai.node.ObjectTracker)
objectTracker.setDetectionLabelsToTrack([0])  # Adjust for your tennis ball class index
objectTracker.setMaxObjectsToTrack(1)
objectTracker.setOcclusionRatioThreshold(0.2)
objectTracker.setTrackerType(dai.TrackerType.SHORT_TERM_IMAGELESS)
#objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM) #Ima keep it a buck I dont get the difference between these two
objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)
objectTracker.setTrackletBirthThreshold(15) #how long til confirm tracklet in frames
objectTracker.setTrackletMaxLifespan(20)  # how long til kill tracklet in frames

spatialDetectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)
spatialDetectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
spatialDetectionNetwork.out.link(objectTracker.inputDetections)

# ---------- Outputs ----------
preview = objectTracker.passthroughTrackerFrame.createOutputQueue()
tracklets = objectTracker.out.createOutputQueue()
detections = objectTracker.passthroughDetections.createOutputQueue()



# ---------- Runtime ----------
startTime = time.monotonic()
counter = 0
fps = 0
color = (255, 255, 255)

pipeline.start()

while pipeline.isRunning():
    imgFrame = preview.get()
    track = tracklets.get()
    detectionData = detections.get().detections
    #detection_map = {d.detectionId: d for d in detectionData}

    counter += 1
    current_time = time.monotonic()
    if (current_time - startTime) > 1:
        fps = counter / (current_time - startTime)
        counter = 0
        startTime = current_time

    frame = imgFrame.getCvFrame()
    trackletsData = track.tracklets

    for t in trackletsData:
        #detection = detection_map.get(t.detectionId, None)
        #confidence = detection.confidence if detection else None

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
            f"Y: {-t.spatialCoordinates.y:.1f} mm | "
            f"Z: {t.spatialCoordinates.z:.1f} mm")


        
        try:
            qx, qy, qz, qw = vector_to_quaternion(
            np.array([
            t.spatialCoordinates.x,
            -t.spatialCoordinates.y,   #This needs to be inverted to match the camera coordinate system
            t.spatialCoordinates.z
            ])
            )
        except ValueError:
            qx,qy,qz,qw = 0,0,0,1

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
