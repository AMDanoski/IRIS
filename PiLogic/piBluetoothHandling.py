import bluetooth
import serial
import time
import numpy as np
import os
import RunTracker
from RunTracker import StopTracker, TrackerWorker
import threading
import globals.globalVars
from odin_cmg_full import odin_cmg_step

LED_PATH = "/sys/class/leds/ACT"
def set_led(state: bool):
    os.system(f"echo none | sudo tee {LED_PATH}/trigger > /dev/null")
    val = "1" if state else "0"
    os.system(f"echo {val} | sudo tee {LED_PATH}/brightness > /dev/null")
    
def blink_led(count: int, delay=0.3):
    for _ in range(count):
        set_led(True)
        time.sleep(delay)
        set_led(False)
        time.sleep(delay)

# ---------------- Bluetooth ----------------
PORT = 1
server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
server_sock.bind(("", PORT))
server_sock.listen(1)
print("Waiting for connection on RFCOMM channel", PORT)
client_sock, addr = server_sock.accept()
print("Connected to", addr)

# ---------------- Serial (VN200) ----------------
port = "/dev/ttyUSB0"  # adjust as needed
ser = serial.Serial(port, baudrate=115200, timeout=0.01)

# Command VN200 to output quaternion
def nmea_checksum(sentence: str) -> str:
    csum = 0
    for c in sentence:
        csum ^= ord(c)
    return f"*{csum:02X}"

def send_cmd(ser, sentence: str):
    msg = f"${sentence}{nmea_checksum(sentence)}\r\n"
    ser.write(msg.encode("ascii"))
    print("[TX]", msg.strip())

send_cmd(ser, "VNWRG,06,02")
time.sleep(0.5)

latest_quat = (0, 0, 0, 0)

# ---------------- Quaternion Parsing ----------------
def parse_quaternion(line: str):
    line = line.strip()
    if not line.startswith("$VNQTN"):
        return None
    try:
        parts = line.split(',')
        q0 = float(parts[1])
        q1 = float(parts[2])
        q2 = float(parts[3])
        q3 = float(parts[4].split('*')[0])
        return q0, q1, q2, q3
    except:
        return None

def quat_to_ypr(q):
    w, x, y, z = q
    yaw = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0))
    roll = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
    return np.degrees([yaw, pitch, roll])

def ypr_to_quat(ypr):
    yaw, pitch, roll = np.radians(ypr)  # convert degrees to radians

    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])

def currentQ():
    line = ser.readline()
    if line:
        q = parse_quaternion(line.decode(errors='ignore'))
        if q:
            quaternion = q
            return quaternion

# ---------------- Main Loop ----------------
quaternion = (0,0,0,0)
ypr = [0,0,0]
BluetoothDelay = 0
data = ""
dataOld = ""
dataSensor = {
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

try:
    while True:
        # --- Receive Bluetooth commands ---
        try:

            client_sock.setblocking(False)
            data = client_sock.recv(1024).decode().strip()
            print(data)

            if dataOld == "SENSEM" and dataOld != data:
                # Signal the worker to stop when user commands new mode
                print("Ending Tracker...")
                run_flag["run"] = False

                # Wait for the tracker to finish
                tracker_thread.join()
                print("Tracker has stopped.")

                print("Stopping tracker pipeline...")
                StopTracker(RunTracker.pipeline)
                print("TrackerWorker: stopped cleanly.")

            dataOld = data

        except BlockingIOError:
            pass  # No data received, continue
        except Exception as e:
            pass

        #print("Data:", data)
        
        if data.startswith("ATT:"):
            parts = data.replace("ATT:", "").split(",")
            vals = {kv.split("=")[0]: int(kv.split("=")[1]) for kv in parts}
            yaw_cmd, pitch_cmd, roll_cmd = vals["Y"], vals["P"], vals["R"]

            q_target = ypr_to_quat([yaw_cmd, pitch_cmd, roll_cmd])

            blink_led(1)

            print(f"Commanded Attitude: Y={yaw_cmd}, P={pitch_cmd}, R={roll_cmd}")
            data = ""

        elif data == "CMCAL":
            print("Center of Mass Calibration")

            blink_led(2)
            data = ""

        elif data == "MMOICAL":
            print("MMOI Tensor Calibration")

            blink_led(3)
            data = ""

        elif data == "SENSEM":
            blink_led(4)

            run_flag = {"run": True}

            # === Run the tracker in a background thread ===
            tracker_thread = threading.Thread(target=TrackerWorker, args=(run_flag,))
            tracker_thread.start()
            data = ""

        with RunTracker.lock:
            # camera threading yields data sensor package which is updated here
            dataSensor = RunTracker.CameraData
            q_target = [dataSensor['Qx'],dataSensor['Qy'],dataSensor['Qz'],dataSensor['Qw']]

            #print(f"[T={dataSensor['T']:.3f}s] FPS={dataSensor['FPS']:.2f} | "
                        #f"Q=({dataSensor['Qx']:.3f}, {dataSensor['Qy']:.3f}, {dataSensor['Qz']:.3f}, {dataSensor['Qw']:.3f}) | "
                        #f"XYZ=({dataSensor['X']:.1f}, {dataSensor['Y']:.1f}, {dataSensor['Z']:.1f})")

        tempvar = currentQ()
        if tempvar is not None:
            # This was just lazy programming from me as I thought the exception handling in 
            # currentQ() would be enough but this is sometimes unstable and Im not sure why.
            latest_quat = tempvar
            globals.globalVars.sensor.quat = latest_quat
            ypr = quat_to_ypr(latest_quat)
            globals.globalVars.sensor.yaw = ypr[0]
            globals.globalVars.sensor.pitch = ypr[1]
            globals.globalVars.sensor.roll = ypr[2]

        #Need q_dot current and target somehow... John. Gamma is current rate of cmgs I think? Need to clairify.
        gdot_cmd, tau_cmd, tau_act, dbg = odin_cmg_step(latest_quat, qdot_cur, q_target, qdot_des, gamma, dt=None)

        # --- Send telemetry data to laptop ---
        if BluetoothDelay > 10:
            msg = f"SENSOR:Y={ypr[0]:.2f},P={ypr[1]:.2f},R={ypr[2]:.2f},T={dataSensor['T']:.2f},FPS={dataSensor['FPS']:.2f},q1={dataSensor['Qx']:.2f},q2={dataSensor['Qy']:.2f},q3={dataSensor['Qz']:.2f},q0={dataSensor['Qw']:.2f},Xpos={dataSensor['X']:.2f},Ypos={dataSensor['Y']:.2f},Zpos={dataSensor['Z']:.2f}\n"
            client_sock.send(msg)
            BluetoothDelay = 0

        BluetoothDelay = BluetoothDelay + 1
        time.sleep(0.01)  # 100 Hz update rate

except KeyboardInterrupt:
    print("Shutting down.")

finally:
   
    client_sock.close()
    server_sock.close()
    ser.close()
