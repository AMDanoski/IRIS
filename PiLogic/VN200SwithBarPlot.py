import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ---------- Serial setup ----------
port = "/dev/ttyUSB0" #will probably be "/dev/ttyUSB0", or "/dev/serial0" on Pi

def nmea_checksum(sentence: str) -> str:
    csum = 0
    for c in sentence:
        csum ^= ord(c)
    return f"*{csum:02X}"

def send_cmd(ser, sentence: str):
    msg = f"${sentence}{nmea_checksum(sentence)}\r\n"
    ser.write(msg.encode("ascii"))
    print("[TX]", msg.strip())

ser = serial.Serial(port, baudrate=115200, timeout=0.001)  # very short timeout
send_cmd(ser, "VNWRG,06,02")
time.sleep(0.5)

latest_quat = (1, 0, 0, 0)

# ---------- Parse quaternion ----------
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

# ---------- Quaternion to YPR ----------
def quat_to_ypr(q):
    w, x, y, z = q
    
    yaw = np.arctan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
    pitch = np.arcsin(np.clip(2*(w*y - z*x), -1.0, 1.0))
    roll = np.arctan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
    
    return np.degrees([yaw, pitch, roll])

# ---------- Matplotlib setup ----------
fig, ax = plt.subplots()
ax.set_xlim(0, 200)
ax.set_ylim(-180, 180)
ax.set_xlabel("Samples")
ax.set_ylabel("Degrees")
ax.set_title("VN-200 Yaw/Pitch/Roll")
ax.grid(True)

yaw_data, pitch_data, roll_data = [], [], []
x_data = []

line_yaw, = ax.plot([], [], label="Yaw", color="r")
line_pitch, = ax.plot([], [], label="Pitch", color="g")
line_roll, = ax.plot([], [], label="Roll", color="b")
ax.legend()

# ---------- Update function ----------
def update(frame):
    global latest_quat

    # Process all available serial lines
    while True:
        line = ser.readline()
        if not line:
            break
        q = parse_quaternion(line.decode(errors='ignore'))
        if q:
            latest_quat = q

    ypr = quat_to_ypr(latest_quat)
    x_data.append(len(x_data))
    yaw_data.append(ypr[0])
    pitch_data.append(ypr[1])
    roll_data.append(ypr[2])

    # Keep last 200 points
    x_plot = x_data[-200:]
    yaw_plot = yaw_data[-200:]
    pitch_plot = pitch_data[-200:]
    roll_plot = roll_data[-200:]

    line_yaw.set_data(x_plot, yaw_plot)
    line_pitch.set_data(x_plot, pitch_plot)
    line_roll.set_data(x_plot, roll_plot)
    ax.set_xlim(x_plot[0], x_plot[-1]+1)

    return line_yaw, line_pitch, line_roll

# ---------- Create animation ----------
anim = FuncAnimation(fig, update, interval=20, blit=True)  # faster updates
plt.show()
ser.close()
