import tkinter as tk
from tkinter import messagebox
import bluetooth
import threading
import time
import csv

class RobotGUI:
    def __init__(self, master):
        self.master = master
        master.title("ODIN Ground Terminal")
        master.geometry("500x350")

        self.sock = None
        self.running = False

        # ---------- Command Buttons ----------
        tk.Button(master, text="CALIBRATE CENTER OF MASS",
                  command=lambda: self.send_cmd("CMCAL")).grid(row=0, column=0, columnspan=2, pady=5)

        tk.Button(master, text="CALIBRATE MMOI TENSOR",
                  command=lambda: self.send_cmd("MMOICAL")).grid(row=1, column=0, columnspan=2, pady=5)

        tk.Button(master, text="SENSOR MODE",
                  command=lambda: self.send_cmd("SENSEM")).grid(row=2, column=0, columnspan=2, pady=5)

        # ---------- Attitude Inputs ----------
        tk.Label(master, text="Yaw:").grid(row=3, column=2, sticky="e")
        tk.Label(master, text="Pitch:").grid(row=4, column=2, sticky="e")
        tk.Label(master, text="Roll:").grid(row=5, column=2, sticky="e")

        self.yaw_entry = tk.Entry(master, width=10)
        self.yaw_entry.grid(row=3, column=3, padx=5, pady=2)

        self.pitch_entry = tk.Entry(master, width=10)
        self.pitch_entry.grid(row=4, column=3, padx=5, pady=2)

        self.roll_entry = tk.Entry(master, width=10)
        self.roll_entry.grid(row=5, column=3, padx=5, pady=2)

        tk.Button(master, text="Send Attitude", width=15,
                  command=self.send_attitude).grid(row=6, column=2, columnspan=2, pady=10)

        # ---------- Sensor Feedback Labels ----------
        tk.Label(master, text="Live Sensor Data:", font=("Arial", 10, "bold")).grid(row=7, column=2, columnspan=2, pady=(10,0))

        self.yaw_label = tk.Label(master, text="Yaw: 0")
        self.yaw_label.grid(row=8, column=2, sticky="w")
        self.pitch_label = tk.Label(master, text="Pitch: 0")
        self.pitch_label.grid(row=9, column=2, sticky="w")
        self.roll_label = tk.Label(master, text="Roll: 0")
        self.roll_label.grid(row=10, column=2, sticky="w")
        self.batt_label = tk.Label(master, text="Battery: 0")
        self.batt_label.grid(row=11, column=2, sticky="w")

        # ---------- Connect / Disconnect ----------
        tk.Button(master, text="Connect", command=self.connect).grid(row=12, column=0, pady=15)
        tk.Button(master, text="Disconnect", command=self.disconnect).grid(row=12, column=1, pady=15)

    # ---------- Bluetooth Handling ----------
    def connect(self):
        PI_BLUETOOTH_ADDR = "DC:A6:32:FE:65:2F"  # Replace with your Pi's MAC
        PORT = 1
        try:
            self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
            self.sock.connect((PI_BLUETOOTH_ADDR, PORT))
            messagebox.showinfo("Connected", f"Connected to {PI_BLUETOOTH_ADDR}")

            # Start the receiver thread for live data
            self.running = True
            thread = threading.Thread(target=self.receive_loop, daemon=True)
            thread.start()
        except Exception as e:
            messagebox.showerror("Connection Failed", str(e))

    def send_cmd(self, cmd: str):
        if self.sock:
            try:
                self.sock.send(cmd + "\n")
            except Exception as e:
                messagebox.showerror("Send Error", str(e))
        else:
            messagebox.showwarning("Not Connected", "Please connect first.")

    def send_attitude(self):
        try:
            y = int(self.yaw_entry.get())
            p = int(self.pitch_entry.get())
            r = int(self.roll_entry.get())
            cmd = f"ATT:Y={y},P={p},R={r}"
            self.send_cmd(cmd)
        except ValueError:
            messagebox.showerror("Input Error", "Please enter valid integers for Y, P, and R.")

    # ---------- Receive Sensor Data ----------
    def receive_loop(self):
        # Open CSV log file
        with open("sensor_log.csv", "a", newline="") as log_file:
            writer = csv.writer(log_file)
            writer.writerow(["timestamp", "yaw", "pitch", "roll", "battery"])  # header

            while self.running and self.sock:
                try:
                    data = self.sock.recv(1024).decode().strip()
                    if data.startswith("SENSOR:"):
                        # Example: SENSOR:Y=10,P=-5,R=0,BATT=12.3
                        parts = data.replace("SENSOR:", "").split(",")
                        vals = {}
                        for kv in parts:
                            k, v = kv.split("=")
                            vals[k] = float(v)
                        yaw, pitch, roll, batt = vals["Y"], vals["P"], vals["R"], vals["BATT"]

                        # Update GUI labels safely
                        self.master.after(0, lambda y=yaw, p=pitch, r=roll, b=batt: self.update_sensor_display(y, p, r, b))

                        # Log data
                        writer.writerow([time.time(), yaw, pitch, roll, batt])
                        log_file.flush()
                except:
                    pass
                time.sleep(0.05)  # small delay to reduce CPU usage

    def update_sensor_display(self, yaw, pitch, roll, batt):
        self.yaw_label.config(text=f"Yaw: {yaw:.1f}")
        self.pitch_label.config(text=f"Pitch: {pitch:.1f}")
        self.roll_label.config(text=f"Roll: {roll:.1f}")
        self.batt_label.config(text=f"Battery: {batt:.2f} V")

    def disconnect(self):
        self.running = False
        if self.sock:
            self.sock.close()
            self.sock = None
            messagebox.showinfo("Disconnected", "Connection closed.")


if __name__ == "__main__":
    root = tk.Tk()
    gui = RobotGUI(root)
    root.mainloop()
