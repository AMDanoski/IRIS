#!/usr/bin/env python3
"""
test_ypr_tracker.py — run your DepthAI tracker in live mode.
Prints Yaw, Pitch, Roll, XYZ, and Omega each frame until Ctrl+C.
Stores all data and generates plots afterward.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from RunTrackerV2 import StartTracker, QueeryTracker, StopTracker, TrackerState

def main():
    print("=== Starting DepthAI YPR tracker test ===")

    tracklets_q, start_time, pipeline = StartTracker()
    state = TrackerState()
    frame = 0
    t0 = time.monotonic()

    # Data storage lists (extended to match QueeryTracker output)
    data = {
        "T": [], "FPS": [],
        "Yaw": [], "Pitch": [], "Roll": [],
        "X": [], "Y": [], "Z": [],
        "OmegaX": [], "OmegaY": [], "OmegaZ": [],
        "Qx": [], "Qy": [], "Qz": [], "Qw": [],
        "QdotX": [], "QdotY": [], "QdotZ": [], "QdotW": [],
    }

    # Console header
    print(f"{'T[s]':>7} {'FPS':>6} "
          f"{'Yaw[°]':>9} {'Pitch[°]':>9} {'Roll[°]':>9} "
          f"{'X':>9} {'Y':>9} {'Z':>9} "
          f"{'Ωx':>9} {'Ωy':>9} {'Ωz':>9}")

    try:
        while True:
            out = QueeryTracker(tracklets_q, start_time, state)
            frame += 1

            # Store everything we track (guard if some keys missing)
            for k in data.keys():
                data[k].append(out.get(k, np.nan))

            # Extract for printing
            T    = out["T"];     FPS  = out["FPS"]
            Yaw  = out["Yaw"];   Pitch = out["Pitch"]; Roll = out["Roll"]
            X    = out["X"];     Y    = out["Y"];      Z    = out["Z"]
            Ox   = out.get("OmegaX", np.nan)
            Oy   = out.get("OmegaY", np.nan)
            Oz   = out.get("OmegaZ", np.nan)

            print(f"{T:7.3f} {FPS:6.2f} "
                  f"{Yaw:9.2f} {Pitch:9.2f} {Roll:9.2f} "
                  f"{X:9.1f} {Y:9.1f} {Z:9.1f} "
                  f"{Ox:9.3f} {Oy:9.3f} {Oz:9.3f}")

            # Optional: stop after fixed duration (e.g., 30 s)
            if (time.monotonic() - t0) > 30:
                break

    except KeyboardInterrupt:
        print("\nUser interrupted.")
    finally:
        StopTracker(pipeline)
        print("Stopped pipeline cleanly.")

    # Convert lists to numpy arrays
    for key in data.keys():
        data[key] = np.array(data[key])

    print(f"\nCollected {len(data['T'])} frames of data.")
    print("Generating plots...")

    create_plots(data)
    print("Plots saved!")

def create_plots(data):
    """Generate comprehensive plots of all tracked data."""
    T = data["T"]

    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(4, 2, figure=fig, hspace=0.3, wspace=0.3)
    # 1) Position over time
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(T, data["X"], label='X', linewidth=2, color='red')     # X → Roll → red
    ax1.plot(T, data["Y"], label='Y', linewidth=2, color='green')   # Y → Pitch → green
    ax1.plot(T, data["Z"], label='Z', linewidth=2, color='blue')    # Z → Yaw → blue
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (mm)')
    ax1.set_title('Position Over Time')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 2) 3D Trajectory
    ax2 = fig.add_subplot(gs[0, 1], projection='3d')
    valid_xyz = ~(np.isnan(data["X"]) | np.isnan(data["Y"]) | np.isnan(data["Z"]))
    if np.any(valid_xyz):
        Xv, Yv, Zv = data["X"][valid_xyz], data["Y"][valid_xyz], data["Z"][valid_xyz]
        sc = ax2.scatter(Xv, Yv, Zv, c=T[valid_xyz], cmap='viridis', s=10, alpha=0.6)
        ax2.plot(Xv, Yv, Zv, 'gray', alpha=0.3, linewidth=0.5)
        plt.colorbar(sc, ax=ax2, label='Time (s)', shrink=0.5)
    ax2.set_xlabel('X (mm)')
    ax2.set_ylabel('Y (mm)')
    ax2.set_zlabel('Z (mm)')
    ax2.set_title('3D Trajectory')

    # 3) Yaw, Pitch, Roll over time
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(T, data["Yaw"],   label='Yaw (Z)',   linewidth=2, color='blue')   # yaw ↔ Z axis
    ax3.plot(T, data["Pitch"], label='Pitch (Y)', linewidth=2, color='green')  # pitch ↔ Y axis
    ax3.plot(T, data["Roll"],  label='Roll (X)',  linewidth=2, color='red')    # roll ↔ X axis
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (°)')
    ax3.set_title('Yaw, Pitch, Roll Over Time')
    ax3.legend()
    ax3.grid(True, alpha=0.3)


    # 4) Yaw vs Pitch
    ax4 = fig.add_subplot(gs[1, 1])
    valid_ypr = ~(np.isnan(data["Yaw"]) | np.isnan(data["Pitch"]))
    if np.any(valid_ypr):
        sc2 = ax4.scatter(data["Yaw"][valid_ypr], data["Pitch"][valid_ypr],
                          c=T[valid_ypr], cmap='viridis', s=20, alpha=0.6)
        ax4.plot(data["Yaw"][valid_ypr], data["Pitch"][valid_ypr], 'gray', alpha=0.3, linewidth=0.5)
        plt.colorbar(sc2, ax=ax4, label='Time (s)')
    ax4.set_xlabel('Yaw (°)'); ax4.set_ylabel('Pitch (°)')
    ax4.set_title('Yaw vs Pitch'); ax4.grid(True, alpha=0.3)

    # 5) Distance from camera
    ax5 = fig.add_subplot(gs[2, 0])
    if np.any(valid_xyz):
        dist = np.sqrt(data["X"]**2 + data["Y"]**2 + data["Z"]**2)
        ax5.plot(T[valid_xyz], dist[valid_xyz], linewidth=2)
        ax5.set_xlabel('Time (s)'); ax5.set_ylabel('Distance (mm)')
        ax5.set_title('Distance from Camera'); ax5.grid(True, alpha=0.3)

    # 6) FPS over time
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.plot(T, data["FPS"], linewidth=2)
    ax6.set_xlabel('Time (s)'); ax6.set_ylabel('FPS')
    ax6.set_title('Tracking FPS Over Time'); ax6.grid(True, alpha=0.3)
    if len(data["FPS"]) > 0:
        good = data["FPS"] > 0
        if np.any(good):
            avg_fps = np.mean(data["FPS"][good])
            ax6.axhline(y=avg_fps, color='red', linestyle='--', label=f'Average: {avg_fps:.2f}')
            ax6.legend()

    # 7) Angular rates (Omega) over time
    ax7 = fig.add_subplot(gs[3, 0])
    ax7.plot(T, data["OmegaX"], label='Ωx', linewidth=2)
    ax7.plot(T, data["OmegaY"], label='Ωy', linewidth=2)
    ax7.plot(T, data["OmegaZ"], label='Ωz', linewidth=2)
    ax7.set_xlabel('Time (s)'); ax7.set_ylabel('Angular Rate (rad/s)')
    ax7.set_title('Angular Velocity Over Time'); ax7.legend(); ax7.grid(True, alpha=0.3)

    # 8) XY top-down
    ax8 = fig.add_subplot(gs[3, 1])
    if np.any(valid_xyz):
        sc3 = ax8.scatter(data["X"][valid_xyz], data["Y"][valid_xyz],
                          c=T[valid_xyz], cmap='viridis', s=20, alpha=0.6)
        ax8.plot(data["X"][valid_xyz], data["Y"][valid_xyz], 'gray', alpha=0.3, linewidth=0.5)
        plt.colorbar(sc3, ax=ax8, label='Time (s)')
    ax8.set_xlabel('X (mm)'); ax8.set_ylabel('Y (mm)')
    ax8.set_title('XY Trajectory (Top-Down)'); ax8.grid(True, alpha=0.3); ax8.axis('equal')

    plt.suptitle('Tennis Ball YPR Tracking Analysis', fontsize=16, fontweight='bold')

    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"ypr_tracker_analysis_{timestamp}.png"
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"Plot saved as: {filename}")
    plt.show()

if __name__ == "__main__":
    main()
