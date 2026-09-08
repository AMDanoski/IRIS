#!/usr/bin/env python3
"""
test_live_tracker_with_plots.py — run your DepthAI tracker in live mode.
Prints quaternion, q̇, and XYZ each frame until Ctrl+C.
Stores all data and generates plots afterward.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from RunTrackerV2PiLogic import StartTracker, QueeryTracker, StopTracker, TrackerState

def main():
    print("=== Starting DepthAI live tracker test ===")

    tracklets_q, start_time, pipeline = StartTracker()
    state = TrackerState()
    frame = 0
    t0 = time.monotonic()

    # Data storage lists
    data = {
        "T": [],
        "FPS": [],
        "Qx": [], "Qy": [], "Qz": [], "Qw": [],
        "QdotX": [], "QdotY": [], "QdotZ": [], "QdotW": [],
        "X": [], "Y": [], "Z": []
    }

    # Print headers for all the data to be printed
    print(f"{'T[s]':>7} {'FPS':>6} {'Qx':>9} {'Qy':>9} {'Qz':>9} {'Qw':>9} "
          f"{'QdotX':>9} {'QdotY':>9} {'QdotZ':>9} {'QdotW':>9} "
          f"{'X':>9} {'Y':>9} {'Z':>9}")

    try:
        while True:
            out = QueeryTracker(tracklets_q, start_time, state)
            frame += 1

            # Store data
            for key in data.keys():
                data[key].append(out[key])

            # Extract data from the dictionary to print it
            T = out["T"]
            FPS = out["FPS"]
            Qx, Qy, Qz, Qw = out["Qx"], out["Qy"], out["Qz"], out["Qw"]
            QdotX, QdotY, QdotZ, QdotW = out["QdotX"], out["QdotY"], out["QdotZ"], out["QdotW"]
            X, Y, Z = out["X"], out["Y"], out["Z"]

            # Print formatted output
            print(f"{T:7.3f} {FPS:6.2f} "
                  f"{Qx:9.4f} {Qy:9.4f} {Qz:9.4f} {Qw:9.4f} "
                  f"{QdotX:9.4f} {QdotY:9.4f} {QdotZ:9.4f} {QdotW:9.4f} "
                  f"{X:9.1f} {Y:9.1f} {Z:9.1f}")

            # Optional: stop after fixed duration (e.g., 30 s)
            if (time.monotonic() - t0) > 30:
                 break

    except KeyboardInterrupt:
        print("\nUser interrupted.")
    finally:
        StopTracker(pipeline)
        print("Stopped pipeline cleanly.")

    # Convert lists to numpy arrays for easier plotting
    for key in data.keys():
        data[key] = np.array(data[key])

    print(f"\nCollected {len(data['T'])} frames of data.")
    print("Generating plots...")

    # Create comprehensive plots
    create_plots(data)
    print("Plots saved!")

def create_plots(data):
    """Generate comprehensive plots of all tracked data."""
    
    T = data["T"]
    
    # Create figure with multiple subplots
    fig = plt.figure(figsize=(16, 12))
    gs = GridSpec(4, 2, figure=fig, hspace=0.3, wspace=0.3)

    # 1. Position (X, Y, Z) over time
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(T, data["X"], label='X', linewidth=2)
    ax1.plot(T, data["Y"], label='Y', linewidth=2)
    ax1.plot(T, data["Z"], label='Z', linewidth=2)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (mm)')
    ax1.set_title('Position Over Time')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # 2. 3D Trajectory
    ax2 = fig.add_subplot(gs[0, 1], projection='3d')
    # Filter out NaN values for cleaner 3D plot
    valid_mask = ~(np.isnan(data["X"]) | np.isnan(data["Y"]) | np.isnan(data["Z"]))
    if np.any(valid_mask):
        X_valid = data["X"][valid_mask]
        Y_valid = data["Y"][valid_mask]
        Z_valid = data["Z"][valid_mask]
        
        # Color by time
        scatter = ax2.scatter(X_valid, Y_valid, Z_valid, c=T[valid_mask], 
                            cmap='viridis', s=10, alpha=0.6)
        ax2.plot(X_valid, Y_valid, Z_valid, 'gray', alpha=0.3, linewidth=0.5)
        plt.colorbar(scatter, ax=ax2, label='Time (s)', shrink=0.5)
    ax2.set_xlabel('X (mm)')
    ax2.set_ylabel('Y (mm)')
    ax2.set_zlabel('Z (mm)')
    ax2.set_title('3D Trajectory')

    # 3. Quaternion components
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(T, data["Qx"], label='Qx', linewidth=2)
    ax3.plot(T, data["Qy"], label='Qy', linewidth=2)
    ax3.plot(T, data["Qz"], label='Qz', linewidth=2)
    ax3.plot(T, data["Qw"], label='Qw', linewidth=2)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Quaternion')
    ax3.set_title('Quaternion Components')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # 4. Quaternion derivative components
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(T, data["QdotX"], label='Q̇x', linewidth=2)
    ax4.plot(T, data["QdotY"], label='Q̇y', linewidth=2)
    ax4.plot(T, data["QdotZ"], label='Q̇z', linewidth=2)
    ax4.plot(T, data["QdotW"], label='Q̇w', linewidth=2)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Quaternion Derivative')
    ax4.set_title('Quaternion Derivative Components')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    # 5. Quaternion magnitude (should be ~1)
    ax5 = fig.add_subplot(gs[2, 0])
    q_mag = np.sqrt(data["Qx"]**2 + data["Qy"]**2 + data["Qz"]**2 + data["Qw"]**2)
    ax5.plot(T, q_mag, linewidth=2, color='purple')
    ax5.axhline(y=1.0, color='red', linestyle='--', label='Expected (1.0)')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Magnitude')
    ax5.set_title('Quaternion Magnitude (Normalization Check)')
    ax5.legend()
    ax5.grid(True, alpha=0.3)

    # 6. Distance from camera
    ax6 = fig.add_subplot(gs[2, 1])
    valid_mask = ~(np.isnan(data["X"]) | np.isnan(data["Y"]) | np.isnan(data["Z"]))
    if np.any(valid_mask):
        distance = np.sqrt(data["X"]**2 + data["Y"]**2 + data["Z"]**2)
        ax6.plot(T[valid_mask], distance[valid_mask], linewidth=2, color='orange')
        ax6.set_xlabel('Time (s)')
        ax6.set_ylabel('Distance (mm)')
        ax6.set_title('Distance from Camera')
        ax6.grid(True, alpha=0.3)

    # 7. FPS over time
    ax7 = fig.add_subplot(gs[3, 0])
    ax7.plot(T, data["FPS"], linewidth=2, color='green')
    ax7.set_xlabel('Time (s)')
    ax7.set_ylabel('FPS')
    ax7.set_title('Tracking FPS Over Time')
    ax7.grid(True, alpha=0.3)
    if len(data["FPS"]) > 0:
        avg_fps = np.mean(data["FPS"][data["FPS"] > 0])
        ax7.axhline(y=avg_fps, color='red', linestyle='--', 
                   label=f'Average: {avg_fps:.2f}')
        ax7.legend()

    # 8. XY position (top-down view)
    ax8 = fig.add_subplot(gs[3, 1])
    if np.any(valid_mask):
        scatter = ax8.scatter(data["X"][valid_mask], data["Y"][valid_mask], 
                            c=T[valid_mask], cmap='viridis', s=20, alpha=0.6)
        ax8.plot(data["X"][valid_mask], data["Y"][valid_mask], 
                'gray', alpha=0.3, linewidth=0.5)
        plt.colorbar(scatter, ax=ax8, label='Time (s)')
    ax8.set_xlabel('X (mm)')
    ax8.set_ylabel('Y (mm)')
    ax8.set_title('XY Trajectory (Top-Down View)')
    ax8.grid(True, alpha=0.3)
    ax8.axis('equal')

    plt.suptitle('Tennis Ball Tracking Analysis', fontsize=16, fontweight='bold')
    
    # Save the figure
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"tracker_analysis_{timestamp}.png"
    plt.savefig(filename, dpi=150, bbox_inches='tight')
    print(f"Plot saved as: {filename}")
    
    # Show the plot
    plt.show()

if __name__ == "__main__":
    main()