import threading
import time
from datetime import datetime
from TrackerV1.RunTracker import StartTracker, QueeryTracker, StopTracker

# Timestamp for logging or naming files
timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

# Shared flag to control the running state
run_flag = {"run": True}

def TrackerWorker(run_flag):
    # Initialize tracker
    tracklets_q, last_time, pipeline = StartTracker()
    counter = 0

    print(f"[{timestamp}] Tracker started.")

    try:
        while run_flag["run"]:
            data = QueeryTracker(last_time, counter, tracklets_q)
            if data:
                # Update time + counter for next loop
                #ast_time = data["last_time"]
                #ounter = data["counter"]

                # Print formatted output (serial-style)
                print(f"[T={data['T']:.3f}s] FPS={data['FPS']:.2f} | "
                      f"Q=({data['Qx']:.3f}, {data['Qy']:.3f}, {data['Qz']:.3f}, {data['Qw']:.3f}) | "
                      f"XYZ=({data['X']:.1f}, {data['Y']:.1f}, {data['Z']:.1f})")
            else:
                time.sleep(0.01)

    except KeyboardInterrupt:
        print("TrackerWorker: interrupted.")

    finally:
        print("Stopping tracker pipeline...")
        StopTracker(pipeline)
        print("TrackerWorker: stopped cleanly.")

# === Run the tracker in a background thread ===
tracker_thread = threading.Thread(target=TrackerWorker, args=(run_flag,))
tracker_thread.start()

try:
    print("Tracker is running. Press Ctrl+C to stop early...")
    time.sleep(20)  # let it run for 60 seconds
except KeyboardInterrupt:
    print("Stopping early via keyboard interrupt...")

# Signal the worker to stop
run_flag["run"] = False

# Wait for the tracker to finish
tracker_thread.join()
print("Tracker has stopped.")
