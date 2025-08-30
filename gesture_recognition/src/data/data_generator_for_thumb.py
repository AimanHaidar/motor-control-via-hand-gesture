import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from models.hand_gesture_detector import HandGestureDetector
import csv
import threading
import time

def collect_data(detector):
    with open("data/thumb_points.csv", "w", newline="") as f:
        header = ["p1_x","p1_y","p2_x","p2_y","p3_x","p3_y","p4_x","p4_y","counted"]
        writer = csv.writer(f)
        writer.writerow(header)
        detector.counted = True
        last_time = time.time()
        while not detector.closed:   # <--- stop cleanly when detector stops
            # only log once per second
            if (time.time() - last_time) < 0.05:
                time.sleep(0.05)  # prevent CPU overuse
                continue

            last_time = time.time()
            if detector.running:
                try:
                    thumb_points = detector.indecies_relative_to_wrist[1:5]
                except Exception as e:
                    print(f"Error: {e}")
                    continue
                
                # flatten [[x,y], [x,y], ...] -> [x,y,x,y,...]
                row = [coord for p in thumb_points for coord in p] + [detector.counted]
                print(row)
                writer.writerow(row)
            else:
                pass


if __name__ == "__main__":
    detector = HandGestureDetector(camera_address=0, fps=10)

    # run data collection in background
    threading.Thread(target=collect_data, args=(detector,), daemon=True).start()

    # main loop → handles camera + "q" press
    detector.run()

    print("Program finished.")
