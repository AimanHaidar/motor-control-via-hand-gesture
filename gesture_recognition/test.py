import pickle
import numpy as np
from src.models.hand_gesture_detector import HandGestureDetector
import threading
import pandas as pd
import time

def thumb_gesture(detector):
    # Load the model
   with open("models/rf_model.pkl", "rb") as f:
        model = pickle.load(f)
        header = ["p1_x","p1_y","p2_x","p2_y","p3_x","p3_y","p4_x","p4_y"]
        last_time = time.time()
        while not detector.closed:
            if not detector.running or (time.time() - last_time) < 0.1:
                time.sleep(0.05)
                continue
            
            sample = [coord for p in detector.indecies_relative_to_wrist[1:5] for coord in p]

            # Wrap sample in DataFrame with same columns
            df_sample = pd.DataFrame([sample], columns=header)

            prediction = model.predict(df_sample)
            print("Prediction:", prediction[0])

if __name__ == "__main__":
    detector = HandGestureDetector()
    threading.Thread(target=thumb_gesture,
                    args=(detector,),
                    daemon=True).start()
    
    detector.run()
    
