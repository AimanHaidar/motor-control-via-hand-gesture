import pickle
import numpy as np
from src.models.hand_gesture_detector import HandGestureDetector
import threading
import pandas as pd
import time

import paho.mqtt.client as mqtt

from collections import Counter

BROKER = "192.168.0.81"  # same as mqtt_server in ESP
PORT = 1883
TOPIC = "esp32/command/speed"

def on_message(client, userdata, msg):
    print(f"Received: {msg.payload.decode()} from topic {msg.topic}")

def thumb_gesture(detector,publish):
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
            publish(TOPIC,f"Prediction: {prediction[0]}")

def hand_gesture(detector,publish):
    pred_buffer = []
    last_time = time.time()
    print("start hand gesture")
    while not detector.closed:
        if not detector.running:
            time.sleep(0.05)
            continue

        if detector.gesture == "00100":
            pred_buffer.append("stop")
        
        else:
            mode = detector.gesture.count('1')
            pred_buffer.append(mode)

        if (time.time() - last_time) > 0.75:
            most_common = Counter(pred_buffer).most_common(1)[0][0]
            print(most_common)
            publish(TOPIC,most_common)
            last_time = time.time()
            pred_buffer = []

        time.sleep(0.1)


if __name__ == "__main__":
    # Create client and connect
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.connect(BROKER, PORT, 60)

    detector = HandGestureDetector(0)
    threading.Thread(target=hand_gesture,
                    args=(detector,client.publish,),
                    daemon=True).start()
    
    detector.run()

    client.disconnect()