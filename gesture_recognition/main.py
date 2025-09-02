import pickle
import numpy as np
from src.models.hand_gesture_detector import HandGestureDetector
import threading
import pandas as pd
import time
import argparse

import paho.mqtt.client as mqtt

from collections import Counter

TOPIC = "esp32/command/speed"

def hand_gesture(detector,publish):
    """
        this function processes hand gestures and 
        publishes the best recognized gesture to the MQTT broker.
        note: it takes values for about 1 second then publishes the most frequent one

        Parameters:
        - detector: An instance of HandGestureDetector used for gesture recognition.
        - publish: A function to publish messages to the MQTT broker.

    """
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
    parser = argparse.ArgumentParser(description="init mqtt requirements")

    # Add arguments
    parser.add_argument("--port", type=int, default=1883, help="Broker Port")
    parser.add_argument("--broker", type=str, default="192.168.0.81", help="MQTT broker address")

    args = parser.parse_args()

    # Use the arguments like normal variables
    print(f"Broker Port set to: {args.port}")
    print(f"Broker Address set to: {args.broker}")
    # Create client and connect
    client = mqtt.Client(protocol=mqtt.MQTTv311)
    client.connect(args.broker, args.port, 60)

    # Start hand gesture processing
    detector = HandGestureDetector(0)
    threading.Thread(target=hand_gesture,
                    args=(detector,client.publish,),
                    daemon=True).start()
    
    detector.run()

    client.disconnect()