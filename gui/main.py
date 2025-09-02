from http import client
from PyQt5.QtWidgets import QApplication, QMainWindow,QMessageBox
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtGui import QFontDatabase, QFont
from generated.main_window import Ui_MainWindow
from dialogs.pid_control_dialog import PidControlDialog
from dialogs.speed_monitor_dialog import SpeedMonitorDialog
import paho.mqtt.client as mqtt
import sys

BROKER = "192.168.0.81"  # same as mqtt_server in ESP
PORT = 1883
PID_TOPIC = "esp32/command/pid"
SPEED_TOPIC = "esp32/sensor/speed"
SPEED_COMMAND_TOPIC = "esp32/command/speed"
class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(BROKER, PORT, 60)
        self.client.loop_start()  # starts a background thread to handle messages

        print("MQTT client connected")
        self.speed_dialog = None

        self.ui.set_pid.clicked.connect(self.open_pid_dialog)
        self.ui.show_speed.clicked.connect(self.show_speed)

    def open_pid_dialog(self):
        dialog = PidControlDialog(self)
        dialog.exec_()

    def show_speed(self):
        self.speed_dialog = SpeedMonitorDialog(self)
        self.speed_dialog.exec_()

    # Callback when connected to broker
    def on_connect(self,client, userdata, flags, rc):
        if rc == 0:
            print("Connected successfully")
            client.subscribe(SPEED_TOPIC)
            client.subscribe(SPEED_COMMAND_TOPIC)
        else:
            print("Connection failed with code", rc)

    # Callback when a message is received
    def on_message(self,client, userdata, msg):
        topic = msg.topic
        if msg.topic == SPEED_TOPIC:
            self.speed_dialog.ui.actual_speed.setText(f"Actual Speed: {msg.payload.decode()}")
        elif msg.topic == SPEED_COMMAND_TOPIC:
            self.speed_dialog.ui.set_speed.setText(f"Commanded Speed: {msg.payload.decode()}")
        
if __name__ == "__main__":
    print("Starting application...")
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()

    window.client.disconnect()
    sys.exit(app.exec_())
