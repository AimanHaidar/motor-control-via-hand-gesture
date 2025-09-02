from PyQt5 import QtWidgets
from generated.pid_control_dialog import Ui_Dialog
import paho.mqtt.client as mqtt

BROKER = "192.168.0.81"  # same as mqtt_server in ESP
PORT = 1883
PID_TOPIC = "esp32/command/pid"

class PidControlDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.connect(BROKER, PORT, 60)
        self.ui.buttonBox.accepted.connect(self.accept)
        self.ui.buttonBox.rejected.connect(self.reject)

    def accept(self):
        super().accept()
        # Handle PID control dialog result here
        self.P = self.ui.P.text()
        self.I = self.ui.I.text()
        self.D = self.ui.D.text()
        self.client.publish(PID_TOPIC, f"{self.P},{self.I},{self.D}")