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
        self.ui.buttonBox.accepted.disconnect()
        self.ui.buttonBox.accepted.connect(self.accept)
        self.ui.buttonBox.rejected.connect(self.reject)

    def accept(self):
        """Validate PID entries, publish if valid, otherwise show warning.

        Rules (adjust as needed):
        - All fields required.
        - Must parse as float.
        - Must be finite (no inf / NaN).
        - Disallow negative values for I and D (common in many use cases) and Kp.
        """
        p_text = self.ui.P.text().strip()
        i_text = self.ui.I.text().strip()
        d_text = self.ui.D.text().strip()

        # Basic presence check
        if not p_text or not i_text or not d_text:
            QtWidgets.QMessageBox.warning(self, "Invalid PID", "All PID fields are required.")
            return

        try:
            p_val = float(p_text)
            i_val = float(i_text)
            d_val = float(d_text)
        except ValueError:
            QtWidgets.QMessageBox.warning(self, "Invalid PID", "PID values must be numeric (e.g. 1.5 0.2 0.01).")
            return

        # Reject NaN / inf
        if any(str(v) in ("nan", "inf", "-inf") for v in (p_val, i_val, d_val)):
            QtWidgets.QMessageBox.warning(self, "Invalid PID", "Values must be finite numbers.")
            return

        # Range checks (customize if negative gains are desired)
        if p_val < 0 or i_val < 0 or d_val < 0:
            QtWidgets.QMessageBox.warning(self, "Invalid PID", "Gains must be non‑negative.")
            return

        # (Optional) Upper bound sanity check
        if p_val > 1000 or i_val > 1000 or d_val > 1000:
            if QtWidgets.QMessageBox.question(
                self,
                "Large Values",
                f"One or more gains are quite large (Kp={p_val}, Ki={i_val}, Kd={d_val}). Continue?",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            ) == QtWidgets.QMessageBox.No:
                return

        payload = f"{p_val},{i_val},{d_val}"
        self.client.publish(PID_TOPIC, payload)

        # Store for later reference if needed
        self.P, self.I, self.D = p_val, i_val, d_val

        # Close dialog correctly (avoid recursive self.accept())
        super().accept()