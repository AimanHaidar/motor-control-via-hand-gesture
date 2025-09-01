from PyQt5 import QtWidgets
from gui.generated.speed_monitor_dialog import Ui_Dialog

class SpeedMonitorDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_Dialog()
        self.ui.setupUi(self)