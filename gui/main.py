from PyQt5.QtWidgets import QApplication, QMainWindow,QMessageBox
from PyQt5 import QtWidgets, QtGui
from PyQt5.QtGui import QFontDatabase, QFont
from generated.main_window import Ui_MainWindow
from dialogs.pid_control_dialog import PidControlDialog
from dialogs.speed_monitor_dialog import SpeedMonitorDialog
class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.ui.set_pid.clicked.connect(self.open_pid_dialog)
        self.ui.show_speed.clicked.connect(self.show_speed)

    def open_pid_dialog(self):
        dialog = PidControlDialog(self)
        dialog.exec_()

    def show_speed(self):
        dialog = SpeedMonitorDialog(self)
        dialog.exec_()
