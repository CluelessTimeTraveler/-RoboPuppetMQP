import sys

from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow
from ui.login_ui import *
import os
from ui.trina2_intro_ui import Ui_Form
from constants import *
from PyQt5.QtWidgets import QDialogButtonBox, QWidget
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QVBoxLayout


class Login(QMainWindow):
    """Dialog."""

    def __init__(self, parent=None):
        QMainWindow.__init__(self)
        self.main_ui = Ui_MainWindow()
        self.main_ui.setupUi(self)


def login():
    # ros = ROS()
    if window.main_ui.un_input.text() == "user" and window.main_ui.pw_input.text() == 'user':
        print('user')
        window.close()
        os.popen('rosrun RoboPuppetMQP gui_lite.py')

    elif window.main_ui.pw_input.text() == "admin" and window.main_ui.pw_input.text() == 'admin':
        print('admin')
        window.close()
        os.popen('rosrun RoboPuppetMQP gui_pro.py')

    else:
        window.main_ui.login_state.setText('Username or password is not correct. Please try again!')
        window.main_ui.login_state.setStyleSheet("QLabel {color:red;}")


class trina2_intro_window(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.main_ui = Ui_Form()
        self.main_ui.setupUi(self)


def trina2_intro():
    print('t2_intro')
    ti.setWindowTitle('TRINA-WPI-2.0')
    ti.show()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Login()
    window.setWindowTitle('Robopuppet User Login')
    window.setFixedSize(window.width(), window.height())
    window.show()
    ti = trina2_intro_window()
    window.main_ui.li_btn.clicked.connect(login)
    window.main_ui.actionTrina2.triggered.connect(trina2_intro)
    sys.exit(app.exec_())
