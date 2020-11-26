import sys
import json
import time
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow
from ui.signup_ui import *
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



class trina2_intro_window(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.main_ui = Ui_Form()
        self.main_ui.setupUi(self)


def trina2_intro():
    print('t2_intro')
    ti.setWindowTitle('TRINA-WPI-2.0')
    ti.show()


def signup():
    afile = open("userlist.json","r")
    json_object = json.load(afile)
    afile.close()
    #print(json_object)
    desired_un = window.main_ui.un_input.text()
    for i in json_object:
        if i == desired_un:
            window.main_ui.warning_label.setText('Username already exist!')
            window.main_ui.warning_label.setStyleSheet("QLabel {color:red;}")
            window.main_ui.warning_label.repaint()
            return

    window.main_ui.warning_label.setText('Success!')
    window.main_ui.warning_label.setStyleSheet("QLabel {color:red;}")
    window.main_ui.warning_label.repaint()

    json_object[desired_un] = {
         'password': window.main_ui.pw_input.text(),
         'default robot': '',
         'user type' : window.main_ui.usertype_box.currentText()
         }
    fileJson = open('userlist.json', 'w')
    json.dump(json_object,fileJson)
    fileJson.close()
    time.sleep(2)
    window.close()
    os.popen('python gui_login.py')


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Login()
    window.setWindowTitle('Robopuppet User Sign Up')
    window.setFixedSize(window.width(), window.height())
    window.show()
    ti = trina2_intro_window()
    window.main_ui.actionTrina2.triggered.connect(trina2_intro)
    window.main_ui.su_btn.clicked.connect(signup)
    sys.exit(app.exec_())
