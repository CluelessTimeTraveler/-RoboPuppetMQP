#!/usr/bin/env python
import sys
import json
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow
from ui.login_ui import *
import os
from ui.trina2_intro_ui import Ui_Form
from ui.feedback import Ui_FeedbackUI
from constants import *
from PyQt5.QtWidgets import QDialogButtonBox, QWidget
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5 import QtGui
from ui.robopuppet_intro import Ui_Robo


class Login(QMainWindow):
    """Dialog."""

    def __init__(self, parent=None):
        QMainWindow.__init__(self)
        self.main_ui = Ui_MainWindow()
        self.main_ui.setupUi(self)

def login():
    # Get user input
    username = window.main_ui.un_input.text()
    passward = window.main_ui.pw_input.text()

    # Check if the username exist & password correct
    afile = open(path+"/userlist.json","r")
    json_object = json.load(afile)
    afile.close()
    print(json_object)
    desired_un = window.main_ui.un_input.text()
    for i in json_object:
        if i == username:
            if passward == json_object[i]['password']:
                if json_object[i]['user type'] == 'Regular':
                    print('user')
                    window.close()
                    os.popen('rosrun RoboPuppetMQP gui_lite.py')
                else:
                    print('admin')
                    window.close()
                    os.popen('rosrun RoboPuppetMQP gui_pro.py')
            else:
                window.main_ui.login_state.setText('Username or password is not correct. Please try again!')
                window.main_ui.login_state.setStyleSheet("QLabel {color:red;}")
        else:
            window.main_ui.login_state.setText('Username or password is not correct. Please try again!')
            window.main_ui.login_state.setStyleSheet("QLabel {color:red;}")


def signup():
    window.close()
    os.popen('python '+path+'/gui_signup.py')

class trina2_intro_window(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.main_ui = Ui_Form()
        self.main_ui.setupUi(self)

class robopupper_intro_window(QWidget):
    def __init__(self):
        QWidget.__init__(self)
        self.main_ui = Ui_Robo()
        self.main_ui.setupUi(self)



def robo():
    print('t2_intro')
    rb.main_ui.label_2.setPixmap(QtGui.QPixmap(path+"/ui/pics/robopuppet.jpg"))
    rb.setWindowTitle('Robopuppet')
    rb.show()

def trina2_intro():
    print('t2_intro')
    ti.main_ui.label_2.setPixmap(QtGui.QPixmap(path+"/ui/pics/TRINA-WPI-2.0.png"))
    ti.setWindowTitle('TRINA-WPI-2.0')
    ti.show()

def feedback():
    window.close()
    os.popen('python ' + path + '/gui_feedback.py')


if __name__ == '__main__':
    path = os.path.dirname(__file__)
    app = QApplication(sys.argv)
    window = Login()
    icon = QtGui.QIcon()
    icon.addPixmap(QtGui.QPixmap(path+"/ui/pics/Hiro_Logo_WPITheme-300x108.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
    window.main_ui.actionTrina2.setIcon(icon)
    window.setWindowTitle('Robopuppet User Login')
    window.setFixedSize(window.width(), window.height())
    window.show()
    ti = trina2_intro_window()
    rb = robopupper_intro_window()
    window.main_ui.li_btn.clicked.connect(login)
    window.main_ui.su_btn.clicked.connect(signup)
    window.main_ui.actionTrina2.triggered.connect(trina2_intro)
    window.main_ui.actionRobopuppet.triggered.connect(robo)
    window.main_ui.actionFeedback.triggered.connect(feedback)
    sys.exit(app.exec_())
