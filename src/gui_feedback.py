#!/usr/bin/env python
import sys
import json
import time
from PyQt5.QtWidgets import QApplication
from PyQt5.QtWidgets import QMainWindow
from ui.signup_ui import *
import os
from ui.feedback import Ui_FeedbackUI
from constants import *
from PyQt5.QtWidgets import QDialogButtonBox, QWidget
from PyQt5.QtWidgets import QFormLayout
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtWidgets import QVBoxLayout
from PyQt5 import QtGui


class Feedback(QMainWindow):
    """Dialog."""

    def __init__(self, parent=None):
        QMainWindow.__init__(self)
        self.main_ui = Ui_FeedbackUI()
        self.main_ui.setupUi(self)


def submit():
    afile = open("survey_result.json","r")
    json_object = json.load(afile)
    ui = window.main_ui
    result = {
        'name': ui.name.text(),
        'task': ui.task.text(),
        'date': ui.date.text(),
        'evaluation': [ui.md.value(), ui.pd.value(), ui.td.value(), ui.p.value(), ui.e.value(), ui.f.value()]
    }
    json_object['survey'].append(result)

    with open('survey_result.json', 'w') as json_file:
        json.dump(json_object, json_file)

    json_file.close()
    time.sleep(2)
    window.close()
    os.popen('python ' + path + '/gui_login.py')


if __name__ == '__main__':
    path = os.path.dirname(__file__)
    app = QApplication(sys.argv)
    window = Feedback()
    window.setWindowTitle('Feedback Survey')
    window.setFixedSize(window.width(), window.height())
    window.main_ui.pushButton.clicked.connect(submit)
    window.show()
    sys.exit(app.exec_())
