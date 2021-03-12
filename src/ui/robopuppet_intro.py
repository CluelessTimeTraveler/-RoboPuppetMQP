# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'robopuppet_intro.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Robo(object):
    def setupUi(self, Robo):
        Robo.setObjectName("Robo")
        Robo.resize(552, 393)
        self.label = QtWidgets.QLabel(Robo)
        self.label.setGeometry(QtCore.QRect(20, 0, 531, 61))
        self.label.setTextFormat(QtCore.Qt.PlainText)
        self.label.setWordWrap(True)
        self.label.setObjectName("label")
        self.label_2 = QtWidgets.QLabel(Robo)
        self.label_2.setGeometry(QtCore.QRect(130, 70, 281, 311))
        self.label_2.setText("")
        self.label_2.setPixmap(QtGui.QPixmap("pics/robopuppet.jpg"))
        self.label_2.setScaledContents(True)
        self.label_2.setObjectName("label_2")

        self.retranslateUi(Robo)
        QtCore.QMetaObject.connectSlotsByName(Robo)

    def retranslateUi(self, Robo):
        _translate = QtCore.QCoreApplication.translate
        Robo.setWindowTitle(_translate("Robo", "Form"))
        self.label.setText(_translate("Robo", "The RoboPuppet is a model version of the Kinova Robotics’ Kinova Gen 3 arm used for intuitive remote control. The RoboPuppet Arm contains joint angle sensors and motors which allow for gravity compensation. "))

