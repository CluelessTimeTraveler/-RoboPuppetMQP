# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'cam_view.ui'
#
# Created by: PyQt5 UI code generator 5.9.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Camera_View(object):
    def setupUi(self, Camera_View):
        Camera_View.setObjectName("Camera_View")
        Camera_View.resize(899, 514)
        self.tabWidget_6 = QtWidgets.QTabWidget(Camera_View)
        self.tabWidget_6.setGeometry(QtCore.QRect(10, 20, 871, 481))
        self.tabWidget_6.setTabPosition(QtWidgets.QTabWidget.West)
        self.tabWidget_6.setObjectName("tabWidget_6")
        self.tab_25 = QtWidgets.QWidget()
        self.tab_25.setObjectName("tab_25")
        self.main_cam_view = QtWidgets.QLabel(self.tab_25)
        self.main_cam_view.setGeometry(QtCore.QRect(10, 10, 821, 461))
        self.main_cam_view.setAlignment(QtCore.Qt.AlignCenter)
        self.main_cam_view.setObjectName("main_cam_view")
        self.tabWidget_6.addTab(self.tab_25, "")
        self.tab_26 = QtWidgets.QWidget()
        self.tab_26.setObjectName("tab_26")
        self.left_cam_view = QtWidgets.QLabel(self.tab_26)
        self.left_cam_view.setGeometry(QtCore.QRect(10, 10, 821, 461))
        self.left_cam_view.setAlignment(QtCore.Qt.AlignCenter)
        self.left_cam_view.setObjectName("left_cam_view")
        self.tabWidget_6.addTab(self.tab_26, "")
        self.tab_27 = QtWidgets.QWidget()
        self.tab_27.setObjectName("tab_27")
        self.right_cam_view = QtWidgets.QLabel(self.tab_27)
        self.right_cam_view.setGeometry(QtCore.QRect(10, 10, 821, 461))
        self.right_cam_view.setMinimumSize(QtCore.QSize(0, 0))
        self.right_cam_view.setScaledContents(False)
        self.right_cam_view.setAlignment(QtCore.Qt.AlignCenter)
        self.right_cam_view.setObjectName("right_cam_view")
        self.tabWidget_6.addTab(self.tab_27, "")

        self.retranslateUi(Camera_View)
        self.tabWidget_6.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Camera_View)

    def retranslateUi(self, Camera_View):
        _translate = QtCore.QCoreApplication.translate
        Camera_View.setWindowTitle(_translate("Camera_View", "Dialog"))
        self.main_cam_view.setText(_translate("Camera_View", "main_cam_view"))
        self.tabWidget_6.setTabText(self.tabWidget_6.indexOf(self.tab_25), _translate("Camera_View", "Main"))
        self.left_cam_view.setText(_translate("Camera_View", "left_cam_view"))
        self.tabWidget_6.setTabText(self.tabWidget_6.indexOf(self.tab_26), _translate("Camera_View", "Left Arm"))
        self.right_cam_view.setText(_translate("Camera_View", "right_cam_view"))
        self.tabWidget_6.setTabText(self.tabWidget_6.indexOf(self.tab_27), _translate("Camera_View", "Right Arm"))

