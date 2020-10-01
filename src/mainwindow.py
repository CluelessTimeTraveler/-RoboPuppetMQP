#!/usr/bin/env python

import sys
import random
import numpy as np
from time import sleep
import datetime
from PyQt5 import QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import rospy
from sensor_msgs.msg import Joy
from kortex_driver.msg import *
from kortex_driver.srv import *

global Joint0_Angle
global Joint1_Angle
global Joint2_Angle
global Joint3_Angle
global Joint4_Angle
global Joint5_Angle
global Joint6_Angle

class ROS(QThread):
    def __init__(self):
        rospy.init_node('gui', anonymous=True)
        #rospy.Subscriber('/joy',Joy,self.update,queue_size=1,buff_size=52428800)
        rospy.Subscriber('/my_gen3/base_feedback', BaseCyclic_Feedback, self.update, queue_size=1, buff_size=52428800)

        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
        self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)

        read_action_full_name = '/' + self.robot_name + '/base/read_action'
        rospy.wait_for_service(read_action_full_name)
        self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)

        execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

    def update(self,base_feedback):
        global Joint0_Angle
        global Joint1_Angle
        global Joint2_Angle
        global Joint3_Angle
        global Joint4_Angle
        global Joint5_Angle
        global Joint6_Angle
        Joint0_Angle = base_feedback.actuators[0].position
        Joint1_Angle = base_feedback.actuators[1].position
        Joint2_Angle = base_feedback.actuators[2].position
        Joint3_Angle = base_feedback.actuators[3].position
        Joint4_Angle = base_feedback.actuators[4].position
        Joint5_Angle = base_feedback.actuators[5].position
        Joint6_Angle = base_feedback.actuators[6].position


    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                sleep(0.01)

    def example_home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = 2
        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            # else:
            #     return self.wait_for_action_end_or_abort()


class plotwindows(QtWidgets.QWidget):
    def __init__(self):
        super(plotwindows,self).__init__()
        layout = QFormLayout()
        self.label0 = QLabel()
        self.label1 = QLabel()
        self.label2 = QLabel()
        self.label3 = QLabel()
        self.label4 = QLabel()
        self.label5 = QLabel()
        self.label6 = QLabel()
        self.homeButton = QPushButton("Home Kinova Arm")
        self.homeButton.clicked.connect(self.Home_Robot)

        layout.addRow("Joint0_Angle", self.label0)
        layout.addRow("Joint1_Angle", self.label1)
        layout.addRow("Joint2_Angle", self.label2)
        layout.addRow("Joint3_Angle", self.label3)
        layout.addRow("Joint4_Angle", self.label4)
        layout.addRow("Joint5_Angle", self.label5)
        layout.addRow("Joint6_Angle", self.label6)
        layout.addRow(self.homeButton)
        self.setLayout(layout)

        self.Mytimer()

    def Mytimer(self):
        timer = QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(100)

    def update(self):
        self.label0.setText("%.2f" % Joint0_Angle)
        self.label1.setText("%.2f" % Joint1_Angle)
        self.label2.setText("%.2f" % Joint2_Angle)
        self.label3.setText("%.2f" % Joint3_Angle)
        self.label4.setText("%.2f" % Joint4_Angle)
        self.label5.setText("%.2f" % Joint5_Angle)
        self.label6.setText("%.2f" % Joint6_Angle)

    def Home_Robot(self):
        print(home)
        ros=ROS()
        ros.example_home_the_robot()


def mainwindows():
    app = QtWidgets.QApplication(sys.argv)
    new = plotwindows()
    new.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    global home
    home = False
    ros = ROS()
    mainwindows()







