#!/usr/bin/env python

import os
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
from control_msgs.msg import GripperCommandActionGoal
from kortex_driver.msg import *
from kortex_driver.srv import *
from design import *
from std_msgs.msg import Float32, Float64
from gazebo_msgs.srv import GetJointProperties, GetLinkState
#from kinematics import angleToCP, inverseKinematics
from constants import *
#from PyQt5.QtChart import QCandlestickSeries, QChart, QChartView, QCandlestickSet
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import Qt, QPointF
#from PyQt5 import QtChart as qc
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QLabel, QSlider
from PyQt5.QtCore import QSize, Qt, pyqtSlot,pyqtSignal
from PyQt5.uic import loadUi

from pyqtgraph import PlotWidget, plot
from pyqtgraph import PlotWidget, plot
#from qwt.qt.QtGui import QApplication, QPen
#from qwt.qt.QtCore import Qt
from qwt import QwtPlot, QwtPlotMarker, QwtLegend, QwtPlotCurve, QwtText
global leftAngleList
global rightAngleList
global leftVelocityList
global rightVelocityList
global leftCpList
global rightCpList
global updateControlPanel
global time

# Joint angles for plots
global angleList_lb
global angleList_l1
global angleList_l2
global angleList_l3
global angleList_l4
global angleList_l5
global angleList_l6

global angleList_rb
global angleList_r1
global angleList_r2
global angleList_r3
global angleList_r4
global angleList_r5
global angleList_r6
updateControlPanel = True
leftRecordList = []
rightRecordList = []

class ROS(QThread):
    def __init__(self):
        rospy.init_node('gui', anonymous=True)
        #rospy.Subscriber('/joy',Joy,self.update,queue_size=1,buff_size=52428800)
        #Feedback currently not working for trina2
        #rospy.Subscriber('/left_arm_/base_feedback', BaseCyclic_Feedback, self.leftUpdate, queue_size=1, buff_size=52428800)
        #rospy.Subscriber('/right_arm_/base_feedback', BaseCyclic_Feedback, self.rightUpdate, queue_size=1, buff_size=52428800)

        #get joint angle from Gazebo Service
        joints_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)


        self.rightJoint1 = rospy.Publisher('/right_arm_joint_1_position_controller/command', Float64, queue_size=1)
        self.rightJoint2 = rospy.Publisher('/right_arm_joint_2_position_controller/command', Float64, queue_size=1)
        self.rightJoint3 = rospy.Publisher('/right_arm_joint_3_position_controller/command', Float64, queue_size=1)
        self.rightJoint4 = rospy.Publisher('/right_arm_joint_4_position_controller/command', Float64, queue_size=1)
        self.rightJoint5 = rospy.Publisher('/right_arm_joint_5_position_controller/command', Float64, queue_size=1)
        self.rightJoint6 = rospy.Publisher('/right_arm_joint_6_position_controller/command', Float64, queue_size=1)
        self.rightJoint7 = rospy.Publisher('/right_arm_joint_7_position_controller/command', Float64, queue_size=1)
        self.rightGripper = rospy.Publisher('/right_arm_robotiq_2f_85_gripper_controller/gripper_cmd/goal', GripperCommandActionGoal, queue_size=1)

        self.leftJoint1 = rospy.Publisher('/left_arm_joint_1_position_controller/command', Float64, queue_size=1)
        self.leftJoint2 = rospy.Publisher('/left_arm_joint_2_position_controller/command', Float64, queue_size=1)
        self.leftJoint3 = rospy.Publisher('/left_arm_joint_3_position_controller/command', Float64, queue_size=1)
        self.leftJoint4 = rospy.Publisher('/left_arm_joint_4_position_controller/command', Float64, queue_size=1)
        self.leftJoint5 = rospy.Publisher('/left_arm_joint_5_position_controller/command', Float64, queue_size=1)
        self.leftJoint6 = rospy.Publisher('/left_arm_joint_6_position_controller/command', Float64, queue_size=1)
        self.leftJoint7 = rospy.Publisher('/left_arm_joint_7_position_controller/command', Float64, queue_size=1)
        self.leftGripper = rospy.Publisher('/left_arm_robotiq_2f_85_gripper_controller/gripper_cmd/goal', GripperCommandActionGoal,
                                            queue_size=1)
        # self.action_topic_sub = rospy.Subscriber("/my_gen3" + "/action_topic", ActionNotification,
        #                                          self.cb_action_topic)
        # self.last_action_notif_type = None
        #
        # self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        # self.degrees_of_freedom = rospy.get_param("/" + self.robot_name + "/degrees_of_freedom", 7)
        # self.is_gripper_present = rospy.get_param("/" + self.robot_name + "/is_gripper_present", False)
        #
        # read_action_full_name = '/' + self.robot_name + '/base/read_action'
        # rospy.wait_for_service(read_action_full_name)
        # self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction)
        #
        # execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
        # rospy.wait_for_service(execute_action_full_name)
        # self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)
        #
        # play_cartesian_trajectory_full_name = '/' + self.robot_name + '/base/play_cartesian_trajectory'
        # rospy.wait_for_service(play_cartesian_trajectory_full_name)
        # self.play_cartesian_trajectory = rospy.ServiceProxy(play_cartesian_trajectory_full_name,
        #                                                     PlayCartesianTrajectory)
        #
        # play_joint_trajectory_full_name = '/' + self.robot_name + '/base/play_joint_trajectory'
        # rospy.wait_for_service(play_joint_trajectory_full_name)
        # self.play_joint_trajectory = rospy.ServiceProxy(play_joint_trajectory_full_name, PlayJointTrajectory)


    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event



    def leftUpdate(self):
        global leftAngleList
        global leftVelocityList
        global leftCpList
        global angleList_lb
        global angleList_l1
        global angleList_l2
        global angleList_l3
        global angleList_l4
        global angleList_l5
        global angleList_l6

    def plot(self):
        global angleList_lb
        global angleList_l1
        global angleList_l2
        global angleList_l3
        global angleList_l4
        global angleList_l5
        global angleList_l6

        rospy.wait_for_service('gazebo/get_joint_properties')
        try:
            joints_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
            joint1_properties = joints_properties("left_arm_joint_1")
            ja1 = joint1_properties.position[0]
            joint2_properties = joints_properties("left_arm_joint_2")
            ja2 = joint2_properties.position[0]
            joint3_properties = joints_properties("left_arm_joint_3")
            ja3 = joint3_properties.position[0]
            joint4_properties = joints_properties("left_arm_joint_4")
            ja4 = joint4_properties.position[0]
            joint5_properties = joints_properties("left_arm_joint_5")
            ja5 = joint5_properties.position[0]
            joint6_properties = joints_properties("left_arm_joint_6")
            ja6 = joint6_properties.position[0]
            joint7_properties = joints_properties("left_arm_joint_7")
            ja7 = joint7_properties.position[0]
        except rospy.ServiceException, e:
            print
            "Service call failed: %s" % e

        perSecond = 1000/ui_update_rate
        jv1 = (ja1-leftAngleList[0]) * perSecond
        jv2 = (ja2 - leftAngleList[1]) * perSecond
        jv3 = (ja3 - leftAngleList[2]) * perSecond
        jv4 = (ja4 - leftAngleList[3]) * perSecond
        jv5 = (ja5 - leftAngleList[4]) * perSecond
        jv6 = (ja6 - leftAngleList[5]) * perSecond
        jv7 = (ja7 - leftAngleList[6]) * perSecond

        # if (len(angleList_lb) > 7):
        #     angleList_lb.remove(0)

        angleList_lb[0] = ja1;
        angleList_l1.append(ja2)
        angleList_l2.append(ja3)
        angleList_l3.append(ja4)
        angleList_l4.append(ja5)
        angleList_l5.append(ja6)
        angleList_l6.append(ja7)



        leftAngleList = [ja1,ja2,ja3,ja4,ja5,ja6,ja7]
        leftVelocityList = [jv1,jv2,jv3,jv4,jv5,jv6,jv7]
        leftCpList = angleToCP(leftAngleList)

    def rightUpdate(self):
        global rightAngleList
        global rightVelocityList
        global rightCpList
        global angleList_rb
        global angleList_r1
        global angleList_r2
        global angleList_r3
        global angleList_r4
        global angleList_r5
        global angleList_r6

    def plot(self):
        global angleList_rb
        global angleList_r1
        global angleList_r2
        global angleList_r3
        global angleList_r4
        global angleList_r5
        global angleList_r6
        rospy.wait_for_service('gazebo/get_joint_properties')
        try:
            joints_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
            joint1_properties = joints_properties("right_arm_joint_1")
            ja1 = joint1_properties.position[0]
            joint2_properties = joints_properties("right_arm_joint_2")
            ja2 = joint2_properties.position[0]
            joint3_properties = joints_properties("right_arm_joint_3")
            ja3 = joint3_properties.position[0]
            joint4_properties = joints_properties("right_arm_joint_4")
            ja4 = joint4_properties.position[0]
            joint5_properties = joints_properties("right_arm_joint_5")
            ja5 = joint5_properties.position[0]
            joint6_properties = joints_properties("right_arm_joint_6")
            ja6 = joint6_properties.position[0]
            joint7_properties = joints_properties("right_arm_joint_7")
            ja7 = joint7_properties.position[0]
        except rospy.ServiceException, e:
            print
            "Service call failed: %s" % e

        perSecond = 1000/ui_update_rate
        jv1 = (ja1-rightAngleList[0]) * perSecond
        jv2 = (ja2 - rightAngleList[1]) * perSecond
        jv3 = (ja3 - rightAngleList[2]) * perSecond
        jv4 = (ja4 - rightAngleList[3]) * perSecond
        jv5 = (ja5 - rightAngleList[4]) * perSecond
        jv6 = (ja6 - rightAngleList[5]) * perSecond
        jv7 = (ja7 - rightAngleList[6]) * perSecond

        #if (len(angleList_rb) > 7):
        #    angleList_rb.remove(1)

        angleList_rb.append(ja1)
        angleList_r1.append(ja2)
        angleList_r2.append(ja3)
        angleList_r3.append(ja4)
        angleList_r4.append(ja5)
        angleList_r5.append(ja6)
        angleList_r6.append(ja7)

        rightAngleList = [ja1, ja2, ja3, ja4, ja5, ja6, ja7]
        rightVelocityList = [jv1,jv2,jv3,jv4,jv5,jv6,jv7]
        rightCpList = angleToCP(rightAngleList)

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                updateControlInfo()
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                sleep(0.01)

    def send_joint_angles(self,leftDesireAngle,rightDesireAngle):
        self.leftJoint1.publish(leftDesireAngle[0])
        self.leftJoint2.publish(leftDesireAngle[1])
        self.leftJoint3.publish(leftDesireAngle[2])
        self.leftJoint4.publish(leftDesireAngle[3])
        self.leftJoint5.publish(leftDesireAngle[4])
        self.leftJoint6.publish(leftDesireAngle[5])
        self.leftJoint7.publish(leftDesireAngle[6])

        self.rightJoint1.publish(rightDesireAngle[0])
        self.rightJoint2.publish(rightDesireAngle[1])
        self.rightJoint3.publish(rightDesireAngle[2])
        self.rightJoint4.publish(rightDesireAngle[3])
        self.rightJoint5.publish(rightDesireAngle[4])
        self.rightJoint6.publish(rightDesireAngle[5])
        self.rightJoint7.publish(rightDesireAngle[6])

    def send_gripper_cmd(self,left,right):
        rospy.loginfo("Send Gripper Command")
        lgmsg = GripperCommandActionGoal()
        lgmsg.header.seq=0
        lgmsg.goal.command.position=left
        rgmsg = GripperCommandActionGoal()
        rgmsg.header.seq = 0
        rgmsg.goal.command.position=right
        self.leftGripper.publish(lgmsg)
        self.rightGripper.publish(rgmsg)


    def send_cartesian_pose(self, x, y, z):
        xopt = inverseKinematics([x,y,z])
        rospy.loginfo(xopt[7:])

    def home_the_robot(self):
        rospy.loginfo("Home robot")
        rightArmJointPositions = right_arm_homepos
        leftArmJointPositions = left_arm_homepos
        self.rightJoint1.publish(rightArmJointPositions[0])
        self.rightJoint2.publish(rightArmJointPositions[1])
        self.rightJoint3.publish(rightArmJointPositions[2])
        self.rightJoint4.publish(rightArmJointPositions[3])
        self.rightJoint5.publish(rightArmJointPositions[4])
        self.rightJoint6.publish(rightArmJointPositions[5])
        self.rightJoint7.publish(rightArmJointPositions[6])

        self.leftJoint1.publish(leftArmJointPositions[0])
        self.leftJoint2.publish(leftArmJointPositions[1])
        self.leftJoint3.publish(leftArmJointPositions[2])
        self.leftJoint4.publish(leftArmJointPositions[3])
        self.leftJoint5.publish(leftArmJointPositions[4])
        self.leftJoint6.publish(leftArmJointPositions[5])
        self.leftJoint7.publish(leftArmJointPositions[6])



def updateControlInfo():
    #Update Control Panel
    window.lineEdit_4.setText("%.2f" % leftAngleList[0])
    window.lineEdit_5.setText("%.2f" % leftAngleList[1])
    window.lineEdit_6.setText("%.2f" % leftAngleList[2])
    window.lineEdit_7.setText("%.2f" % leftAngleList[3])
    window.lineEdit_8.setText("%.2f" % leftAngleList[4])
    window.lineEdit_9.setText("%.2f" % leftAngleList[5])
    window.lineEdit_10.setText("%.2f" % leftAngleList[6])

    window.lineEdit_11.setText("%.2f" % rightAngleList[0])
    window.lineEdit_12.setText("%.2f" % rightAngleList[1])
    window.lineEdit_13.setText("%.2f" % rightAngleList[2])
    window.lineEdit_14.setText("%.2f" % rightAngleList[3])
    window.lineEdit_15.setText("%.2f" % rightAngleList[4])
    window.lineEdit_16.setText("%.2f" % rightAngleList[5])
    window.lineEdit_17.setText("%.2f" % rightAngleList[6])



def updateInfo():
    #Left
    #Angles
    ros.leftUpdate()
    ros.rightUpdate()

    window.label_15.setText("%.2f" % leftAngleList[0])
    window.label_16.setText("%.2f" % leftAngleList[1])
    window.label_17.setText("%.2f" % leftAngleList[2])
    window.label_18.setText("%.2f" % leftAngleList[3])
    window.label_19.setText("%.2f" % leftAngleList[4])
    window.label_20.setText("%.2f" % leftAngleList[5])
    window.label_21.setText("%.2f" % leftAngleList[6])
    #Velocity
    window.label_8.setText("%.2f" % leftVelocityList[0])
    window.label_9.setText("%.2f" % leftVelocityList[1])
    window.label_10.setText("%.2f" % leftVelocityList[2])
    window.label_11.setText("%.2f" % leftVelocityList[3])
    window.label_12.setText("%.2f" % leftVelocityList[4])
    window.label_13.setText("%.2f" % leftVelocityList[5])
    window.label_14.setText("%.2f" % leftVelocityList[6])
    #Cartesian pose
    window.label_35.setText("x_pose: %.2f" % leftCpList[0])
    window.label_36.setText("y_pose: %.2f" % leftCpList[1])
    window.label_37.setText("z_pose: %.2f" % leftCpList[2])

    #Right
    #Angles
    window.label_52.setText("%.2f" % rightAngleList[0])
    window.label_53.setText("%.2f" % rightAngleList[1])
    window.label_54.setText("%.2f" % rightAngleList[2])
    window.label_55.setText("%.2f" % rightAngleList[3])
    window.label_56.setText("%.2f" % rightAngleList[4])
    window.label_57.setText("%.2f" % rightAngleList[5])
    window.label_58.setText("%.2f" % rightAngleList[6])
    #Velocity
    window.label_60.setText("%.2f" % rightVelocityList[0])
    window.label_61.setText("%.2f" % rightVelocityList[1])
    window.label_62.setText("%.2f" % rightVelocityList[2])
    window.label_63.setText("%.2f" % rightVelocityList[3])
    window.label_64.setText("%.2f" % rightVelocityList[4])
    window.label_65.setText("%.2f" % rightVelocityList[5])
    window.label_66.setText("%.2f" % rightVelocityList[6])
    #Cartesian pose
    window.label_68.setText("x_pose: %.2f" % rightCpList[0])
    window.label_69.setText("y_pose: %.2f" % rightCpList[1])
    window.label_70.setText("z_pose: %.2f" % rightCpList[2])

def home_robot():
    ros.home_the_robot()

def send_cartesian_pose():
    ros.send_cartesian_pose(float(window.lineEdit.text()),float(window.lineEdit_2.text()),float(window.lineEdit_3.text()))
    #ros.example_send_cartesian_pose(0,0,0)


def send_joint_angles():
    lda0 = float(window.lineEdit_4.text())
    lda1 = float(window.lineEdit_5.text())
    lda2 = float(window.lineEdit_6.text())
    lda3 = float(window.lineEdit_7.text())
    lda4 = float(window.lineEdit_8.text())
    lda5 = float(window.lineEdit_9.text())
    lda6 = float(window.lineEdit_10.text())
    left_daList = [lda0,lda1,lda2,lda3,lda4,lda5,lda6]

    rda0 = float(window.lineEdit_11.text())
    rda1 = float(window.lineEdit_12.text())
    rda2 = float(window.lineEdit_13.text())
    rda3 = float(window.lineEdit_14.text())
    rda4 = float(window.lineEdit_15.text())
    rda5 = float(window.lineEdit_16.text())
    rda6 = float(window.lineEdit_17.text())
    right_daList = [rda0,rda1,rda2,rda3,rda4,rda5,rda6]

    ros.send_joint_angles(left_daList,right_daList)

def sendGripperCmd():
    ros.send_gripper_cmd(window.leftGripperSlider.value()/10.,window.rightGripperSlider.value()/10.)

def recordAngle():
    leftRecordList.append(leftAngleList)
    rightRecordList.append(rightAngleList)
    rospy.loginfo("recording")

def recordCP():
    recordList.append(cpList)
    rospy.loginfo("recording")

def record():
    leftRecordList=[]
    rightRecordList=[]
    record_timer.start(record_rate)    # Record angeles every 1 second

def stop():
    record_timer.stop()
    rospy.loginfo("recording terminated")

def play():
    rospy.loginfo("playing")
    for i in range(len(leftRecordList)):
        ros.send_joint_angles(leftRecordList[i],rightRecordList[i])
        sleep(1)
        #ros.send_cartesian_pose(i[0],i[1],i[2])

def plot():
    window.angleGraph_lb.plot(time, angleList_lb)
    window.angleGraph_l1.plot(time, angleList_l1)
    window.angleGraph_l2.plot(time, angleList_l2)
    window.angleGraph_l4.plot(time, angleList_l3)
    window.angleGraph_l5.plot(time, angleList_l4)
    window.angleGraph_l5.plot(time, angleList_l5)
    window.angleGraph_l6.plot(time, angleList_l6)

    window.angleGraph_rb.plot(time, angleList_rb)
    window.angleGraph_r1.plot(time, angleList_r1)
    window.angleGraph_r2.plot(time, angleList_r2)
    window.angleGraph_r3.plot(time, angleList_r3)
    window.angleGraph_r4.plot(time, angleList_r4)
    window.angleGraph_r5.plot(time, angleList_r5)
    window.angleGraph_r6.plot(time, angleList_r6)

if __name__ == "__main__":
    recording = False
    leftAngleList=[0,0,0,0,0,0,0]
    rightAngleList=[0,0,0,0,0,0,0]
    time=[-6,-5, -4, -3, -2, -1, 0]
    angleList_lb = [0, 0, 0, 0, 0, 0, 0]
    angleList_l1 = [0, 0, 0, 0, 0, 0, 0]
    angleList_l2 = [0, 0, 0, 0, 0, 0, 0]
    angleList_l3 = [0, 0, 0, 0, 0, 0, 0]
    angleList_l4 = [0, 0, 0, 0, 0, 0, 0]
    angleList_l5 = [0, 0, 0, 0, 0, 0, 0]
    angleList_l6 = [0, 0, 0, 0, 0, 0, 0]

    angleList_rb = [0, 0, 0, 0, 0, 0, 0]
    angleList_r1 = [0, 0, 0, 0, 0, 0, 0]
    angleList_r2 = [0, 0, 0, 0, 0, 0, 0]
    angleList_r3 = [0, 0, 0, 0, 0, 0, 0]
    angleList_r4 = [0, 0, 0, 0, 0, 0, 0]
    angleList_r5 = [0, 0, 0, 0, 0, 0, 0]
    angleList_r6 = [0, 0, 0, 0, 0, 0, 0]

    ros = ROS()
    app = QtWidgets.QApplication(sys.argv)
    widget = QtWidgets.QWidget()
    window = Ui_Dialog()
    window.setupUi(widget)
    window.pushButton.clicked.connect(send_cartesian_pose)
    window.pushButton_2.clicked.connect(home_robot)
    window.setAngleButton.clicked.connect(send_joint_angles)
    window.refreshButton.clicked.connect(updateControlInfo)
    window.recordButton.clicked.connect(record)
    window.stopButton.clicked.connect(stop)
    window.playButton.clicked.connect(play)
    ros.send_gripper_cmd(0,0)

    window.leftGripperSlider.valueChanged.connect(sendGripperCmd)
    window.rightGripperSlider.valueChanged.connect(sendGripperCmd)

    window.lineEdit.setText("0")
    window.lineEdit_2.setText("0")
    window.lineEdit_3.setText("0")
    updateControlInfo()
    widget.show()
    timer = QTimer()
    record_timer = QTimer()
    timer.timeout.connect(updateInfo)
    timer.timeout.connect(plot)
    record_timer.timeout.connect(recordAngle)
    timer.start(ui_update_rate)

    sys.exit(app.exec_())
