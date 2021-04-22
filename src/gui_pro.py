#!/usr/bin/env python

import sys
import random
from time import sleep
import datetime
from PyQt5 import QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import rospy
import math
from sensor_msgs.msg import Joy
from control_msgs.msg import GripperCommandActionGoal
from kortex_driver.msg import *
from kortex_driver.srv import *
from ui.gui_pro_ui import *
from std_msgs.msg import Float32, Float64
from gazebo_msgs.srv import GetJointProperties, GetLinkState
from kinematics import angleToCP
from constants import *
from PyQt5 import QtCore, QtWidgets
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from msg_arduino.msg import JointPositions
from msg_arduino.msg import LeftArmPositions
from msg_arduino.msg import RightArmPositions
#import qtmodern.styles
#import qtmodern.windows


global leftAngleList
global rightAngleList
global leftVelocityList
global rightVelocityList
global leftCpList
global rightCpList
global updateControlPanel
global leftMode
global rightMode
global states
global mirror
global plot_time  # Jason
global left_cv_image,right_cv_image,main_cv_image
global playcount, playi, play_totalcount
global angleList_lb, angleList_l1, angleList_l2, angleList_l3, angleList_l4, angleList_l5, angleList_l6
global angleList_rb, angleList_r1, angleList_r2, angleList_r3, angleList_r4, angleList_r5, angleList_r6
updateControlPanel = True

#Used to update GUI
leftRecordList = []
leftGripperPosition = 0

rightRecordList = []
rightGripperPosition = 0

##Used to save recieved values from the physical arm
savedLeftAngles = []
savedRightAngles = []

class ROS(QThread):
    def __init__(self):
        rospy.init_node('gui', anonymous=True)
        #rospy.Subscriber('/joy',Joy,self.robopuppet,queue_size=1,buff_size=52428800)
        # Subscribe to Robopuppet

        #rospy.Subscriber('/potAngles', JointPositions, self.robopuppet, queue_size=1)
        
        rospy.Subscriber('/LeftArm', LeftArmPositions, self.leftArmSaveData, queue_size=1)
        rospy.Subscriber('/RightArm', RightArmPositions, self.rightArmSaveData, queue_size=1)

        # Subscribe to cameras on the robot
        rospy.Subscriber(robot_prefix + '/right_arm_cam/color/image_raw', Image, self.right_image, queue_size=1)
        rospy.Subscriber(robot_prefix + '/left_arm_cam/color/image_raw', Image, self.left_image, queue_size=1)
        rospy.Subscriber(robot_prefix + '/main_cam/color/image_raw', Image, self.main_image, queue_size=1)
        self.bridge = CvBridge()
        # get joint angle from Gazebo Service
        joints_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
        # Control each joint of the arm via publisher
        self.rightJoint1 = rospy.Publisher('/' + robot_prefix + '/right_arm_joint_1_position_controller/command',
                                           Float64, queue_size=1)
        self.rightJoint2 = rospy.Publisher('/' + robot_prefix + '/right_arm_joint_2_position_controller/command',
                                           Float64, queue_size=1)
        self.rightJoint3 = rospy.Publisher('/' + robot_prefix + '/right_arm_joint_3_position_controller/command',
                                           Float64, queue_size=1)
        self.rightJoint4 = rospy.Publisher('/' + robot_prefix + '/right_arm_joint_4_position_controller/command',
                                           Float64, queue_size=1)
        self.rightJoint5 = rospy.Publisher('/' + robot_prefix + '/right_arm_joint_5_position_controller/command',
                                           Float64, queue_size=1)
        self.rightJoint6 = rospy.Publisher('/' + robot_prefix + '/right_arm_joint_6_position_controller/command',
                                           Float64, queue_size=1)
        self.rightJoint7 = rospy.Publisher('/' + robot_prefix + '/right_arm_joint_7_position_controller/command',
                                           Float64, queue_size=1)
        self.rightGripper = rospy.Publisher('/' + robot_prefix + '/right_arm_robotiq_2f_85_gripper_controller/gripper_cmd/goal',GripperCommandActionGoal, queue_size=1)

        self.leftJoint1 = rospy.Publisher('/' + robot_prefix + '/left_arm_joint_1_position_controller/command', Float64,
                                          queue_size=1)
        self.leftJoint2 = rospy.Publisher('/' + robot_prefix + '/left_arm_joint_2_position_controller/command', Float64,
                                          queue_size=1)
        self.leftJoint3 = rospy.Publisher('/' + robot_prefix + '/left_arm_joint_3_position_controller/command', Float64,
                                          queue_size=1)
        self.leftJoint4 = rospy.Publisher('/' + robot_prefix + '/left_arm_joint_4_position_controller/command', Float64,
                                          queue_size=1)
        self.leftJoint5 = rospy.Publisher('/' + robot_prefix + '/left_arm_joint_5_position_controller/command', Float64,
                                          queue_size=1)
        self.leftJoint6 = rospy.Publisher('/' + robot_prefix + '/left_arm_joint_6_position_controller/command', Float64,
                                          queue_size=1)
        self.leftJoint7 = rospy.Publisher('/' + robot_prefix + '/left_arm_joint_7_position_controller/command', Float64,
                                          queue_size=1)
        self.leftGripper = rospy.Publisher('/' + robot_prefix + '/left_arm_robotiq_2f_85_gripper_controller/gripper_cmd/goal',GripperCommandActionGoal,queue_size=1)
    # convert raw camera data using CV
    def left_image(self,data):
        global left_cv_image
        left_cv_image = self.bridge.imgmsg_to_cv2(data,'rgb8')

    def right_image(self,data):
        global right_cv_image
        right_cv_image = self.bridge.imgmsg_to_cv2(data,'rgb8')

    def main_image(self,data):
        global main_cv_image
        main_cv_image = self.bridge.imgmsg_to_cv2(data,'rgb8')

    # This contorls the arm
    def publish_to_left(self, angles):
        self.leftJoint1.publish(angles[0])
        self.leftJoint2.publish(angles[1])
        self.leftJoint3.publish(angles[2])
        self.leftJoint4.publish(angles[3])
        self.leftJoint5.publish(angles[4])
        self.leftJoint6.publish(angles[5])
        self.leftJoint7.publish(angles[6])

    def publish_to_right(self, angles):
        self.rightJoint1.publish(angles[0])
        self.rightJoint2.publish(angles[1])
        self.rightJoint3.publish(angles[2])
        self.rightJoint4.publish(angles[3])
        self.rightJoint5.publish(angles[4])
        self.rightJoint6.publish(angles[5])
        self.rightJoint7.publish(angles[6])

    def send_joint_angles(self, leftDesireAngle, rightDesireAngle):
        self.publish_to_left(leftDesireAngle)
        self.publish_to_right(rightDesireAngle)

    def updateRobopuppet(self):
        global savedLeftAngles
        global savedRightAngles
        global leftGripperPosition
        global rightGripperPosition
        print("Size of savedRightAngles " + str(len(savedRightAngles)))
        
        c = math.pi/180
        if states == 'Enabled':
            if mirror:
                angles_right = []
                for i in savedLeftAngles:
                    angles_right.append(-i)
            ##else:
                ##angles_right = angles

            # if leftMode & rightMode:
            #     self.send_gripper_cmd(leftGripperPosition, rightGripperPosition)
            #     self.publish_to_left(savedLeftAngles)
            #     self.publish_to_right(savedRightAngles)
            # elif leftMode:
            #     self.send_gripper_cmd(leftGripperPosition, 0)
            #     self.publish_to_left(savedLeftAngles)
            # else:
            #     self.send_gripper_cmd(0, rightGripperPosition)
            #     self.publish_to_right(savedRightAngles)


    def leftArmSaveData(self, data):
        c = math.pi/180
        global savedLeftAngles
        global leftGripperPosition

        savedLeftAngles = [data.servoOne*c, data.servoTwo*c, data.servoThree*c, data.encoderOne*c, data.encoderTwo*c,
                      data.encoderThree*c, data.encoderFour*c]  # angles from robopuppet's left arm
        leftGripperPosition = data.gripperToggle

        if leftMode:
            self.send_gripper_cmd(leftGripperPosition, rightGripperPosition)
            self.publish_to_left(savedLeftAngles)

        # self.updateRobopuppet()

    def rightArmSaveData(self, data):
        c = math.pi/180
        global savedRightAngles
        global rightGripperPosition
        savedRightAngles = [data.servoOne*c, data.servoTwo*c, data.servoThree*c, data.encoderOne*c, data.encoderTwo*c,
                      data.encoderThree*c, data.encoderFour*c]  # angles from robopuppet's right arm
        rightGripperPosition = data.gripperToggle
        
        if rightMode:    
            self.send_gripper_cmd(leftGripperPosition, rightGripperPosition)
            self.publish_to_right(savedRightAngles)
        
        
        # self.updateRobopuppet()


    # This updates the information for GUI
    def leftUpdate(self):
        global leftAngleList
        global leftVelocityList
        global leftCpList
        global angleList_lb, angleList_l1, angleList_l2, angleList_l3, angleList_l4, angleList_l5, angleList_l6
        rospy.wait_for_service('gazebo/get_joint_properties')
        try:
            joints_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
            joint1_properties = joints_properties(robot_prefix+"/left_arm_joint_1")
            ja1 = joint1_properties.position[0]
            joint2_properties = joints_properties(robot_prefix+"/left_arm_joint_2")
            ja2 = joint2_properties.position[0]
            joint3_properties = joints_properties(robot_prefix+"/left_arm_joint_3")
            ja3 = joint3_properties.position[0]
            joint4_properties = joints_properties(robot_prefix+"/left_arm_joint_4")
            ja4 = joint4_properties.position[0]
            joint5_properties = joints_properties(robot_prefix+"/left_arm_joint_5")
            ja5 = joint5_properties.position[0]
            joint6_properties = joints_properties(robot_prefix+"/left_arm_joint_6")
            ja6 = joint6_properties.position[0]
            joint7_properties = joints_properties(robot_prefix+"/left_arm_joint_7")
            ja7 = joint7_properties.position[0]
        except rospy.ServiceException as e:
            print
            "Service call failed: %s" % e

        perSecond = 1000 / ui_update_rate
        jv1 = (ja1 - leftAngleList[0]) * perSecond
        jv2 = (ja2 - leftAngleList[1]) * perSecond
        jv3 = (ja3 - leftAngleList[2]) * perSecond
        jv4 = (ja4 - leftAngleList[3]) * perSecond
        jv5 = (ja5 - leftAngleList[4]) * perSecond
        jv6 = (ja6 - leftAngleList[5]) * perSecond
        jv7 = (ja7 - leftAngleList[6]) * perSecond

        leftAngleList = [ja1, ja2, ja3, ja4, ja5, ja6, ja7]
        leftVelocityList = [jv1, jv2, jv3, jv4, jv5, jv6, jv7]
        leftCpList = angleToCP(leftAngleList)

    def rightUpdate(self):
        global rightAngleList
        global rightVelocityList
        global rightCpList
        global angleList_rb, angleList_r1, angleList_r2, angleList_r3, angleList_r4, angleList_r5, angleList_r6
        rospy.wait_for_service('gazebo/get_joint_properties')
        try:
            joints_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
            joint1_properties = joints_properties(robot_prefix+"/right_arm_joint_1")
            ja1 = joint1_properties.position[0]
            joint2_properties = joints_properties(robot_prefix+"/right_arm_joint_2")
            ja2 = joint2_properties.position[0]
            joint3_properties = joints_properties(robot_prefix+"/right_arm_joint_3")
            ja3 = joint3_properties.position[0]
            joint4_properties = joints_properties(robot_prefix+"/right_arm_joint_4")
            ja4 = joint4_properties.position[0]
            joint5_properties = joints_properties(robot_prefix+"/right_arm_joint_5")
            ja5 = joint5_properties.position[0]
            joint6_properties = joints_properties(robot_prefix+"/right_arm_joint_6")
            ja6 = joint6_properties.position[0]
            joint7_properties = joints_properties(robot_prefix+"/right_arm_joint_7")
            ja7 = joint7_properties.position[0]
        except rospy.ServiceException as e:
            print
            "Service call failed: %s" % e

        perSecond = 1000 / ui_update_rate
        jv1 = (ja1 - rightAngleList[0]) * perSecond
        jv2 = (ja2 - rightAngleList[1]) * perSecond
        jv3 = (ja3 - rightAngleList[2]) * perSecond
        jv4 = (ja4 - rightAngleList[3]) * perSecond
        jv5 = (ja5 - rightAngleList[4]) * perSecond
        jv6 = (ja6 - rightAngleList[5]) * perSecond
        jv7 = (ja7 - rightAngleList[6]) * perSecond

        rightAngleList = [ja1, ja2, ja3, ja4, ja5, ja6, ja7]
        rightVelocityList = [jv1, jv2, jv3, jv4, jv5, jv6, jv7]
        rightCpList = angleToCP(rightAngleList)



    def send_gripper_cmd(self, left, right):
        #rospy.loginfo("Send Gripper Command")
        lgmsg = GripperCommandActionGoal()
        lgmsg.header.seq = 0
        lgmsg.goal.command.position = left
        rgmsg = GripperCommandActionGoal()
        rgmsg.header.seq = 0
        rgmsg.goal.command.position = right
        self.leftGripper.publish(lgmsg)
        self.rightGripper.publish(rgmsg)

    def home_the_robot(self):
        rospy.loginfo("Home robot")
        rightArmJointPositions = right_arm_homepos
        leftArmJointPositions = left_arm_homepos
        self.publish_to_right(rightArmJointPositions)
        self.publish_to_left(leftArmJointPositions)


def updateControlInfo():
    # Update Control Panel
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

# This actually updates the GUI
def updateInfo():
    # Left
    # Angles
    ros.leftUpdate()
    ros.rightUpdate()

    window.label_15.setText("%.2f" % leftAngleList[0])
    window.label_16.setText("%.2f" % leftAngleList[1])
    window.label_17.setText("%.2f" % leftAngleList[2])
    window.label_18.setText("%.2f" % leftAngleList[3])
    window.label_19.setText("%.2f" % leftAngleList[4])
    window.label_20.setText("%.2f" % leftAngleList[5])
    window.label_21.setText("%.2f" % leftAngleList[6])
    # Velocity
    window.label_8.setText("%.2f" % leftVelocityList[0])
    window.label_9.setText("%.2f" % leftVelocityList[1])
    window.label_10.setText("%.2f" % leftVelocityList[2])
    window.label_11.setText("%.2f" % leftVelocityList[3])
    window.label_12.setText("%.2f" % leftVelocityList[4])
    window.label_13.setText("%.2f" % leftVelocityList[5])
    window.label_14.setText("%.2f" % leftVelocityList[6])
    # Cartesian pose
    window.label_35.setText("x_pose: %.2f" % leftCpList[0])
    window.label_36.setText("y_pose: %.2f" % leftCpList[1])
    window.label_37.setText("z_pose: %.2f" % leftCpList[2])
    window.label_38.setText("x_pose: %.2f" % leftCpList[0])
    window.label_39.setText("y_pose: %.2f" % leftCpList[1])
    window.label_40.setText("z_pose: %.2f" % leftCpList[2])

    # Right
    # Angles
    window.label_52.setText("%.2f" % rightAngleList[0])
    window.label_53.setText("%.2f" % rightAngleList[1])
    window.label_54.setText("%.2f" % rightAngleList[2])
    window.label_55.setText("%.2f" % rightAngleList[3])
    window.label_56.setText("%.2f" % rightAngleList[4])
    window.label_57.setText("%.2f" % rightAngleList[5])
    window.label_58.setText("%.2f" % rightAngleList[6])
    # Velocity
    window.label_60.setText("%.2f" % rightVelocityList[0])
    window.label_61.setText("%.2f" % rightVelocityList[1])
    window.label_62.setText("%.2f" % rightVelocityList[2])
    window.label_63.setText("%.2f" % rightVelocityList[3])
    window.label_64.setText("%.2f" % rightVelocityList[4])
    window.label_65.setText("%.2f" % rightVelocityList[5])
    window.label_66.setText("%.2f" % rightVelocityList[6])
    # Cartesian pose
    window.label_68.setText("x_pose: %.2f" % rightCpList[0])
    window.label_69.setText("y_pose: %.2f" % rightCpList[1])
    window.label_70.setText("z_pose: %.2f" % rightCpList[2])
    window.label_77.setText("x_pose: %.2f" % rightCpList[0])
    window.label_78.setText("y_pose: %.2f" % rightCpList[1])
    window.label_79.setText("z_pose: %.2f" % rightCpList[2])

    height, width, channel = main_cv_image.shape
    bytesPerLine = 3 * width
    main_qImg = QImage(main_cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)
    main_qImg = main_qImg.scaledToWidth(cam_width)
    window.main_cam_view.setPixmap(QPixmap(main_qImg))

    height, width, channel = left_cv_image.shape
    bytesPerLine = 3 * width
    left_qImg = QImage(left_cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)
    left_qImg = left_qImg.scaledToWidth(cam_width)
    window.left_cam_view.setPixmap(QPixmap(left_qImg))

    height, width, channel = right_cv_image.shape
    bytesPerLine = 3 * width
    right_qImg = QImage(right_cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)
    right_qImg = right_qImg.scaledToWidth(cam_width)
    window.right_cam_view.setPixmap(QPixmap(right_qImg))

def home_robot():
    ros.home_the_robot()


def send_cartesian_pose():
    ros.send_cartesian_pose(float(window.lineEdit.text()), float(window.lineEdit_2.text()),
                            float(window.lineEdit_3.text()))
    # ros.example_send_cartesian_pose(0,0,0)


def send_joint_angles():
    lda0 = float(window.lineEdit_4.text())
    lda1 = float(window.lineEdit_5.text())
    lda2 = float(window.lineEdit_6.text())
    lda3 = float(window.lineEdit_7.text())
    lda4 = float(window.lineEdit_8.text())
    lda5 = float(window.lineEdit_9.text())
    lda6 = float(window.lineEdit_10.text())
    left_daList = [lda0, lda1, lda2, lda3, lda4, lda5, lda6]

    rda0 = float(window.lineEdit_11.text())
    rda1 = float(window.lineEdit_12.text())
    rda2 = float(window.lineEdit_13.text())
    rda3 = float(window.lineEdit_14.text())
    rda4 = float(window.lineEdit_15.text())
    rda5 = float(window.lineEdit_16.text())
    rda6 = float(window.lineEdit_17.text())
    right_daList = [rda0, rda1, rda2, rda3, rda4, rda5, rda6]

    ros.send_joint_angles(left_daList, right_daList)


def sendGripperCmd():
    ros.send_gripper_cmd(window.leftGripperSlider.value() / 10., window.rightGripperSlider.value() / 10.)


def recordAngle():
    leftRecordList.append(leftAngleList)
    rightRecordList.append(rightAngleList)
    rospy.loginfo("recording")


def record():
    leftRecordList = []
    rightRecordList = []
    record_timer.start(record_rate)  # Record angeles every 1 second


def stop():
    record_timer.stop()
    play_timer.stop()
    rospy.loginfo("recording terminated")
    states = 'Enabled'
    color = 'green'
    window.label_72.setStyleSheet("QLabel {color:" + color + ";}")


def play():
    global states,playi,playcount, play_totalcount
    states = 'Disabled'
    color = 'red'
    window.label_72.setStyleSheet("QLabel {color:" + color + ";}")
    rospy.loginfo("playing")
    play_timer.start(record_rate)
    play_totalcount = int(window.lineEdit.text())
    playi = 0
    playcount = 0


def playing():
    global playi, playcount
    ros.send_joint_angles(leftRecordList[playi], rightRecordList[playi])
    playi += 1
    if playi == len(leftRecordList):
        playi = 0
        playcount += 1
    if playcount == play_totalcount:
        play_timer.stop()
        return


def estimate():
    lda0 = float(window.lineEdit_4.text())
    lda1 = float(window.lineEdit_5.text())
    lda2 = float(window.lineEdit_6.text())
    lda3 = float(window.lineEdit_7.text())
    lda4 = float(window.lineEdit_8.text())
    lda5 = float(window.lineEdit_9.text())
    lda6 = float(window.lineEdit_10.text())
    left_daList = [lda0, lda1, lda2, lda3, lda4, lda5, lda6]

    rda0 = float(window.lineEdit_11.text())
    rda1 = float(window.lineEdit_12.text())
    rda2 = float(window.lineEdit_13.text())
    rda3 = float(window.lineEdit_14.text())
    rda4 = float(window.lineEdit_15.text())
    rda5 = float(window.lineEdit_16.text())
    rda6 = float(window.lineEdit_17.text())
    right_daList = [rda0, rda1, rda2, rda3, rda4, rda5, rda6]

    leftEstCpList = angleToCP(left_daList)
    rightEstCpList = angleToCP(right_daList)

    window.label_75.setText("x_pose: %.2f" % leftEstCpList[0])
    window.label_76.setText("y_pose: %.2f" % leftEstCpList[1])
    window.label_80.setText("z_pose: %.2f" % leftEstCpList[2])

    window.label_82.setText("x_pose: %.2f" % rightEstCpList[0])
    window.label_137.setText("y_pose: %.2f" % rightEstCpList[1])
    window.label_138.setText("z_pose: %.2f" % rightEstCpList[2])


def set_RP_mode():
    if window.radioButton.isChecked():
        global leftMode, rightMode, mirror
        leftMode = True
        rightMode = False
    elif window.radioButton_2.isChecked():
        global leftMode, rightMode
        leftMode = False
        rightMode = True
    else:
        global leftMode, rightMode
        leftMode = True
        rightMode = True
        if window.checkBox.isChecked():
            mirror = True
        else:
            mirror = False


def connect_RP():
    global states
    color = 'black'
    if states == 'Enabled':
        states = 'Disabled'
        color = 'red'
    elif states == 'Disabled':
        states = 'Enabled'
        color = 'green'
    window.label_72.setText(states)
    window.label_72.setStyleSheet("QLabel {color:" + color + ";}")


# For Jason
def plot():
    global ui_update_rate, plot_time, rightAngleList, leftAngleList
    global angleList_lb, angleList_l1, angleList_l2, angleList_l3, angleList_l4, angleList_l5, angleList_l6
    global angleList_rb, angleList_r1, angleList_r2, angleList_r3, angleList_r4, angleList_r5, angleList_r6

    size = len(plot_time)
    plot_time.append(size)
    angleList_lb.append(leftAngleList[0])
    angleList_l1.append(leftAngleList[1])
    angleList_l2.append(leftAngleList[2])
    angleList_l3.append(leftAngleList[3])
    angleList_l4.append(leftAngleList[4])
    angleList_l5.append(leftAngleList[5])
    angleList_l6.append(leftAngleList[6])

    angleList_rb.append(rightAngleList[0])
    angleList_r1.append(rightAngleList[1])
    angleList_r2.append(rightAngleList[2])
    angleList_r3.append(rightAngleList[3])
    angleList_r4.append(rightAngleList[4])
    angleList_r5.append(rightAngleList[5])
    angleList_r6.append(rightAngleList[6])

    if len(plot_time) > 20:
        #plot_time.pop(0)
        plot_time = list(range(20))
    if (len(angleList_lb) > 20):
        #plot_time.insert(0)
        angleList_lb.pop(0)
        angleList_l1.pop(0)
        angleList_l2.pop(0)
        angleList_l3.pop(0)
        angleList_l4.pop(0)
        angleList_l5.pop(0)
        angleList_l6.pop(0)
        angleList_rb.pop(0)
        angleList_r1.pop(0)
        angleList_r2.pop(0)
        angleList_r3.pop(0)
        angleList_r4.pop(0)
        angleList_r5.pop(0)
        angleList_r6.pop(0)


        # for x in plot_time:
        #     angleList_lb.pop(x)
        #     angleList_l1.pop(x)
        #     angleList_l2.pop(x)
        #     angleList_l3.pop(x)
        #     angleList_l4.pop(x)
        #     angleList_l5.pop(x)
        #     angleList_l6.pop(x)
        #     angleList_rb.pop(x)
        #     angleList_r1.pop(x)
        #     angleList_r2.pop(x)
        #     angleList_r3.pop(x)
        #     angleList_r4.pop(x)
        #     angleList_r5.pop(x)
        #     angleList_r6.pop(x)

    window.angleGraph_lb.setRange(rect=None, xRange=None, yRange = (-3, 3))
    window.angleGraph_l1.setRange(rect=None, xRange=None, yRange = (-3, 3))
    window.angleGraph_l2.setRange(rect=None, xRange=None, yRange = (-3, 3))
    window.angleGraph_l3.setRange(rect=None, xRange=None, yRange = (-3, 3))
    window.angleGraph_l4.setRange(rect=None, xRange=None, yRange = (-3, 3))
    window.angleGraph_l5.setRange(rect=None, xRange=None, yRange = (-3, 3))
    window.angleGraph_l6.setRange(rect=None, xRange=None, yRange = (-3, 3))

    window.angleGraph_rb.setRange(rect=None, xRange=None, yRange = (-3, 3))
    window.angleGraph_r1.setRange(rect=None, xRange=None, yRange = (-3, 3))
    window.angleGraph_r2.setRange(rect=None, xRange=None, yRange = (-3, 3))
    window.angleGraph_r3.setRange(rect=None, xRange=None, yRange = (-3, 3))
    window.angleGraph_r4.setRange(rect=None, xRange=None, yRange = (-3, 3))
    window.angleGraph_r5.setRange(rect=None, xRange=None, yRange = (-3, 3))
    window.angleGraph_r6.setRange(rect=None, xRange=None, yRange = (-3, 3))

    window.angleGraph_lb.clear()
    window.angleGraph_lb.plot(plot_time, angleList_lb)
    window.angleGraph_l1.clear()
    window.angleGraph_l1.plot(plot_time, angleList_l1)
    window.angleGraph_l2.clear()
    window.angleGraph_l2.plot(plot_time, angleList_l2)
    window.angleGraph_l3.clear()
    window.angleGraph_l3.plot(plot_time, angleList_l3)
    window.angleGraph_l4.clear()
    window.angleGraph_l4.plot(plot_time, angleList_l4)
    window.angleGraph_l5.clear()
    window.angleGraph_l5.plot(plot_time, angleList_l5)
    window.angleGraph_l6.clear()
    window.angleGraph_l6.plot(plot_time, angleList_l6)

    window.angleGraph_rb.clear()
    window.angleGraph_rb.plot(plot_time, angleList_rb)
    window.angleGraph_r1.clear()
    window.angleGraph_r1.plot(plot_time, angleList_r1)
    window.angleGraph_r2.clear()
    window.angleGraph_r2.plot(plot_time, angleList_r2)
    window.angleGraph_r3.clear()
    window.angleGraph_r3.plot(plot_time, angleList_r3)
    window.angleGraph_r4.clear()
    window.angleGraph_r4.plot(plot_time, angleList_r4)
    window.angleGraph_r5.clear()
    window.angleGraph_r5.plot(plot_time, angleList_r5)
    window.angleGraph_r6.clear()
    window.angleGraph_r6.plot(plot_time, angleList_r6)

    #for x in plot_time:
    #    window.angleGraph_lb.remove(x)


def var_init():
    global leftMode, rightMode, leftAngleList, rightAngleList, states, plot_time
    global angleList_lb, angleList_l1, angleList_l2, angleList_l3, angleList_l4, angleList_l5, angleList_l6
    global angleList_rb, angleList_r1, angleList_r2, angleList_r3, angleList_r4, angleList_r5, angleList_r6
    states = 'Enabled'
    leftMode = True
    rightMode = False
    leftAngleList = [0, 0, 0, 0, 0, 0, 0]
    rightAngleList = [0, 0, 0, 0, 0, 0, 0]
    plot_time = [0]

    angleList_lb = [0]
    angleList_l1 = [0]
    angleList_l2 = [0]
    angleList_l3 = [0]
    angleList_l4 = [0]
    angleList_l5 = [0]
    angleList_l6 = [0]

    angleList_rb = [0]
    angleList_r1 = [0]
    angleList_r2 = [0]
    angleList_r3 = [0]
    angleList_r4 = [0]
    angleList_r5 = [0]
    angleList_r6 = [0]

def window_init(window):
    window.setupUi(widget)
    window.label_72.setText(states)
    window.pushButton_2.clicked.connect(home_robot)
    window.setAngleButton.clicked.connect(send_joint_angles)
    window.refreshButton.clicked.connect(updateControlInfo)
    window.recordButton.clicked.connect(record)
    window.stopButton.clicked.connect(stop)
    window.playButton.clicked.connect(play)
    window.estimateButton.clicked.connect(estimate)
    window.setRPButton.clicked.connect(set_RP_mode)
    window.connectRPButton.clicked.connect(connect_RP)
    window.leftGripperSlider.valueChanged.connect(sendGripperCmd)
    window.rightGripperSlider.valueChanged.connect(sendGripperCmd)
    window.label_72.setStyleSheet("QLabel {color:green;}")


if __name__ == "__main__":
    var_init()
    ros = ROS()
    app = QtWidgets.QApplication(sys.argv)
    widget = QtWidgets.QWidget()
    window = Ui_Dialog()
    window_init(window)
    widget.setWindowTitle('Robopuppet GUI Pro')
    updateControlInfo()

    # qtmodern.styles.dark(app)
    # mw = qtmodern.windows.ModernWindow(widget)
    # mw.show()
    widget.show()
    
    
    timer = QTimer()
    record_timer = QTimer()
    plot_timer = QTimer()  # Jason
    play_timer = QTimer()
    timer.timeout.connect(updateInfo)
    record_timer.timeout.connect(recordAngle)
    play_timer.timeout.connect(playing)
    plot_timer.timeout.connect(plot)  # Jason
    plot_timer.start(ui_update_rate)  # Jason update once per sec
    timer.start(ui_update_rate)

    sys.exit(app.exec_())
