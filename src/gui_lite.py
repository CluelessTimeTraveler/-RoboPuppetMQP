#!/usr/bin/env python

import os
import sys
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
import rospy
import math
from sensor_msgs.msg import Joy
from control_msgs.msg import GripperCommandActionGoal
from kortex_driver.msg import *
from kortex_driver.srv import *
from ui.gui_lite_ui import *
from std_msgs.msg import Float32, Float64
from gazebo_msgs.srv import GetJointProperties, GetLinkState
from kinematics import angleToCP
from constants import *
from PyQt5 import QtCore, QtWidgets
from cv_bridge import CvBridge, CvBridgeError
from msg_arduino.msg import JointPositions
from sensor_msgs.msg import Image
from ui.cam_view_ui import Ui_Camera_View
global leftAngleList, rightAngleList
global leftVelocityList, rightVelocityList
global leftCpList, rightCpList
global updateControlPanel
global leftMode, rightMode
global states
global mirror, reverse
global angleList_lb  # Jason
global left_cv_image,right_cv_image,main_cv_image
updateControlPanel = True
leftRecordList = []
rightRecordList = []


class ROS(QThread):
    def __init__(self):
        rospy.init_node('gui', anonymous=True)
        rospy.Subscriber('/potAngles', JointPositions, self.robopuppet, queue_size=1)
        rospy.Subscriber(robot_prefix+'/right_arm_cam/color/image_raw', Image, self.right_image, queue_size=1)
        rospy.Subscriber(robot_prefix+'/left_arm_cam/color/image_raw', Image, self.left_image, queue_size=1)
        rospy.Subscriber(robot_prefix+'/main_cam/color/image_raw', Image, self.main_image, queue_size=1)
        # get joint angle from Gazebo Service
        joints_properties = rospy.ServiceProxy('gazebo/get_joint_properties', GetJointProperties)
        self.bridge = CvBridge()

        # for moveit
        # self.pub_angle = rospy.Publisher('/desired_angle', joint_angle, queue_size=1)

        self.rightJoint1 = rospy.Publisher('/'+robot_prefix+'/right_arm_joint_1_position_controller/command', Float64, queue_size=1)
        self.rightJoint2 = rospy.Publisher('/'+robot_prefix+'/right_arm_joint_2_position_controller/command', Float64, queue_size=1)
        self.rightJoint3 = rospy.Publisher('/'+robot_prefix+'/right_arm_joint_3_position_controller/command', Float64, queue_size=1)
        self.rightJoint4 = rospy.Publisher('/'+robot_prefix+'/right_arm_joint_4_position_controller/command', Float64, queue_size=1)
        self.rightJoint5 = rospy.Publisher('/'+robot_prefix+'/right_arm_joint_5_position_controller/command', Float64, queue_size=1)
        self.rightJoint6 = rospy.Publisher('/'+robot_prefix+'/right_arm_joint_6_position_controller/command', Float64, queue_size=1)
        self.rightJoint7 = rospy.Publisher('/'+robot_prefix+'/right_arm_joint_7_position_controller/command', Float64, queue_size=1)
        self.rightGripper = rospy.Publisher('/'+robot_prefix+'/right_arm_robotiq_2f_85_gripper_controller/gripper_cmd/goal',
                                            GripperCommandActionGoal, queue_size=1)

        self.leftJoint1 = rospy.Publisher('/'+robot_prefix+'/left_arm_joint_1_position_controller/command', Float64, queue_size=1)
        self.leftJoint2 = rospy.Publisher('/'+robot_prefix+'/left_arm_joint_2_position_controller/command', Float64, queue_size=1)
        self.leftJoint3 = rospy.Publisher('/'+robot_prefix+'/left_arm_joint_3_position_controller/command', Float64, queue_size=1)
        self.leftJoint4 = rospy.Publisher('/'+robot_prefix+'/left_arm_joint_4_position_controller/command', Float64, queue_size=1)
        self.leftJoint5 = rospy.Publisher('/'+robot_prefix+'/left_arm_joint_5_position_controller/command', Float64, queue_size=1)
        self.leftJoint6 = rospy.Publisher('/'+robot_prefix+'/left_arm_joint_6_position_controller/command', Float64, queue_size=1)
        self.leftJoint7 = rospy.Publisher('/'+robot_prefix+'/left_arm_joint_7_position_controller/command', Float64, queue_size=1)
        self.leftGripper = rospy.Publisher('/'+robot_prefix+'/left_arm_robotiq_2f_85_gripper_controller/gripper_cmd/goal',
                                           GripperCommandActionGoal,
                                           queue_size=1)


    def left_image(self,data):
        global left_cv_image
        left_cv_image = self.bridge.imgmsg_to_cv2(data,'rgb8')

    def right_image(self,data):
        global right_cv_image
        right_cv_image = self.bridge.imgmsg_to_cv2(data,'rgb8')

    def main_image(self,data):
        global main_cv_image
        main_cv_image = self.bridge.imgmsg_to_cv2(data,'rgb8')

    def cb_action_topic(self, notif):
        self.last_action_notif_type = notif.action_event

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

    def robopuppet(self, data):
        c = math.pi/180
        if states == 'Enabled':
            #!!!!!!!!!!!!!!!!11
            gripper = data.gripperToggle

            angles = [data.servoOne*c, data.servoTwo*c, data.servoThree*c, data.encoderOne*c, data.encoderTwo*c,
                      data.encoderThree*c, data.encoderFour*c]  # angles from robopuppet
            if mirror:
                angles_right = []
                for i in angles:
                    angles_right.append(-i)
            else:
                angles_right = angles

            if leftMode & rightMode:
                self.send_gripper_cmd(gripper, gripper)
                if reverse:
                    self.publish_to_left(angles_right)
                    self.publish_to_right(angles)
                else:
                    self.publish_to_left(angles)
                    self.publish_to_right(angles_right)

            elif leftMode:
                self.send_gripper_cmd(gripper, 0)
                self.publish_to_left(angles)
            else:
                self.send_gripper_cmd(0, gripper)
                self.publish_to_right(angles)

    def leftUpdate(self):
        global leftAngleList
        global leftVelocityList
        global leftCpList

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
        rospy.loginfo("Send Gripper Command")
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



def updateInfo():
    if window.main_ui.radioButton_3.isChecked():
        window.main_ui.checkBox.setDisabled(False)
        window.main_ui.checkBox_2.setDisabled(False)
    else:
        window.main_ui.checkBox.setDisabled(True)
        window.main_ui.checkBox_2.setDisabled(True)

    ros.leftUpdate()
    ros.rightUpdate()

    if leftMode and not rightMode:
        angleList = leftAngleList
        veloList = leftVelocityList
        cplist = leftCpList
    elif rightMode and not leftMode:
        angleList = rightAngleList
        veloList = rightVelocityList
        cplist = rightCpList
    elif rightMode and leftMode and reverse:
        angleList = rightAngleList
        veloList = rightVelocityList
        cplist = rightCpList
    else:
        angleList = leftAngleList
        veloList = leftVelocityList
        cplist = leftCpList


    window.main_ui.label_15.setText("%.2f" % angleList[0])
    window.main_ui.label_16.setText("%.2f" % angleList[1])
    window.main_ui.label_17.setText("%.2f" % angleList[2])
    window.main_ui.label_18.setText("%.2f" % angleList[3])
    window.main_ui.label_19.setText("%.2f" % angleList[4])
    window.main_ui.label_20.setText("%.2f" % angleList[5])
    window.main_ui.label_21.setText("%.2f" % angleList[6])
    # Velocity
    window.main_ui.label_8.setText("%.2f" % veloList[0])
    window.main_ui.label_9.setText("%.2f" % veloList[1])
    window.main_ui.label_10.setText("%.2f" % veloList[2])
    window.main_ui.label_11.setText("%.2f" % veloList[3])
    window.main_ui.label_12.setText("%.2f" % veloList[4])
    window.main_ui.label_13.setText("%.2f" % veloList[5])
    window.main_ui.label_14.setText("%.2f" % veloList[6])
    # Cartesian pose
    window.main_ui.label_35.setText("x_pose: %.2f" % cplist[0])
    window.main_ui.label_36.setText("y_pose: %.2f" % cplist[1])
    window.main_ui.label_37.setText("z_pose: %.2f" % cplist[2])


    height, width, channel = main_cv_image.shape
    bytesPerLine = 3 * width
    main_qImg = QImage(main_cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)
    main_qImg = main_qImg.scaledToWidth(cam_width)
    child.child_ui.main_cam_view.setPixmap(QPixmap(main_qImg))

    height, width, channel = left_cv_image.shape
    bytesPerLine = 3 * width
    left_qImg = QImage(left_cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)
    left_qImg = left_qImg.scaledToWidth(cam_width)
    child.child_ui.left_cam_view.setPixmap(QPixmap(left_qImg))

    height, width, channel = right_cv_image.shape
    bytesPerLine = 3 * width
    right_qImg = QImage(right_cv_image.data, width, height, bytesPerLine, QImage.Format_RGB888)
    right_qImg = right_qImg.scaledToWidth(cam_width)
    child.child_ui.right_cam_view.setPixmap(QPixmap(right_qImg))



def set_RP_mode():
    global leftMode,rightMode,mirror,reverse
    if window.main_ui.radioButton.isChecked():
        leftMode = True
        rightMode = False
    elif window.main_ui.radioButton_2.isChecked():
        leftMode = False
        rightMode = True
    else:
        leftMode = True
        rightMode = True
	#TODO FIXXXXX
        if window.main_ui.checkBox.isChecked():
            mirror = True
        if window.main_ui.checkBox_2.isChecked():
            reverse = True



def connect_RP():
    global states
    color = 'black'
    if states == 'Enabled':
        states = 'Disabled'
        color = 'red'
    elif states == 'Disabled':
        states = 'Enabled'
        color = 'green'
    window.main_ui.label_72.setText(states)
    window.main_ui.label_72.setStyleSheet("QLabel {color:" + color + ";}")



def var_init():
    global leftMode, rightMode, leftAngleList, rightAngleList, states, angleList_lb, mirror, reverse
    states = 'Enabled'
    leftMode = True
    rightMode = False
    mirror = False
    reverse = False
    leftAngleList = [0, 0, 0, 0, 0, 0, 0]
    rightAngleList = [0, 0, 0, 0, 0, 0, 0]
    angleList_lb = [0]


class parentWindow(QDialog):
    def __init__(self):
        QDialog.__init__(self)
        self.main_ui = Ui_Dialog()
        self.main_ui.setupUi(self)

class childWindow(QDialog):
    def __init__(self):
        QDialog.__init__(self)
        self.child_ui=Ui_Camera_View()
        self.child_ui.setupUi(self)

def show_new_window():
    child.show()

def home_robot():
    ros.home_the_robot()

def window_init(window):
    window.label_72.setText(states)
    window.setRPButton.clicked.connect(set_RP_mode)
    window.connectRPButton.clicked.connect(connect_RP)
    window.open_cam.clicked.connect(show_new_window)
    window.pushButton.clicked.connect(home_robot)
    window.label_72.setStyleSheet("QLabel {color:green;}")


if __name__ == "__main__":
    var_init()
    ros = ROS()
    ros.home_the_robot()
    app = QtWidgets.QApplication(sys.argv)
    window = parentWindow()
    window.setWindowTitle('Robopuppet GUI Lite')
    child = childWindow()
    child.setWindowTitle('Robopuppet Camera View')
    window.show()
    window_init(window.main_ui)
    # widget.show()
    timer = QTimer()
    timer.timeout.connect(updateInfo)
    timer.start(ui_update_rate)
    sys.exit(app.exec_())
