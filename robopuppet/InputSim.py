#!/usr/bin/env python

import rospy
import serial
import sys
from std_msgs.msg import String
from msg_arduino.msg import LeftArmPositions
from msg_arduino.msg import RightArmPositions

def PuppetSerialCommSim():
    leftPub = rospy.Publisher('RightArm', LeftArmPositions, queue_size=1)
    rightPub = rospy.Publisher('LeftArm', RightArmPositions, queue_size=1)

    rospy.init_node('PuppetSerialCommSim', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():

        servoData1 = 0
        servoData2 = 0
        servoData3 = 0
        encoderData1 = 0
        encoderData2 = 0
        encoderData3 = 0
        encoderData4 = 0
        gripperEngaged = 0
        armLocked = 0
    
        msg = LeftArmPositions(servoData1, servoData2, servoData3, encoderData1, encoderData2, encoderData3, encoderData4, gripperEngaged, armLocked)
        rospy.loginfo(msg)
        #leftPub.publish(msg)

        servoData1 = 1
        servoData2 = 1
        servoData3 = 1
        encoderData1 = 45
        encoderData2 = 1
        encoderData3 = 1
        encoderData4 = 1
        gripperEngaged = 0
        armLocked = 1

        msg = RightArmPositions(servoData1, servoData2, servoData3, encoderData1, encoderData2, encoderData3, encoderData4, gripperEngaged, armLocked)
        rightPub.publish(msg)
        rospy.loginfo(msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        PuppetSerialCommSim()
    except rospy.ROSInterruptException:
        pass
