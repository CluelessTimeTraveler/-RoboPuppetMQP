#!/usr/bin/env python
###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2019 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import rospy
import serial
import sys
from std_msgs.msg import String
from msg_arduino.msg import JointPositions

def PuppetSerialComms():
    pub = rospy.Publisher('potAngles', JointPositions, queue_size=1)
#    if(len(sys.argv) != 4):
#        print("Argv = " + str(len(sys.argv)) + str(sys.argv))
#        print("NO SERIAL PORT GIVEN OR INVALID PORT")
#        return;

    #port = sys.argv[1]
    port = '/dev/ttyACM0'
    try:
        print("Attemping connection with " + str(port))
        serialInterface = serial.Serial('/dev/ttyACM0', baudrate=9600)
    except serial.SerialException:
        print("----------ERROR------------Unable to open Serial Port -----------ERROR---------")
        return;

    rospy.init_node('PuppetSerialComms', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        while serialInterface.in_waiting:
            data = serialInterface.readline().decode("utf-8",errors='ignore')
            print("Data recieved: " + data);
            try:
                dataStringValues = data.split(",")
            except ValueError:
                print("Error parsing CSV data string, trying again")
                continue

            if len(dataStringValues)!=9:
                print("Full data string not recieved, trying again")
                continue
            try:
                servoData1 = int(dataStringValues[0])
                servoData2 = int(dataStringValues[1])
                servoData3 = int(dataStringValues[2])
                encoderData1 = int(dataStringValues[3])
                encoderData2 = int(dataStringValues[4])
                encoderData3 = int(dataStringValues[5])
                encoderData4 = int(dataStringValues[6])
                gripperEngaged = int(dataStringValues[7])
                armLocked = int(dataStringValues[8])
            except ValueError:
                print("Unable to convert one of the data values to integer, trying again")
                continue

            msg = JointPositions(servoData1, servoData2, servoData3, encoderData1, encoderData2, encoderData3, encoderData4, gripperEngaged, armLocked)
            #rospy.loginfo(msg)
            pub.publish(msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        PuppetSerialComms()
    except rospy.ROSInterruptException:
        pass
