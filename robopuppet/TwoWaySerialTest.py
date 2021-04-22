import serial
import time
from struct import *

def trySerial():
    port = '/dev/ttyACM0'
    try:
        print("Attemping connection with " + str(port))
        serialInterface = serial.Serial('/dev/ttyACM0', baudrate=9600)
    except serial.SerialException:
        print("----------ERROR------------Unable to open Serial Port -----------ERROR---------")
        return
    print("Sucess! :)")
    print("Awaiting startup message...")
    armStarted = 0
    while armStarted == 0:
        if serialInterface.in_waiting != 0:
            startMsgRx = serialInterface.readline().decode("utf-8",errors='ignore')
            if startMsgRx == "0":
                armStarted = 1
    print("Arm Started Successfully")


    print("Sending Calibration Request")
    request = "2:calibrate"
    serialInterface.write(bytes(request, 'utf-8'))
    print("Awaiting Calibration success...")
    while armCalibrated == 0:
        if serialInterface.in_waiting != 0:
            calibrateMsgRx = serialInterface.readline().decode("utf-8",errors='ignore')
            if calibrateMsg == "0":
                armCalibrated = 1
    print("Arm Calibrated")


    while 1:
        print("Sending request for data...")
        request = "1:get data"
        serialInterface.write(bytes(request, 'utf-8'))
        print("Waiting for requested data...")
        while recPosData == 0:
            if serialInterface.in_waiting != 0:
                posDataString = serialInterface 
        data = serialInterface.readline().decode("utf-8",errors='ignore')
        print("Data recieved: " + data)
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
	msg = LeftArmPositions(servoData1, servoData2, servoData3, encoderData1, encoderData2, encoderData3, encoderData4, gripperEngaged, armLocked)
        #rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()


trySerial()
