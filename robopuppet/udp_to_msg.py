
import rospy
import sys
import socket
from std_msgs.msg import String
from msg_arduino.msg import LeftArmPositions
from msg_arduino.msg import RightArmPositions

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

def PuppetSerialComms():
 
    leftPub = rospy.Publisher('LeftArm', LeftArmPositions, queue_size=1)
    rightPub = rospy.Publisher('RightArm', RightArmPositions, queue_size=1)

    sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

    rospy.init_node('PuppetSerialComms', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        while True:
            data, addr = sock.recvfrom(1024)
            print("Data recieved: " + data)
            try:
                dataStringValues = data.split(",")
            except ValueError:
                print("Error parsing CSV data string, trying again")
                continue

            if len(dataStringValues)!=18:
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

                servoData1_b = int(dataStringValues[9])
                servoData2_b = int(dataStringValues[10])
                servoData3_b = int(dataStringValues[11])
                encoderData1_b = int(dataStringValues[12])
                encoderData2_b = int(dataStringValues[13])
                encoderData3_b = int(dataStringValues[14])
                encoderData4_b = int(dataStringValues[15])
                gripperEngaged_b = int(dataStringValues[16])
                armLocked_b = int(dataStringValues[17])
            except ValueError:
                print("Unable to convert one of the data values to integer, trying again")
                continue

            msg = LeftArmPositions(servoData1, servoData2, servoData3, encoderData1, encoderData2, encoderData3, encoderData4, gripperEngaged, armLocked)
	        msg_b = RightArmPositions(servoData1_b, servoData2_b, servoData3_b, encoderData1_b, encoderData2_b, encoderData3_b, encoderData4_b, gripperEngaged_b, armLocked_b)
            #rospy.loginfo(msg)
            leftPub.publish(msg)
	        rightPub.publish(msg_b)

            rate.sleep()

if __name__ == '__main__':
    try:
        PuppetSerialComms()
    except rospy.ROSInterruptException:
        pass
