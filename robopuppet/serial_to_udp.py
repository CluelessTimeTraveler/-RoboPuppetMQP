import serial
import sys
import socket

UDP_IP = "127.0.0.1"
UDP_PORT = 5005
MESSAGE = b""

def PuppetSerialComms():

    #port = sys.argv[1]
    port = '/dev/ttyACM0'
    try:
        print("Attemping connection with " + str(port))
        serialInterface = serial.Serial('/dev/ttyACM0', baudrate=9600)
    except serial.SerialException:
        print("----------ERROR------------Unable to open Serial Port -----------ERROR---------")
        return

    while serialInterface.in_waiting:
        data = serialInterface.readline().decode("utf-8",errors='ignore')
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

        MESSAGE = data
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
