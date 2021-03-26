
#include "SerialInterface.h"
#include <Encoders.h>
#include <Servos.h>
#include <Buttons.h>
#include <Arduino.h>
#include <hallEncoders.h>

#define SEND_EVERY_MS 1000


//-----Command Dictionary
/*
0 -> Arm Startup message
1 -> Arm should send response with joint positions and response command code 10
2 -> Arm should calibrate and send response command code 20 to indicate completion
*/

namespace SerialInterface
{
  int toSend[9];
}
bool SerialInterface::init() {
  Serial.begin(9600);
  Serial.println("0");
  return true;
}
void SerialInterface::update() {
  // String msg = Serial.readString();
  // if(msg != "101"){
  //   return;
  // }

  //Set each of these equal to the correct values coming from the arm
  toSend[0] = Encoders::getStatus(0); // Servo One
  toSend[1] = hallEncoders::getStatus(0); //Servo Two
  toSend[2] = Encoders::getStatus(1); //Servo Three
  toSend[3] = hallEncoders::getStatus(1);; //Encoder One
  toSend[4] = Encoders::getStatus(2); //Encoder Two
  toSend[5] = hallEncoders::getStatus(2);; //Encoder Three
  toSend[6] = Encoders::getStatus(3); //Encoder Four
  toSend[7] = Buttons::getGripperStatus(); //Gripper Engaged
  toSend[8] = Buttons::getHoldStatus(); //Arm Locked
  //Second arm
  toSend[9] = Encoders::getStatus(4); // Servo One 
  toSend[10] = hallEncoders::getStatus(3); //Servo Two
  toSend[11] = Encoders::getStatus(5); //Servo Three
  toSend[12] = hallEncoders::getStatus(4);; //Encoder One
  toSend[13] = Encoders::getStatus(6); //Encoder Two
  toSend[14] = hallEncoders::getStatus(5);; //Encoder Three
  toSend[15] = Encoders::getStatus(7); //Encoder Four
  toSend[16] = Buttons::getGripperStatus(); //Gripper Engaged
  toSend[17] = Buttons::getHoldStatus(); //Arm Locked

    String servoCompressed = String(toSend[0]) + ',' + String(toSend[1]) + ',' + String(toSend[2]); //String of Servo Data
    String encoderCompressed = String(toSend[3]) + ',' + String(toSend[4]) + ',' + String(toSend[5]) + ',' + String(toSend[6]); //String of Encoder Data
    String otherData = String(toSend[7]) + ',' + String(toSend[8]); // String of other data
    
    String servoCompressed_b = String(toSend[9]) + ',' + String(toSend[10]) + ',' + String(toSend[11]); //String of Servo Data
    String encoderCompressed_b = String(toSend[12]) + ',' + String(toSend[13]) + ',' + String(toSend[14]) + ',' + String(toSend[15]); //String of Encoder Data
    String otherData_b = String(toSend[16]) + ',' + String(toSend[17]); // String of other data
    
    // Serial.println("Encoder 1: Absolute: " + String(toSend[0]));
    // Serial.println("Encoder 2: Hall: " + String(toSend[1]));
    // Serial.println("Encoder 3: Absolute: " + String(toSend[2]));
    // Serial.println("Encoder 4: Hall: " + String(toSend[3]));
    // Serial.println("Encoder 5: Absolute: " + String(toSend[4]));
    // Serial.println("Encoder 6: Hall: " + String(toSend[5]));
    // Serial.println("Encoder 7: Absolute: " + String(toSend[6]));
    
    // Serial.println("Encoder 1b: Absolute: " + String(toSend[9]));
    // Serial.println("Encoder 2b: Hall: " + String(toSend[10]));
    // Serial.println("Encoder 3b: Absolute: " + String(toSend[11]));
    // Serial.println("Encoder 4b: Hall: " + String(toSend[12]));
    // Serial.println("Encoder 5b: Absolute: " + String(toSend[13]));
    // Serial.println("Encoder 6b: Hall: " + String(toSend[14]));
    // Serial.println("Encoder 7b: Absolute: " + String(toSend[15]));

    Serial.println("10:" + servoCompressed + ',' + encoderCompressed + ',' + otherData + ',' + servoCompressed_b + ',' + encoderCompressed_b + ',' + otherData_b); //Concats all strings together and sends over serial. 
    //Make sure companion Python Script is running to parse and send to ROS

    delay(SEND_EVERY_MS);
  
}

void SerialInterface::commandHandler() {
  if(Serial.available()){
    String msg = Serial.readString();
    int commandCode = atoi(strtok((char *)msg.c_str(),":"));

    switch (commandCode)
    {
    case 1: //GUI wants joint positions
      SerialInterface::update();
      break;

    case 2: //GUI wants recalibration
      Servos::init();
      break;

    default:
      //invalid command
      break;
    }
  }

}