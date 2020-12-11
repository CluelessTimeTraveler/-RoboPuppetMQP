
#include "SerialInterface.h"
#include <Encoders.h>
#include <Buttons.h>
#include <Arduino.h>
#include <hallEncoders.h>

#define SEND_EVERY_MS 1000


namespace SerialInterface
{
  int toSend[9];
}
bool SerialInterface::init() {
  Serial.begin(9600);
  return true;
}
void SerialInterface::update() {

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

    String servoCompressed = String(toSend[0]) + ',' + String(toSend[1]) + ',' + String(toSend[2]); //String of Servo Data
    String encoderCompressed = String(toSend[3]) + ',' + String(toSend[4]) + ',' + String(toSend[5]) + ',' + String(toSend[6]); //String of Encoder Data
    String otherData = String(toSend[7]) + ',' + String(toSend[8]); // String of other data
    
    /*
    Serial.println("Encoder 1: Absolute: " + String(toSend[0]));
    Serial.println("Encoder 2: Hall: " + String(toSend[1]));
    Serial.println("Encoder 3: Absolute: " + String(toSend[2]));
    Serial.println("Encoder 4: Hall: " + String(toSend[3]));
    Serial.println("Encoder 5: Absolute: " + String(toSend[4]));
    Serial.println("Encoder 6: Hall: " + String(toSend[5]));
    Serial.println("Encoder 7: Absolute: " + String(toSend[6]));
    */

    Serial.println(servoCompressed + ',' + encoderCompressed + ',' + otherData); //Concats all strings together and sends over serial. 
    //Make sure companion Python Script is running to parse and send to ROS

    delay(SEND_EVERY_MS);
  
}
