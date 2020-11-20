
#include "SerialInterface.h"
#include <Encoders.h>
#include <Buttons.h>
#include <Arduino.h>
#include <hallEncoders.h>

#define SEND_EVERY_MS 1000


namespace SerialInterface
{
  int sensorPin = 32;    // select the input pin for the potentiometer
  int sensorValue;  // variable to store the value coming from the sensor
  uint16_t toSend[9];
  uint16_t prevSend[9];
  bool messageChange;
}
bool SerialInterface::init() {

  pinMode(sensorPin, INPUT);
  messageChange = true;
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
  toSend[6] = 0; //Encoder Four
  toSend[7] = 1; //Encoder Three
  toSend[8] = 2; //Encoder Four
  //toSend[7] = (int)Buttons::getGripperStatus; //Gripper Engaged
  //toSend[8] = (int)Buttons::getHoldStatus; //Arm Locked
/*
  for(int i=0; i<8; i++){
    if (abs(toSend[i] - prevSend[i]) > 5)
        messageChange = true;
  }

 // if(messageChange){
//  for(int i=0; i<8; i++){
 //       prevSend[i] = toSend[i];
 //   }
    messageChange = false;
*/
    String servoCompressed = String(toSend[0]) + ',' + String(toSend[1]) + ',' + String(toSend[2]); //String of Servo Data
    String encoderCompressed = String(toSend[3]) + ',' + String(toSend[4]) + ',' + String(toSend[5]) + ',' + String(toSend[6]); //String of Encoder Data
    String otherData = String(toSend[7]) + ',' + String(toSend[8]); // String of other data
    
    Serial.println(servoCompressed + ',' + encoderCompressed + ',' + otherData); //Concats all strings together and sends over serial. 
    //Make sure companion Python Script is running to parse and send to ROS

    delay(SEND_EVERY_MS);
  
}
