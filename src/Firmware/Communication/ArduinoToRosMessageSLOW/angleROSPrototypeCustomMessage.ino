#include <ros.h>
#include <std_msgs/Int16.h>
#include <msg_arduino/JointPositions.h>

ros::NodeHandle nh;

//std_msgs::Int16 posMsg;

msg_arduino::JointPositions posMsg;

ros::Publisher potAngles("potAngles", &posMsg);
//ros::Publisher potAngles("potAngles", &posMsg);

int sensorPin = 32;    // select the input pin for the potentiometer
int sensorValue;  // variable to store the value coming from the sensor



void setup() {

  //Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  nh.initNode();
  nh.advertise(potAngles);
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  //posMsg.data = sensorValue;
  posMsg.servoOne = sensorValue;
  posMsg.servoTwo = 22;
  posMsg.servoThree = 23;
  posMsg.encoderOne = 10;
  posMsg.encoderTwo = 11;
  posMsg.encoderThree = 12;
  posMsg.encoderFour = 13;
  posMsg.gripperToggle = 1;
  posMsg.holdToggle = 0;


  
  potAngles.publish(&posMsg);
  //Serial.println(String(sensorValue));
  delay(500);
  nh.spinOnce();
  
}
