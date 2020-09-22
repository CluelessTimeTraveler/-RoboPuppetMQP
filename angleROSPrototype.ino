#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int16 str_msg;
ros::Publisher potAngles("potAngles", &str_msg);


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
  str_msg.data = sensorValue;
  potAngles.publish(&str_msg);
  //Serial.println(String(sensorValue));
  delay(500);
  nh.spinOnce();
  
}
