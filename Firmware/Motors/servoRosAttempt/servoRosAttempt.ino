#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int16 str_msg;
ros::Publisher potAngles("potAngles", &str_msg);


#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
int sensorValue;
int pos = 0;    // variable to store the servo position
int sensorPin = A0;
void setup() {
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
  myservo.attach(2);  // attaches the servo on pin 9 to the servo object
    nh.initNode();
   nh.advertise(potAngles);
   
   
}

void loop() {
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    sensorValue = myservo.read();
    str_msg.data = sensorValue;
    potAngles.publish(&str_msg);
    //sensorValue = analogRead(sensorPin);
   // Serial.println(sensorValue);
    delay(250);                       // waits 15ms for the servo to reach the position
 
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    sensorValue = myservo.read();
    //sensorValue = analogRead(sensorPin);
    str_msg.data = sensorValue;
    potAngles.publish(&str_msg);
    //Serial.println(sensorValue);
    delay(250);                       // waits 15ms for the servo to reach the position
  }
}
  



//int sensorPin = A0;    // select the input pin for the potentiometer
//int sensorValue;  // variable to store the value coming from the sensor
//
//
//
//void setup() {
//
//  //Serial.begin(9600);
//  pinMode(sensorPin, INPUT);
//  nh.initNode();
//  nh.advertise(potAngles);
//}
//
//void loop() {
//  // read the value from the sensor:
//  sensorValue = analogRead(sensorPin);
//  str_msg.data = sensorValue;
//  potAngles.publish(&str_msg);
//  //Serial.println(String(sensorValue));
//  delay(500);
//  nh.spinOnce();
//  
//}



