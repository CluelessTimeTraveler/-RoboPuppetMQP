#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>
#include <DigitalOut.h>

ros::NodeHandle nh;

std_msgs::Int16 str_msg;
ros::Publisher potAngles("potAngles", &str_msg);

/**
 * Private subsystem info
 */
namespace Motors
{
  //temp
  const int sensorPin = 14;
  DigitalOut enable_output(pin_en);

  Servo myservo[6];  // create servo object to control a servo
  
  // Pin definitions
  const uint8_t pin_en = 2;
  const uint8_t servo_pins[6] = {2, 3, 4, 5, 6, 7};
  
  // Init flag
  bool init_complete = false;
}

/**
 * @brief Initializes motor drivers
 */
void Motors::init()
{
  if (!init_complete)
  {
    // Disable motor drivers
    enable_output = 0;

    // Initalize Servo objects
    for (uint8_t j = 0; j < num_joints; j++)
    {
        myservo[j].attach(servo_pins[j]);
    }
    
    // Set init flag
    init_complete = true;
  }
}

void Motors::testMotion(int i) {
  int pos = 0;
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo[i].write(pos);              // tell servo to go to position in variable 'pos'
    sensorValue = myservo[i].read();
    //str_msg.data = sensorValue;
    //potAngles.publish(&str_msg);
    //sensorValue = analogRead(sensorPin);
   // Serial.println(sensorValue);
    delay(250);                       // waits 15ms for the servo to reach the position
 
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo[i].write(pos);              // tell servo to go to position in variable 'pos'
    sensorValue = myservo{i].read();
    //sensorValue = analogRead(sensorPin);
    //str_msg.data = sensorValue;
    //potAngles.publish(&str_msg);
    //Serial.println(sensorValue);
    delay(250);                       // waits 15ms for the servo to reach the position
  }
}
  



