#include <Arduino.h>
#include "Servos.h"
#include <Servo.h>

/**
 * Servos namespace
 */
namespace Servos
{
  //Variables
  const uint8_t sensorPin = A0;
  const uint8_t pos = 0; // variable to store the servo position
  const uint8_t sensorValue;

  //Servos
  const uint8_t num_servos = 4;
  const uint8_t servoPints[num_servos] = [2, 3, 4, 5];
  float angles[num_servos;
  Servo myservo[num_servos];  // create servo objects to control the servos

  // Init flag
  bool init_complete = false;

}

/**
 * Initializes subsystem
 */
void Servos::init()
{
  if (!init_complete)
  {
    //Set up sensor pin
    pinMode(sensorPin, INPUT);
  
    // attaches the servo pins to the servo objects
      for (uint8_t j = 0; j < Servos::num_servos; j++)
      {
        myservo[j].attach(servoPin(j))
      }
  
    //Initalize the serial connection - check baud rate for microcontroller
    Serial.begin(115200);
  
    //Nice screen things
    Serial.println("Servos Initialized");

    //initalize joint angles
    for (uint8_t j = 0; j < num_servos; j++)
		{
			angles[j] = 0.0f;
		}

    // Set init flag
    init_complete = true;
  }
}

/**
 * @brief Reads and stores each servo angle
 */
void Servos::update()
{
	for (uint8_t j = 0; j < Servos::num_servos; j++)
	{
		Servos::angles[j] = Servos::getPosition(j);
	}
}

/**
 * @brief Transmits servo value to RosComms
 */
float Servos::getStatus(uint8_t servo)
{
  return Servos::angles[servo];
}

/**
 * @brief Gets Position value from the Servo
 */
uint8_t Servos::getPosition(uint8_t servo_num)
{
    sensorValue = myservo[servo_num].read();
    return sensorValue; 
}