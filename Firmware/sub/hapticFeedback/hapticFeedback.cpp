/**
 * @file hapticFeedback.cpp
 */
#include "Buttons.h"
#include <Arduino.h>
#include "pinConfig.h"
#include "InfoLCD.h"
/**
 * Private subsystem info
 */
namespace hapticFeedback
{
  const int leftMotors = pinConfig::leftMotors;
  const int rightMotors = pinConfig::rightMotors;
}

void hapticFeedback::init(){
  pinMode(leftMotors, OUTPUT);
  pinMode(rightMotors, OUTPUT); 
}

void hapticFeedback::update(){
  /*
    //Choose condition
    if(condition)
      digitalWrite(leftMotors, HIGH);
      digitalWrite(rightMotors, HIGH);
    else
      digitalWrite(leftMotors, LOW);
      digitalWrite(rightMotors, LOW);
  */
}
