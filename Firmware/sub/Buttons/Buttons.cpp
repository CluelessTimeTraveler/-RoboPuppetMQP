/**
 * @file Buttons.cpp
 */
#include "Buttons.h"
#include <Arduino.h>
#include "pinConfig.h"
#include "InfoLCD.h"
/**
 * Private subsystem info
 */
namespace Buttons
{
  const int gripperToggle = pinConfig::gripperToggle;
  const int holdToggle = pinConfig::holdToggle;
  bool gripperState;
  bool holdState;
  unsigned long timeOfGripperToggle = 0;
}

void Buttons::init(){
  pinMode(gripperToggle, INPUT);
  pinMode(holdToggle, INPUT);
  holdState = true;
  gripperState = false;
}

void Buttons::update(){
  holdState = digitalRead(holdToggle);
  int toggleReading = digitalRead(gripperToggle);
  // Gripper button debounce
  unsigned long currentTime = millis();
  if(toggleReading && (currentTime - timeOfGripperToggle) > 2000){
    gripperState = !gripperState;
    if(gripperState){
      InfoLCD::printToLCD("Gripper Engaged");
    }else{
      InfoLCD::printToLCD("Gripper Disengaged");
    }
    timeOfGripperToggle = currentTime;
  }
}

bool Buttons::getHoldStatus()
{
  return holdState;
}

bool Buttons::getGripperStatus()
{
  return gripperState;
}

