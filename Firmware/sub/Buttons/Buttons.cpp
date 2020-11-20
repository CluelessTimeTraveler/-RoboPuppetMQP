/**
 * @file Buttons.cpp
 */
#include "Buttons.h"
#include <Arduino.h>
#include "pinConfig.h"
/**
 * Private subsystem info
 */
namespace Buttons
{
  const int gripperToggle = pinConfig::gripperToggle;
  const int holdToggle = pinConfig::holdToggle;
  bool gripperState;
  bool holdState;
}

void Buttons::init(){
  pinMode(gripperToggle, INPUT);
  pinMode(holdToggle, INPUT);
  holdState = true;
  gripperState = false;
}

void Buttons::update(){
 
 if(digitalRead(holdToggle)){
    Serial.print("hold toggle: ");
    gripperState =! gripperState;
    Serial.println(gripperState);
}

if(digitalRead(gripperToggle))
    gripperState =! gripperState;
}

bool Buttons::getHoldStatus()
{
  return holdState;
}

bool Buttons::getGripperStatus()
{
  return gripperState;
}

