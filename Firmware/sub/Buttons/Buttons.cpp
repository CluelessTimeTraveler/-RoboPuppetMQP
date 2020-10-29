/**
 * @file Buttons.cpp
 */
#include "Buttons.h"
#include <Arduino.h>
/**
 * Private subsystem info
 */
namespace Buttons
{
  const int gripperToggle = 14;
  const int holdToggle = 13;
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
 
 if(digitalRead(holdToggle))
    gripperState =! gripperState;

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

