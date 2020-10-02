/**
 * @file Buttons.cpp
 */
#include "buttons.h"

/**
 * Private subsystem info
 */
namespace Buttons
{
  const int gripperToggle = 14;
  const int holdToggle = 13;
  bool closeGripper;
  bool holdState;
}

void Buttons::init(){
  pinMode(gripperToggle, INPUT);
  pinMode(holdToggle, INPUT);
  holdState = false;
  closeGripper = false;
}

bool Buttons::holdStatus()
{
  if(digitalRead(holdToggle))
     holdState =! holdState;
}

bool Buttons::toggleStatus()
{
   if(digitalRead(holdToggle))
      closeGripper =! closeGripper;
}

