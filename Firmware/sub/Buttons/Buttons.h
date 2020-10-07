/**
 * @file UserBtns.h
 */

#pragma once
#include <stdint.h>

namespace Buttons
{
  void init();
  bool getHoldStatus();
  bool getGripperStatus();
  void update();
}
