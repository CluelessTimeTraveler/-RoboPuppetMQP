/**
 * @file UserBtns.h
 */

#pragma once
#include <stdint.h>

namespace hallEncoders
{
  void init();
  bool getStatus();
  bool getPosition();
  void update();
}
