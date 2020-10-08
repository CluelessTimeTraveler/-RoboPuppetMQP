/**
 * @file UserBtns.h
 */

#pragma once
#include <stdint.h>

namespace Buttons
{
  void init();
  bool getStatus();
  bool getPosition();
  void update();
}
