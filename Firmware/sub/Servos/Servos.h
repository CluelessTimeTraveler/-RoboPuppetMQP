/**
 * @file UserBtns.h
 */

#pragma once
#include <stdint.h>

namespace Servos
{
  void init();
  bool getStatus();
  bool getPosition();
  void update();
}
