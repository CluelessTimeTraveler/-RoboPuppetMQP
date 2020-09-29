/**
 * @file Encoders.h
 */
#pragma once
//#include <RoboPuppet.h>
//#include <stdint.h>

namespace Encoders
{
  void init();
  uint16_t getPositionSPI(uint8_t, uint8_t);
}
