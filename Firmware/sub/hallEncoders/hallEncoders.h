/**
 * @file hallEncoders.h
 */

#pragma once
#include <stdint.h>

namespace hallEncoders
{
  void init();
  uint16_t getPositionSPI(uint8_t, uint8_t);
  uint16_t updateSingle(uint8_t);
  void update();
  int getStatus(uint8_t);
  uint8_t spiWriteRead(uint8_t, uint8_t, uint8_t);
}
