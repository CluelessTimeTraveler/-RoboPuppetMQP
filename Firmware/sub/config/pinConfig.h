/**
 * @file config.h
 */

#pragma once
#include <stdint.h>

namespace pinConfig
{
    //Abs Encoders
    const uint8_t encoder1 = 2;
    const uint8_t encoder2 = 4;
    const uint8_t encoder3 = 6;
    const uint8_t encoder4 = 21;

    //Hall Encoders
    const uint8_t hallEncode1 = 3;
    const uint8_t hallEncode2 = 5;
    const uint8_t hallEncode3 = 7;

    //SPI
    const uint8_t SPI_MOSI = 51;
    const uint8_t SPI_MISO = 50;
    const uint8_t SPI_SCLK = 52;

    //Buttons
    const int gripperToggle = 34;
    const int holdToggle = 35;


}