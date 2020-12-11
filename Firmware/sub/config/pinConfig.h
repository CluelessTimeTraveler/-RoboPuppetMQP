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
    const uint8_t encoder4 = 44;

    //Hall Encoders
    const uint8_t hallEncode1 = 3;
    const uint8_t hallEncode2 = 5;
    const uint8_t hallEncode3 = 7;

    //SPI Arduino
    const uint8_t SPI_COPI = 51; //yellow
    const uint8_t SPI_CIPO = 50; //white
    const uint8_t SPI_SCLK = 52; //green 

    //SPI Teensy
    // const uint8_t SPI_COPI = 11; //yellow (MOSI)
    // const uint8_t SPI_CIPO = 12; //white (MISO)
    // const uint8_t SPI_SCLK = 13; //green 
    
    // const uint8_t SPI_COPI_h = 11; //yellow
    // const uint8_t SPI_CIPO_h = 12; //white
    // const uint8_t SPI_SCLK_h = 13; //green 

    //Servos
    const uint8_t Servos[6] = {8, 9, 10, 11, 12, 46};

    //Buttons
    const int gripperToggle = 34;
    const int holdToggle = 35;


}