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

    const uint8_t encoder1b = 22;
    const uint8_t encoder2b = 24;
    const uint8_t encoder3b = 26;
    const uint8_t encoder4b = 28;

    //Hall Encoders
    const uint8_t hallEncode1 = 3;
    const uint8_t hallEncode2 = 5;
    const uint8_t hallEncode3 = 7;

    const uint8_t hallEncode1b = 23;
    const uint8_t hallEncode2b = 25;
    const uint8_t hallEncode3b = 27;

    //SPI Arduino
    const uint8_t SPI_COPI = 51; //yellow
    const uint8_t SPI_CIPO = 50; //white
    const uint8_t SPI_SCLK = 52; //green 

    //Servos
    const uint8_t Servos[6] = {8, 9, 10, 11, 12, 46};
    const uint8_t Servosb[6] = {29, 30, 31, 32, 33, 34};

    //Buttons
    const int gripperToggle = 36;
    const int gripperToggleb = 35;
    const int holdToggle = 37;

    //Vibration Motors
    const int leftMotors = 38;
    const int rightMotors = 39;


}