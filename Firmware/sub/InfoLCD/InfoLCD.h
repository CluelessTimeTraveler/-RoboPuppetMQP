/**
 * @file InfoLCD.h
 */
#pragma once
#include <stdint.h>
#include <Arduino.h>

namespace InfoLCD
{
    bool init();
    void printToLCD(String toPrint);
}