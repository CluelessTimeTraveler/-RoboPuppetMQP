/**
 * @file callibration.cpp
 */

#include "calibration.h"
#include <Encoders.h>
#include <hallEncoders.h>
#include "pinConfig.h"
#include <stdint.h>
#include <Arduino.h>

namespace calibration
{
    
}

void calibration::calibrate() {
    Encoders::setZeroSPI(pinConfig::encoder1);
    delay(500);
    Encoders::setZeroSPI(pinConfig::encoder2);
    delay(500);
    Encoders::setZeroSPI(pinConfig::encoder3);
    delay(500);
    Encoders::setZeroSPI(pinConfig::encoder4);
    delay(500);
    Encoders::setZeroSPI(pinConfig::encoder1b);
    delay(500);
    Encoders::setZeroSPI(pinConfig::encoder2b);
    delay(500);
    Encoders::setZeroSPI(pinConfig::encoder3b);
    delay(500);
    Encoders::setZeroSPI(pinConfig::encoder4b);
    delay(500);

    //Serial.println("Abs zeroed");

    hallEncoders::setZeroFor(0);
    hallEncoders::setZeroFor(1);
    hallEncoders::setZeroFor(2);
    
    hallEncoders::setZeroFor(3);
    hallEncoders::setZeroFor(4);
    hallEncoders::setZeroFor(5);
    
    //Serial.println("Hall zeroed");

}