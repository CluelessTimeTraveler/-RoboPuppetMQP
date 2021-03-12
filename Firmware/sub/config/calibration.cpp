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
    Encoders::setZeroSPI(pinConfig::encoder2);
    Encoders::setZeroSPI(pinConfig::encoder3);
    Encoders::setZeroSPI(pinConfig::encoder4);
    
    Encoders::setZeroSPI(pinConfig::encoder1b);
    Encoders::setZeroSPI(pinConfig::encoder2b);
    Encoders::setZeroSPI(pinConfig::encoder3b);
    Encoders::setZeroSPI(pinConfig::encoder4b);

    //Serial.println("Abs zeroed");

    hallEncoders::setZeroFor(0);
    hallEncoders::setZeroFor(1);
    hallEncoders::setZeroFor(2);
    
    hallEncoders::setZeroFor(3);
    hallEncoders::setZeroFor(4);
    hallEncoders::setZeroFor(5);
    
    //Serial.println("Hall zeroed");

}