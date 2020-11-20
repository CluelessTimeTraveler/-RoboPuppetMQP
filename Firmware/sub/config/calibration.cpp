/**
 * @file callibration.cpp
 */

#pragma once
#include "callibration.h"
#include <Encoders.h>
#include <hallEncoders.h>
#include "pinConfig.h"
#include <stdint.h>


namespace calibration
{
    
}

void calibration::calibrate() {
    Encoders::setZeroSPI(pinConfig::encoder1);
    Encoders::setZeroSPI(pinConfig::encoder2);
    Encoders::setZeroSPI(pinConfig::encoder3);
    Encoders::setZeroSPI(pinConfig::encoder4);

    hallEncoders::setZeroFor(1);
    hallEncoders::setZeroFor(2);
    hallEncoders::setZeroFor(3);
}