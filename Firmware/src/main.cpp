#include <SerialInterface.h>
#include <Encoders.h>
#include <Buttons.h>
#include <Arduino.h>
#include <Servos.h>
#include <hallEncoders.h>
#include <calibration.h>
#include <InfoLCD.h>

void setup() {
  //delay(5000);
  SerialInterface::init();
  InfoLCD::init();
  Encoders::init();
  hallEncoders::init();
  //Servos::init();
  calibration::calibrate();
  Buttons::init();
}

void loop() {
  delay(2000);
  Encoders::update();
  hallEncoders::update();
  Buttons::update();
  Servos::update();
  SerialInterface::update();

}
