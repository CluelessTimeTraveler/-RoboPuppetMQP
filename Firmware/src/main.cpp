#include <SerialInterface.h>
#include <Encoders.h>
#include <Buttons.h>
#include <Arduino.h>
#include <Servos.h>
#include <hallEncoders.h>
#include <calibration.h>

void setup() {
  delay(5000);
  SerialInterface::init();
  Encoders::init();
  hallEncoders::init();
  Servos::init();
  calibration::calibrate();
  //Buttons::init();
  //Servos::init();
}

void loop() {
  //Serial.println("Begin loop:");
  Encoders::update();
  hallEncoders::update();
  //Buttons::update();
  //Servos::update();
  SerialInterface::update();
  


}
