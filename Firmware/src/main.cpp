#include <SerialInterface.h>
#include <Encoders.h>
#include <Buttons.h>
#include <Arduino.h>
#include <Servos.h>
#include <hallEncoders.h>

void setup() {
  SerialInterface::init();
  Encoders::init();
  hallEncoders::init();
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
