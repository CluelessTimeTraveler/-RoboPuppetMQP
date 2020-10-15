#include <SerialInterface.h>
#include <Encoders.h>
#include <Buttons.h>
#include <Arduino.h>

void setup() {
  SerialInterface::init();
  Encoders::init();
  //Buttons::init();
}

void loop() {
  //Serial.println("Begin loop:");
  Encoders::update();//To-do for bella
  //Buttons::update();
  SerialInterface::update();
  


}
