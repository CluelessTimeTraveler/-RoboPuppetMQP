#include <SerialInterface.h>
#include <Encoders.h>
#include <Buttons.h>

void setup() {
  SerialInterface::init();
  Encoders::init();
  Buttons::init();
}

void loop() {
  //Encoders::update();//To-do for bella
  Buttons::update();
  SerialInterface::update();
  


}
