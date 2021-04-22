#include <SPI.h>
#include "hallEncoders.h"
#include <Arduino.h>
#include "AS5048A.h"
#include "pinConfig.h"
#include "Buttons.h"

namespace hallEncoders
{
  // SPI Pins - should be automatically selected
  const uint8_t SPI_COPI = pinConfig::SPI_COPI;     // MOSI pin
  const uint8_t SPI_CIPO = pinConfig::SPI_CIPO;     // MISO pin
  const uint8_t SPI_SCLK = pinConfig::SPI_SCLK;     // SLCK pin

  //Chip or Slave select
  const uint8_t encoder1 = pinConfig::hallEncode1;
  const uint8_t encoder2 = pinConfig::hallEncode2;
  const uint8_t encoder3 = pinConfig::hallEncode3;
  const uint8_t encoder4 = pinConfig::hallEncode1b;
  const uint8_t encoder5 = pinConfig::hallEncode2b;
  const uint8_t encoder6 = pinConfig::hallEncode3b;

  //Encoders 
  const uint8_t num_enc = 6;
  float angles[num_enc];
  const uint8_t encoderPins [num_enc] = {encoder1, encoder2, encoder3, encoder4, encoder5, encoder6};
  //int angle;

  // Init flag
  bool init_complete = false;

  //Make encoder objects?
  AS5048A* angleSensor = (AS5048A*) malloc(sizeof(AS5048A)*num_enc);
  //AS5048A angleSensor[num_enc];
  //AS5048A encoderSingle = AS5048A(encoder1);

  // Methods
}

/**
 * Initializes subsystem
 */
void hallEncoders::init()
{
  if (!init_complete)
  {
    //Set up encoder pins 
    pinMode(encoder1, OUTPUT);
    pinMode(encoder2, OUTPUT);
    pinMode(encoder3, OUTPUT);
    pinMode(encoder4, OUTPUT);
    pinMode(encoder5, OUTPUT);
    pinMode(encoder6, OUTPUT);
    
    //Set the CS line high which is the default inactive state
    digitalWrite(encoder1, HIGH);
    digitalWrite(encoder2, HIGH);
    digitalWrite(encoder3, HIGH);
    digitalWrite(encoder4, HIGH);
    digitalWrite(encoder5, HIGH);
    digitalWrite(encoder6, HIGH);

    //Nice screen things


    //initalize joint angles
    for (uint8_t j = 0; j < num_enc; j++)
		{
			angles[j] = 0.0f;
      angleSensor[j] = AS5048A(encoderPins[j]);
      angleSensor[j].init();
		}
    //angle = 0;

    // Set init flag
    init_complete = true;
  }
    // SPI.setClockDivider(SPI_CLOCK_DIV32);    // 1M Hz

    // SPI.begin();

    //encoderSingle.init();
}

/**
 * @brief Reads and stores each servo angle
 */
void hallEncoders::update()
{
  bool holdState = Buttons::getHoldStatus();
    if(!holdState){
      //Serial.println("Hall encoders update!");
      for (uint8_t j = 0; j < hallEncoders::num_enc; j++){
        angles[j] = angleSensor[j].getRotation();
        delay(50);
    }
  }
}

/**
 * @brief Transmits encoder value to RosComms
 */
int hallEncoders::getStatus(uint8_t encoderNumber)
{
  int tempMap;
  //tempMap = map(hallEncoders::angles[encoderNumber], 0, 16384, 0, 359);
  int angleVal = (int)hallEncoders::angles[encoderNumber];

  if(angleVal <= 0){
    tempMap = map(angleVal, -8192, 0, 180, 0);
  }else{
    tempMap = map(angleVal, 0, 8192, 360, 180);
  }
  //tempMap = map(angleVal, -8192, 8192, 0, 360); //Mapping is changed to 0 360 for real arm, was -180 to 180
    //tempMap = map(tempMap, 0, 359, -180, 180);
  
  return tempMap;
  // float tempMap;
  // tempMap = map(hallEncoders::angle, 0, 16384, 1, 360);
  // return (int)tempMap;
}

void hallEncoders::setZeroFor(int encoder){

  //Serial.println()

  angleSensor[encoder].setZeroPosition(angleSensor[encoder].getRotation());
  delay(500);
}