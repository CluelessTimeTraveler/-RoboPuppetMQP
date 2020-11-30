#include <SPI.h>
#include "hallEncoders.h"
#include <Arduino.h>
#include "AS5048A.h"
#include "pinConfig.h"

namespace hallEncoders
{
  // SPI Pins - should be automatically selected
  const uint8_t SPI_MOSI = pinConfig::SPI_MOSI;     // MOSI pin
  const uint8_t SPI_MISO = pinConfig::SPI_MISO;     // MISO pin
  const uint8_t SPI_SCLK = pinConfig::SPI_SCLK;     // SLCK pin


  //Chip or Slave select
  const uint8_t encoder1 = pinConfig::hallEncode1;
  const uint8_t encoder2 = pinConfig::hallEncode2;
  const uint8_t encoder3 = pinConfig::hallEncode3;

  //Encoders 
  const uint8_t num_enc = 3;
  float angles[num_enc];
  const uint8_t encoderPins [num_enc] = {encoder1, encoder2, encoder3};
  //int angle;

  // Init flag
  bool init_complete = false;

  //Make encoder objects?
  AS5048A* angleSensor = (AS5048A*) malloc(sizeof(AS5048A)*num_enc);
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
    
    //Set the CS line high which is the default inactive state
    digitalWrite(encoder1, HIGH);
    digitalWrite(encoder2, HIGH);
    digitalWrite(encoder3, HIGH);
    //digitalWrite(encoder4, HIGH);

    //Nice screen things
    Serial.println("Hall Encoders Initialized");

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
		for (uint8_t j = 0; j < hallEncoders::num_enc; j++){
		  angles[j] = angleSensor[j].getRotation();
      delay(50);
	}

  //angle = encoderSingle.getRawRotation();
  //Serial.println(angle);

}

/**
 * @brief Transmits encoder value to RosComms
 */
int hallEncoders::getStatus(uint8_t encoderNumber)
{
  int tempMap;
  //tempMap = map(hallEncoders::angles[encoderNumber], 0, 16384, 0, 359);
  int angleVal = (int)hallEncoders::angles[encoderNumber];
  tempMap = map(angleVal, -8152, 8152, 0, 359);
  tempMap = map(tempMap, 0, 359, -180, 180);
  return tempMap;

  // float tempMap;
  // tempMap = map(hallEncoders::angle, 0, 16384, 1, 360);
  // return (int)tempMap;
}

void hallEncoders::setZeroFor(int encoder){
  angleSensor[encoder].setZeroPosition(angleSensor[encoder].getRotation());
}