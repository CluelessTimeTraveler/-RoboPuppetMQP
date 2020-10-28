#include <SPI.h>
#include "hallEncoders.h"
#include <Arduino.h>

/**
 * THIS IS IN PROGRESS SORRY IT IS BROKE :(
 */
namespace hallEncoders
{
  // SPI Pins - should be automatically selected
  const uint8_t SPI_MOSI = 11;     // MOSI pin
  const uint8_t SPI_MISO = 12;     // MISO pin
  const uint8_t SPI_SCLK = 13;     // SLCK pin

  //Chip or Slave select
  const uint8_t encoder1 = 2;
  const uint8_t encoder2 = 3;
  const uint8_t encoder3 = 4;

  //SPI commands
  const uint8_t AS5048_NOP = 0x0000;
  const uint8_t AS5048_RESET = 0x0001;
  const uint8_t AS5048_ANGLE = 0x3FFF;

  //Encoders 
  const uint8_t num_enc = 2;
  float angles[num_enc];
  const uint8_t encoderPins [num_enc] = {encoder1, encoder2};

  // Init flag
  bool init_complete = false;

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

    pinMode(SPI_SCLK, OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, INPUT);

    //Serial.begin(115200);
    
    //Set the CS line high which is the default inactive state
    digitalWrite(encoder1, HIGH);
    digitalWrite(encoder2, HIGH);
    digitalWrite(encoder3, HIGH);
    //digitalWrite(encoder4, HIGH);

    SPI.setClockDivider(SPI_CLOCK_DIV32);    // 1M Hz

    SPI.begin();
    //SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  
    //Nice screen things
    Serial.println("Encoders Initialized");

    //initalize joint angles
    for (uint8_t j = 0; j < num_enc; j++)
		{
			angles[j] = 0.0f;
		}

    // Set init flag
    init_complete = true;
  }
}


/**
 * @brief Reads and stores each servo angle
 */
void hallEncoders::update()
{
	
}

/**
 * @brief Transmits servo value to RosComms
 */
float hallEncoders::getStatus(uint8_t servo)
{
  return 
}

/**
 * @brief Gets Position value from the Servo
 */
uint8_t hallEncoders::getPosition(uint8_t servo_num)
{
    sensorValue = myservo[servo_num].read();
    return sensorValue; 
}

///////////////////////// Begin encoder read /////////////////////////////////////

uint8_t Encoders:: spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  //holder for the received over SPI
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  digitalWrite(encoder ,LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  digitalWrite(encoder, releaseLine); //if releaseLine is high set it high else it stays low
  
  return data;
}

uint16_t hallEncoders::getPositionSPI(uint8_t encoder)
{
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AS5048_READ, encoder, false) << 8;   

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //OR the low byte with the currentPosition variable. release line after second byte
  uint16_t temp; 
  temp = spiWriteRead(AS5048_READ, encoder, true);    
  currentPosition |= temp;      

  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition &= 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }

  return currentPosition;
}

uint16_t hallEncoders::updateSingle(uint8_t encoder)
{
    //Serial.println("Encoders Update Begin");
    uint16_t encoderPosition;
    //let's also create a variable where we can count how many times we've tried to obtain the position in case there are errors
    uint8_t attempts;

   //set attemps counter at 0 so we can try again if we get bad position    
    attempts = 0;

    //this function gets the encoder position and returns it as a uint16_t
    //send the function either res12 or res14 for your encoders resolution
    encoderPosition = getPositionSPI(encoder); 

    //check error bit 
    //make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
    while (encoderPosition == 0xFFFF && ++attempts < 3)
    {
      encoderPosition = getPositionSPI(encoder); //try again
    }

    return encoderPosition;
    //For the purpose of this demo we don't need the position returned that quickly so let's wait a half second between reads
    //delay() is in milliseconds
    delay(500);
}