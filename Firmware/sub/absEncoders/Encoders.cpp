#include <SPI.h>
#include "Encoders.h"
#include <Arduino.h>

/**
 * Private subsystem info
 */
namespace Encoders
{
  const uint8_t RES12 = 12;
  const uint8_t RES14 = 14;

  // SPI Pins - should be automatically selected
  const uint8_t SPI_MOSI = 51;     // MOSI pin
  const uint8_t SPI_MISO = 50;     // MISO pin
  const uint8_t SPI_SCLK = 52;     // SLCK pin

  //Chip or Slave select
  const uint8_t encoder1 = 2;
  const uint8_t encoder2 = 3;
  const uint8_t encoder3 = 4;
  const uint8_t encoder4 = 21;

  //SPI commands
  const uint8_t AMT22_NOP = 0x00;
  const uint8_t AMT22_RESET = 0x60;
  const uint8_t AMT22_ZERO = 0x70;

  //Encoders 
  const uint8_t num_enc = 4;
  float angles[num_enc];
  const uint8_t encoderPins [num_enc] = {encoder1, encoder2, encoder3, encoder4};

  // Init flag
  bool init_complete = false;

  // Methods
  uint8_t spiTransmit(uint8_t encoder, uint8_t msg, uint8_t releaseLine);
}

/**
 * Initializes subsystem
 */
void Encoders::init()
{
  if (!init_complete)
  {
    //Set up encoder pins 
    pinMode(encoder1, OUTPUT);
    pinMode(encoder2, OUTPUT);
    pinMode(encoder3, OUTPUT);
    pinMode(encoder4, OUTPUT);

    pinMode(SPI_SCLK, OUTPUT);
    pinMode(SPI_MOSI, OUTPUT);
    pinMode(SPI_MISO, INPUT);

    //Serial.begin(115200);
    
    //Set the CS line high which is the default inactive state
    digitalWrite(encoder1, HIGH);
    digitalWrite(encoder2, HIGH);
    digitalWrite(encoder3, HIGH);
    digitalWrite(encoder4, HIGH);

    SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz

    SPI.begin();
    //SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  
    //Nice screen things
    //Serial.println("Encoders Initialized");

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
 * @brief Reads and stores each encoder angle
 * Default to 12 bit resolution
 */

void Encoders::update()
{
	for (uint8_t j = 0; j < Encoders::num_enc; j++)
	{
    //Serial.print("Update encoder:");
    //Serial.println(j);
		angles[j] = Encoders::updateSingle(encoderPins[j]);
    //Serial.println(angles[j]); //print the position in decimal format
    delay(50);
	}
  //Serial.println();
  //Serial.println();
}


/**
 * @brief Transmits encoder value to RosComms
 * @param encoder number [1,2,3,4]
 */
int Encoders::getStatus(uint8_t encoder)
{
  float tempMap;
  tempMap = map(Encoders::angles[encoder], 0, 4096, 1, 360);
  return (int)tempMap;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//BEGIN AMT22 CODE
/**
 * @brief Transmits message to encoder
 * @param encoder number [1,2,3,4], msg (see SPI Commands above), releaseLine [high, low]
 */
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

uint16_t Encoders::getPositionSPI(uint8_t encoder, uint8_t resolution)
{
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, encoder, false) << 8;   

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //OR the low byte with the currentPosition variable. release line after second byte
  uint16_t temp; 
  temp = spiWriteRead(AMT22_NOP, encoder, true);    
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

  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;

  return currentPosition;
}

uint16_t Encoders::updateSingle(uint8_t encoder)
{
    //Serial.println("Encoders Update Begin");
    uint16_t encoderPosition;
    //let's also create a variable where we can count how many times we've tried to obtain the position in case there are errors
    uint8_t attempts;

   //set attemps counter at 0 so we can try again if we get bad position    
    attempts = 0;

    //this function gets the encoder position and returns it as a uint16_t
    //send the function either res12 or res14 for your encoders resolution
    encoderPosition = getPositionSPI(encoder, RES12); 

    //if the position returned was 0xFFFF we know that there was an error calculating the checksum
    //make 3 attempts for position. we will pre-increment attempts because we'll use the number later and want an accurate count
    while (encoderPosition == 0xFFFF && ++attempts < 3)
    {
      encoderPosition = getPositionSPI(encoder, RES12); //try again
    }

    //Serial.println("Attempts Complete");

    if (encoderPosition == 0xFFFF) //position is bad, let the user know how many times we tried
    {
      //Serial.print("Encoder 1 error. Attempts: ");
      //Serial.println(attempts, DEC); //print out the number in decimal format. attempts - 1 is used since we post incremented the loop
    }
    else //position was good, print to serial stream
    {
      //Serial.println(encoderPosition, DEC); //print the position in decimal format
    }

    return encoderPosition;
    //For the purpose of this demo we don't need the position returned that quickly so let's wait a half second between reads
    //delay() is in milliseconds
}