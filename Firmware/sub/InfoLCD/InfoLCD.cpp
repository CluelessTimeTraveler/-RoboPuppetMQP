#include <Arduino.h>
#include "InfoLCD.h"
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD

namespace InfoLCD
{
    LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); // Change to (0x27,16,2) for 16x2 LCD.
}

bool InfoLCD::init(){
  lcd.init();
  lcd.backlight();
  lcd.clear();

  return true;
}

void InfoLCD::printToLCD(String toPrint){

    lcd.clear();
    if(toPrint.length() > 32){
        lcd.setCursor(0,0);
        lcd.print("Message too long");
    }

    if(toPrint.length()>16){
        lcd.setCursor(0,0);
        lcd.print(toPrint.substring(0,16));
        lcd.setCursor(0,1);
        lcd.print(toPrint.substring(16));
    }else{
        lcd.print(toPrint);
    }

    
}