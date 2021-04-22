/**
 * @file Servos.cpp
 */

#include <Servo.h> //The Adruino one
#include "Servos.h" //The specific header for this file
#include <Buttons.h>
#include <Encoders.h>
#include <hallEncoders.h>
#include <Arduino.h>
#include <InfoLCD.h>

/**
 * Private subsystem info
 */

namespace Servos
{
    // define all servo pins
    const uint8_t numServos = 12;
    uint8_t servoPins[numServos] = {8, 9, 10, 11, 12, 46, 29, 30, 31, 32, 33, 34};
    uint8_t setPos[numServos];
    bool holdState;
    Servo myServo[12];
    int servoAngles[12];

    //Servo methods
    void getServoAngles();
    void attachServos();
    void goToHomePosition();
}

void Servos::init(){
    holdState = Buttons::getHoldStatus();
    //Attach each servo with correct PWM range
    InfoLCD::printToLCD("Lifting...");
    attachServos();
    //goToHomePosition();
}

void Servos::update(){
    holdState = Buttons::getHoldStatus();
    if(holdState){
        //read all encoder  values
        getServoAngles(); 
        //Serial.println("Hold position:");
        for(uint8_t i = 0; i<numServos; i++){
            myServo[i].write(servoAngles[i]); //set servos to each position read in from the encoders
            //Serial.println((servoAngles[i]));
        }
        attachServos();
    }
    else{
        //Serial.println("Stop hold--");
        for(uint8_t i = 0; i<numServos; i++)
            myServo[i].detach(); //detach all servo objects 
    }
    
}

void Servos::getServoAngles(){
    //Map encoder ranges to servo ranges (found experimentally)
    servoAngles[0] = 1500; 
    servoAngles[1] = map(hallEncoders::getStatus(0), -22, 131, 180, 10) + 5; 
    servoAngles[2] = 1500; 
    servoAngles[3] = map(hallEncoders::getStatus(1), -36, 127, 15, 175) - 5; 
    servoAngles[4] = 1500; 
    servoAngles[5] = map(hallEncoders::getStatus(2), -55, 105, 10, 160) - 10;
    //Second arm
    servoAngles[6] = 1500; 
    servoAngles[7] = map(hallEncoders::getStatus(0), -22, 131, 180, 10) + 5; 
    servoAngles[8] = 1500; 
    servoAngles[9] = map(hallEncoders::getStatus(1), -36, 127, 15, 175) - 5; 
    servoAngles[10] = 1500; 
    servoAngles[11] = map(hallEncoders::getStatus(2), -55, 105, 10, 160) - 10;
}

void Servos::attachServos(){
    //Attaches each servo with the correct PWM 
    myServo[0].attach(servoPins[0]);
    myServo[1].attach(servoPins[1], 800, 2200);
    myServo[2].attach(servoPins[2]);
    myServo[3].attach(servoPins[3], 553, 2270);
    myServo[4].attach(servoPins[4]);
    myServo[5].attach(servoPins[5], 553, 2270);
    //Second arm
    myServo[6].attach(servoPins[0]);
    myServo[7].attach(servoPins[1], 800, 2200);
    myServo[8].attach(servoPins[2]);
    myServo[9].attach(servoPins[3], 553, 2270);
    myServo[10].attach(servoPins[4]);
    myServo[11].attach(servoPins[5], 553, 2270);
}

// Moves arm into completely upright position 
void Servos::goToHomePosition(){
    //InfoLCD::printToLCD("Going to home position");
    myServo[0].write(1500);
    //Serial.println("Lifting...");
    // for(int i = 20; i < 170; i++){
    //     myServo[1].write(i);
    //     delay(100);
    // }   
    myServo[1].write(170);
    delay(1000);
    myServo[2].write(1500);
    myServo[3].write(45);
    delay(1000);
    myServo[4].write(1500);
    myServo[5].write(40);
    delay(2000); //time to settle
}

void Servos::goToHomePosition2(){
    //Moves arm into completely upright position
    //InfoLCD::printToLCD("Going to home position");
    myServo[6].write(1500);
    //Serial.println("Lifting...");
    // for(int i = 20; i < 170; i++){
    //     myServo[1].write(i);
    //     delay(100);
    // }   
    myServo[7].write(170);
    delay(1000);
    myServo[8].write(1500);
    myServo[9].write(45);
    delay(1000);
    myServo[10].write(1500);
    myServo[11].write(40);
    delay(2000); //time to settle
}