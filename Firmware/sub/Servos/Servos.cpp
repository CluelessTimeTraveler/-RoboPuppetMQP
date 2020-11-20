/**
 * @file Servos.cpp
 */

#include <Servo.h> //The Adruino one
#include "Servos.h" //The specific header for this file
#include <Buttons.h>
#include <Encoders.h>
#include <Arduino.h>

/**
 * Private subsystem info
 */

namespace Servos
{
    // define all servo pins
    const uint8_t numServos = 7;
    uint8_t servoPins[numServos] = {6, 7, 8, 9, 10, 11, 12};
    uint8_t setPos[7];
    bool holdState;
    Servo myServo[numServos];
    int* servoAngle;
    int* getAngles();
}

void Servos::init(){
    holdState = Buttons::getHoldStatus();
    //create servo object 
}

void Servos::update(){
    holdState = Buttons::getHoldStatus();
    if(holdState){
        //read all encoder  values
        servoAngle = getAngles(); 
        for(uint8_t i = 0; i<numServos; i++){
            myServo[i].write(*(servoAngle+i)); //set servos to each position read in from the encoders
            myServo[i].attach(servoPins[i]); //attach servo objects 
        }
    }
    else{
        for(uint8_t i = 0; i<numServos; i++)
            myServo[i].detach(); //detach all servo objects 
    }
    
}

int* Servos::getAngles(){
    int readAngle[numServos];
    readAngle[0] = 0; //Encoders::getStatus(0);
    readAngle[1] = 0; //20;
    readAngle[2] = 0; //Encoders::getStatus(1);
    readAngle[3] = 0; //40;
    readAngle[4] = 0; //Encoders::getStatus(2);
    readAngle[5] = 0; //60;
    readAngle[6] = 0; //Encoders::getStatus(3);

    return readAngle;
}