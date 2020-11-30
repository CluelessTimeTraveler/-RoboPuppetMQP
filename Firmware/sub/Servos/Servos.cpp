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
    const uint8_t numServos = 6;
    uint8_t servoPins[numServos] = {8, 9, 10, 11, 12, 13};;
    uint8_t setPos[numServos];
    bool holdState;
    Servo myServo[6];
    int* servoAngle;
    int* getAngles();

    //Servo methods
    void attachServos();
    void goToHomePosition();
}

void Servos::init(){
    holdState = Buttons::getHoldStatus();
    //Attach each servo with correct PWM range
    attachServos();
    goToHomePosition();
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
    //Map encoder ranges to servo ranges 
    int readAngle[numServos];
    readAngle[0] = 1500; //Encoders::getStatus(0);
    readAngle[1] = 0; //;
    readAngle[2] = 1500; //Encoders::getStatus(1);
    readAngle[3] = 0; //40;
    readAngle[4] = 1500; //Encoders::getStatus(2);
    readAngle[5] = 0; //60;

    return readAngle;
}

void Servos::attachServos(){
    //Attaches each servo with the correct PWM 
    myServo[0].attach(servoPins[0]);
    myServo[1].attach(servoPins[1], 800, 2200);
    myServo[2].attach(servoPins[2]);
    myServo[3].attach(servoPins[3], 553, 2520);
    myServo[4].attach(servoPins[4]);
    myServo[5].attach(servoPins[5], 553, 2270);
}

void Servos::goToHomePosition(){
    //Moves arm into completely upright position
    Serial.println("Goin home");
    myServo[0].write(1500);
    Serial.println("Lifting...");
    for(int i = 20; i < 170; i++){
        myServo[1].write(i);
        delay(100);
    }   
    delay(1000);
    Serial.println("Lift complete");
    myServo[2].write(1500);
    myServo[3].write(65);
    delay(1000);
    myServo[4].write(1500);
    myServo[5].write(49);
    delay(1000);
}