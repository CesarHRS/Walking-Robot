/**
 * @file Motor.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-11-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "Arduino.h"
#include "Motor.h"

Motor::Motor(short pinFoward, short pinBack, short pinSpeed)
{
    this->pinFoward = pinFoward;
    this->pinBack = pinBack;
    this->pinSpeed = pinSpeed;
}

void Motor::begin()
{
    pinMode(pinFoward, OUTPUT);
    pinMode(pinBack, OUTPUT);
    pinMode(pinSpeed, OUTPUT);
    digitalWrite(pinFoward, LOW);
    digitalWrite(pinBack, LOW);
    analogWrite(pinSpeed, 0);
}

void Motor::setSpeed(short speed)
{
    digitalWrite(pinFoward, speed > 0);
    digitalWrite(pinBack, speed < 0);
    analogWrite(pinSpeed, abs(speed));
}
