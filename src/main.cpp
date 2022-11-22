/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-21
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "main.h"

KeyToIno keyboard;

Motor motorRight = Motor(
    MOTOR_RIGHT_FOWARD_PIN,
    MOTOR_RIGHT_BACK_PIN,
    MOTOR_RIGHT_SPEED_PIN);

Motor motorLeft = Motor(
    MOTOR_LEFT_FOWARD_PIN,
    MOTOR_LEFT_BACK_PIN,
    MOTOR_LEFT_SPEED_PIN);

void setup()
{
  // put your setup code here, to run once:
  keyboard.begin(9600);
  motorRight.begin();
  motorLeft.begin();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}

void loop()
{
  keyboard.readKeys();

  if (keyboard.isPressed(keys::KEY_Up))
  {
    motorRight.setSpeed(255);
    motorLeft.setSpeed(255);
  }
  else if (keyboard.isPressed(keys::KEY_Down))
  {
    motorRight.setSpeed(-255);
    motorLeft.setSpeed(-255);
  }
  else if (keyboard.isPressed(keys::KEY_Left))
  {
    motorRight.setSpeed(255);
    motorLeft.setSpeed(-255);
  }
  else if (keyboard.isPressed(keys::KEY_Right))
  {
    motorRight.setSpeed(-255);
    motorLeft.setSpeed(255);
  }
  else
  {
    motorRight.setSpeed(0);
    motorLeft.setSpeed(0);
  }
}