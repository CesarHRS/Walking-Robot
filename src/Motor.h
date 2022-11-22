/**
 * @file Motor.h
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-11-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __MOTOR__H__
#define __MOTOR__H__

class Motor
{
private:
    short pinFoward;
    short pinBack;
    short pinSpeed;
public:
    Motor(short pinFoward, short pinBack, short pinSpeed);
    void begin();
    void setSpeed(short speed);
};

#endif  //!__MOTOR__H__