/*
  DCMotorControl.h - Library for using an H bridge for DC motor control 
  Created by Keenan Rodewald, November 18, 2016
  Released into the public domain.
*/
#ifndef DCMotorControl_h
#define DCMotorControl_h

#include "Arduino.h"

class DCMotorControl 
{
  public:

    DCMotorControl(int ent1, int in1, int in2);
    void driveMotor(int val);
    void direction(boolean dirr);
  private:
    int _ent1;
    int _in1;
    int _in2;
};

#endif