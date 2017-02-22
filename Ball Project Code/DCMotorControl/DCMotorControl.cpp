/* 
DCMotorControl.cpp - Library for using an H bridge for DC motor control 
  Created by Keenan Rodewald, November 18, 2016
  Released into the public domain.
*/

#include "Arduino.h"
#include "DCMotorControl.h"

DCMotorControl::DCMotorControl(int ent1, int in1, int in2)
{
  _ent1 = ent1;
  _in1 = in1;
  _in2 = in2;
  pinMode(_ent1, OUTPUT);
  pinMode(_in1, OUTPUT);
  pinMode(_in2, OUTPUT);
}


/* Sets the direction on H bridge
*/
void DCMotorControl::direction(boolean dirr)
{
  if (dirr == true) {
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
  }
  else {
    digitalWrite(_in1, HIGH);
    digitalWrite(_in2, LOW);
  }
}


/* First determines and sets direction based on input sign, then outputs 
* level (saturated to 255) to PWM output. 
*/
void DCMotorControl::driveMotor(int val)
{
  if (val >= 255) { //first set direction based on sign 
    direction(1);
  }
  else {
    direction(0);
  }

  val = abs(val);
  if (val > 255) { //saturate to 255
    val = 255;
  }
  analogWrite(_ent1, val);
}



