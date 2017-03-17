#include <DCMotorControl.h>

 /* Define pins */ 
  const int sensorPin = A0;
  const int pwmPin = 3;
  const int motorFwd = 5;
  const int motorBck = 6;
  boolean dir;
  boolean up;
  int input;
  DCMotorControl motor(pwmPin, motorFwd, motorBck);


void setup() {
  // put your setup code here, to run once:
  motor.direction(1);
  dir = true;
  up = true;
  input = 0;
}

void loop() {

/*
 *    |
 *255 |  /\          /\
 *    | /  \        /  \
 *    |/ ___\______/____\___________t
 *    |      \    /      \
 *    |       \  /        \
 -255 |        \/   
 *    |
 */

  // put your main code here, to run repeatedly:
  if (up) {
    if (input == 255) {
      up = false;
      input--;       
    } else {
      if (input == 0) {
        dir = true;
      }
      input++;
    }
  } else {
    if (input == -255) {
      up = true;
      input++;
    } else {
      if (input == 0) {
        dir = false;
      }
      input--;
    }
  }

  motor.direction(dir);
  motor.driveMotor(abs(input));
  delay(100);
}
/*
int abs(int num) {
  if (num < 0) {
    num = -1*num;
  }
  return num;
}*/

