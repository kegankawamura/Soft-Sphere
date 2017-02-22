#include <DCMotorControl.h>
int en1 = 3;
int in1 = 5;
int in2 = 6;
DCMotorControl motor(en1, in1, in2);

void setup() {
  Serial.begin(9600);
}

void loop() {
  int input;
  if (Serial.available()) {
    input = Serial.parseInt();
    motor.driveMotor(input);
    Serial.print("driveMotor set to: ");
    Serial.println(input);
  }
}
