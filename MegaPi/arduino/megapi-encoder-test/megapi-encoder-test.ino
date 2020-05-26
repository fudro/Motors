/*
 * This program illustrates motor control using an encoder.
 * 
 * HARDWARE:
 * MakeBlock Mega Pi
 * 
 */

#include "MeMegaPi.h"

MeMegaPiDCMotor motor1(PORT2A);
MeMegaPiDCMotor motor2(PORT2B);

int motorSpeed = 125; /* value: between -255 and 255. */
const byte MOTOR1 = A6;   //assign analog pins to each motor
const byte MOTOR2 = A7;
int encoder1 = 0;         //variables to count encoder disc "ticks" each time the encoder circuitry sends an interrupt pulse. 
int encoder2 = 0;
bool lastState1 = false;
bool lastState2 = false;
bool moving = false;
int encoderTarget = 40;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Encoder Test!!");
  pinMode(MOTOR1, INPUT);
  pinMode(MOTOR2, INPUT);
}

void loop() {
  motorTest();
  delay(500);
}

void motorTest() {
  Serial.print("Motor Speed: ");
  Serial.println(motorSpeed);
  moving = true;
  motor1.run(motorSpeed);
  motor2.run(motorSpeed);
  while (encoder1 < encoderTarget && encoder2 < encoderTarget) {
    checkEncoders();
  }
  motor1.stop();
  motor2.stop();
  resetEncoders();
  motorSpeed = motorSpeed * -1;   //Reverse direction
  moving = false;
}

void checkEncoders() {
  //Update Motor 1
  if (analogRead(MOTOR1) > 900 && lastState1 == false) {
    lastState1 = HIGH;
    encoder1++;
    Serial.print("MOTOR1 H: ");
    Serial.println(encoder1);
  }
  else if (analogRead(MOTOR1) < 100 && lastState1 == true) {
    lastState1 = LOW;
    encoder1++;
    Serial.print("MOTOR1 L: ");
    Serial.println(encoder1);
  }

  //Update Motor 2
  if (analogRead(MOTOR2) > 900 && lastState2 == false) {
    lastState2 = HIGH;
    encoder2++;
    Serial.print("MOTOR2 H: ");
    Serial.println(encoder2);
  }
  else if (analogRead(MOTOR2) < 100 && lastState2 == true) {
    lastState2 = LOW;
    encoder2++;
    Serial.print("MOTOR2 L: ");
    Serial.println(encoder2);
  }
}

void resetEncoders() {
  encoder1 = 0;
  encoder2 = 0;
}
