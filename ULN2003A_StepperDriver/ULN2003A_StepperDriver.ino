/*
 * This program uses a ULN2003 stepper motor driver to control a stepper motor
 * 
 * HARDWARE:
 * Arduino Mega 2560
 * Stepper Motor: 28BYJ-48 12v unipolar stepper motor
 * 
 */

#include <Stepper.h>

const float STEPS_PER_REV = 32;   //Number of steps for one revolution of the internal drive shaft
const float GEAR_REDUCTION = 64;  //Internal stepper motor gear reduction of 64 turns of internal shaft to 1 turn of output shaft
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_REDUCTION;   //Number of steps required to rotate the output shaft 1 revolution
int stepsRequired;        //Number of steps required for the current command
int stepperState = 0;     //Track current position of stepper motor with regards to teh switch it is activating
int stepperSpeed = 1000;  //NOTE: 1000 is the max speed of the stepper motor!
unsigned long startTime;
unsigned long stopTime;
int count = 0;
Stepper stepperMotor_FL(STEPS_PER_REV, 46, 50, 48, 52);    //Define stepper motor and the pins used to control the coils


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Stepper Speed Test!");
}

void loop() {
  // put your main code here, to run repeatedly:
  while(count < 20){
    //Rotate CW
    stepperState = 1;
    stepsRequired = STEPS_PER_OUT_REV;    //Rotate a fraction of a revolution. Change divisor value to adjust amount of rotation. Change sign to adjust direction of rotation.
    stepperMotor_FL.setSpeed(stepperSpeed);
    startTime = millis();
    stepperMotor_FL.step(stepsRequired);
    stopTime = millis();
    Serial.print("Speed: ");
    Serial.print(stepperSpeed);
    Serial.print("\t");
    Serial.print("Rotation Time: ");
    Serial.println((stopTime - startTime)/1000);
    delay(3000);
    //Rotate CCW
    stepsRequired = -STEPS_PER_OUT_REV/2;
    stepperMotor_FL.setSpeed(stepperSpeed);
    stepperMotor_FL.step(stepsRequired);
    stepperState = 0;
    delay(3000);
    count = count + 1;
  }
}
