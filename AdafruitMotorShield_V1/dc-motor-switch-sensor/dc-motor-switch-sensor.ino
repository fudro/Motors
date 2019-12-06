/************************************
 * This program uses the state of a switch to control a DC Motor.
 * The motor is powered using the Adafruit Motorshield V1.
 * 
 * REFERENCE:
 * https://learn.adafruit.com/adafruit-motor-shield/overview
 */

#include <AFMotor.h>

AF_DCMotor motor(4);    //Create motor object and set motor port (1-4)

const int switchPin = 17;  //Set switch to the digital equivalent of analog pin A3

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("DC Motor Switch Test!");

  // Setup motor
  motor.setSpeed(200);  //Set default motor speed. Speeds allowed from 0 to 255.
  motor.run(RELEASE);   //Reset the motor (clear any previous commands) by "releasing" it.
}

void loop() {   //Rotate motor and pause at each activation of the switch, then continue.
  motor.run(FORWARD);
  while (digitalRead(switchPin) == HIGH) {
    //wait for switch to release
  }
  while (digitalRead(switchPin) == LOW) {
    //wait for switch to become pressed
  }
  delay (10);
  motor.run(RELEASE);   
  delay(3000);
}
