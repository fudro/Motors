/************************************
 * This program demonstrates control of a DC Motor using the Adafruit Motorshield V1.
 * 
 * REFERENCE:
 * https://learn.adafruit.com/adafruit-motor-shield/overview
 */

#include <AFMotor.h>

AF_DCMotor motor(3);    //Create motor object and set motor port (1-4)

void setup() {
  Serial.begin(115200);           // set up Serial library at 9600 bps
  Serial.println("DC Motor test!");

  // turn on motor
  motor.setSpeed(255);  //Set default motor speed. Speeds allowed from 0 to 255.
  motor.run(RELEASE);   //Reset the motor (clear any previous commands) by "releasing" it.
}

void loop() {
  uint8_t i;
  
  motor.run(FORWARD);
  for (i=0; i<255; i++) {   //Ramp up speed.
    motor.setSpeed(i);  
    delay(10);
 }
 delay (5000);
 
  for (i=255; i!=0; i--) {  //Ramp down speed.
    motor.setSpeed(i);  
    delay(10);
 }

  motor.run(BACKWARD);
  for (i=0; i<255; i++) {   //Ramp up speed.
    motor.setSpeed(i);  
    delay(10);
 }
 
  for (i=255; i!=0; i--) {  //Ramp down speed.
    motor.setSpeed(i);  
    delay(10);
 }
  
  motor.run(RELEASE);
  delay(1000);
}
