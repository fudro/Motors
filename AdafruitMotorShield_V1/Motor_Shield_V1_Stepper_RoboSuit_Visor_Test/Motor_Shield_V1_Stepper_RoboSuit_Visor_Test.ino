#include <AFMotor.h>


AF_Stepper motor(32, 2);

int stepperSpeed = 1000;
long stepCount = 5000;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Stepper test!");

  Serial.print("TotalSteps: ");
  Serial.println(stepCount);

  motor.setSpeed(stepperSpeed);  // "rpm"   
  Serial.print("Speed: ");
  Serial.println(stepperSpeed);

  motor.step(100, FORWARD, SINGLE); 
  motor.release();
  delay(1000);
}

void loop() {
//  Serial.println("Forward Single");
//  motor.step(1000, FORWARD, SINGLE); 
//  motor.step(1000, BACKWARD, SINGLE); 

  Serial.println("Forward Double");
  motor.step(stepCount, FORWARD, DOUBLE); 
  motor.step(stepCount, BACKWARD, DOUBLE);

//  Serial.println("Forward Interleave");
//  motor.step(1000, FORWARD, INTERLEAVE); //This mode performs about half the travel
//  motor.step(1000, BACKWARD, INTERLEAVE); 
//
//  Serial.println("Forward Microstep");
//  motor.step(1000, FORWARD, MICROSTEP); 
//  motor.step(1000, BACKWARD, MICROSTEP); 
}
