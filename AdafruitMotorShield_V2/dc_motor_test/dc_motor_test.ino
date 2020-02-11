/*************************
 * This program demonstrates basic motor control.
 * 
 * HARDWARE:
 * Adafruit Motoshield V2.3
 ************************/

#include <Adafruit_MotorShield.h>
 

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Connect a DC motors on ports M3 and M4 (refer to the PCB screenprint)
Adafruit_DCMotor *Motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *Motor4 = AFMS.getMotor(4);


void setup() {
  Serial.begin(115200);
  Serial.println("Robot Start!");
  
  AFMS.begin();  // start motorshield object with the default frequency of 1.6KHz
    // turn on motor M3
  Motor3->setSpeed(255);    //Max motor speed is 255.
  Motor3->run(RELEASE);
    // turn on motor M4
  Motor4->setSpeed(255);
  Motor4->run(RELEASE);
}


void loop() {
  //FORWARD
  Motor3->setSpeed(255);
  Motor4->setSpeed(255);
  Motor3->run(FORWARD);
  Motor4->run(FORWARD);
  delay(5000);
  Motor3->run(RELEASE);
  Motor4->run(RELEASE);
  delay(2000);
  
  //REVERSE
  Motor3->setSpeed(127);
  Motor4->setSpeed(127);
  Motor3->run(BACKWARD);
  Motor4->run(BACKWARD);
  delay(5000);
  Motor3->run(RELEASE);
  Motor4->run(RELEASE);
}
