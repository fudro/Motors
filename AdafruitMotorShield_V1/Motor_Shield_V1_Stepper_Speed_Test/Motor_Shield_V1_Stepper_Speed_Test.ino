/*
 * This program uses an Adafruit Motor Shield V1 to control a 28BYJ-48 Stepper Motor.
 * 
 * OPERATION:
 * 1) Pressing the button connected to analog pin A0 will increase the speed of the stepper with each press.
 * 2) Pressing the button connected to analog pin A5 will reset the speed to the default value.
 * 
 * STEPPER MOTOR NOTES
 * The 28BYJ-48 has a step angle of 11.25 degrees, 32 steps per revolution of the motor rotor.
 * The gear train has a ratio of 64:1 for a total of 2048 steps per revolution of the motor axle.
 * 
 * STEPPER MOTOR PORT WIRING SETUP:
 * Motorshield port color coded wiring from edge to edge: 
 *  Blue - outer screw terminal on the RIGHT SIDE when viewing the open face of the port
 *  Yellow - screw terminal adjacent to the BLUE screw terminal
 *  Red - middle screw terminal (stepper center tap)
 *  Orange - screw terminal adjacent to PINK screw terminal
 *  Pink - outer screw terminal on the LEFT SIDE when facing the open face of the port
 *  
 *  REFERENCE:
 *  WIRING DIAGRAM AND GENERAL INFO: https://www.instructables.com/28BYJ-48-Stepper-Motor-Arduino-L293D-Motor-Shield-/
 *  Stepper 28BYJ-48 General Info: https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/#:~:text=According%20to%20the%20data%20sheet,*64)%20steps%20per%20revolution.
 *  ADAFRUIT PAGE FOR THE MOTOR SHIELD V1: https://learn.adafruit.com/adafruit-motor-shield/using-stepper-motors
 */

#include <AFMotor.h>

//REMEMBER TO SET THE CORRECT MOTOR PORT:
AF_Stepper motor(32, 1);    //MOTOR PORT: For motor port M1 and M2 set the second parameter to 1. For motor ports M3 and M4, set the second parameter to 2.
                            //PARAMETER FORMAT: (STEPPER_STEPS, MOTOR_SHIELD_PORT). Set stepper steps to 360 divided by the step angle for the stepper. FOR EXAMPLE: 360 / 11.25 = 32

int port_1 = A0;
int port_2 = A5;
int motorTravel = 500;   //Amount of travel the stepper should rotate
int motorSpeed = 100;     //1000 seems to be the maximum speed without too much vibration and no skipping under reasonable load.


void setup() {
  pinMode(port_1, INPUT_PULLUP); //Butons connect to GND upon press, so set default to be pulled up.
  pinMode(port_2, INPUT_PULLUP); 

  motor.setSpeed(motorSpeed);  // Use to set the stepper RPM
  motor.step(100, FORWARD, SINGLE); //(I assume this initializes the motor)
  motor.release();

  Serial.begin(9600);   // set up Serial library at 9600 bps
  Serial.println("Stepper Motor Speed Test!");
  Serial.print("Motor Speeed: ");
  Serial.println(motorSpeed);
  Serial.println();

  delay(1000);
}

void loop() {
  if(digitalRead(port_1) == LOW && digitalRead(port_2) == HIGH) { //Check for button press on sensor pin assigned to the visor
    motorSpeed += 100;
    motor.setSpeed(motorSpeed);  // Use to set the stepper RPM
    Serial.print("Motor Speeed: ");
    Serial.println(motorSpeed);
    motor.step(motorTravel, FORWARD, DOUBLE);
    motor.release();
  }
  else if(digitalRead(port_1) == HIGH && digitalRead(port_2) == LOW) {
    Serial.println("Motor Speed RESET!");
    motorSpeed = 100;
    motor.setSpeed(motorSpeed);  // Use to set the stepper RPM
    Serial.print("Motor Speeed: ");
    Serial.println(motorSpeed);
    motor.step(motorTravel, FORWARD, DOUBLE);
    motor.release();
  }
  else if(digitalRead(port_1) == LOW && digitalRead(port_2) == LOW) {
    Serial.println("Motor STOPPED!");
    motorSpeed = 0;
    motor.setSpeed(motorSpeed);  // Use to set the stepper RPM
    Serial.print("Motor Speeed: ");
    Serial.println(motorSpeed);
    motor.release();
  }
}
