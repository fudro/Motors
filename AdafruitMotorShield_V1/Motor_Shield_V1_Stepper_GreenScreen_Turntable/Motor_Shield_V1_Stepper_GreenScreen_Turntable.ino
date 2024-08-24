/*
 * This program uses an Adafruit Motor Shield V1 to control a turntable powered by a 28BYJ-48 Stepper Motor.
 * 
 * OPERATION:
 * Buttons connected to analog pins A0 and A5 control the rotation and direction of the turntable.
 * The turntable has two motors controlling two seprate actions: a) Turntable Rotation, b) Camera Boom.
 * a) Press each button once to rotate its associrated motor in a specific direction.
 * b) Press the same button again to change the direction of the associated motor
 * Each motor will move a finite amount for each button press and release.
 * Press and hold the button to make the desired motor rotate continuously in the current direction.
 * The button must be released to reverse the direction of movement for each motor.
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
 *  ADAFRUIT PAGE FOR THE MOTOR SHIELD V1: https://learn.adafruit.com/adafruit-motor-shield/using-stepper-motors
 */

#include <AFMotor.h>

//REMEMBER TO SET THE CORRECT MOTOR PORTS:
AF_Stepper motor_table(32, 1);  //PARAMETER FORMAT: (STEPPER_STEPS, MOTOR_SHIELD_PORT). Set stepper steps to 360 divided by the step angle for the stepper. FOR EXAMPLE: 360 / 5.625 = 64
AF_Stepper motor_camera(32, 2);  //MOTOR PORT: For motor port M1 and M2 set the second parameter to 1. For motor ports M3 and M4, set the second parameter to 2.
int port_1 = A0;
int port_2 = A5;
int buttonState_1 = 0;
int buttonState_2 = 0;
int motorDir_1 = 0; //0 = CW, 1 = CCW
int motorDir_2 = 0;
int motorTravel = 100;   //Amount of travel the stepper should rotate

void setup() {
  pinMode(port_1, INPUT_PULLUP); //Butons connect to GND upon press, so set default to be pulled up.
  pinMode(port_2, INPUT_PULLUP); 

  motor_table.setSpeed(1000);  // Use to set the stepper RPM
  motor_table.step(100, FORWARD, SINGLE); //(I assume this initializes the motor)
  motor_table.release();
  motor_camera.setSpeed(1000);  // Use to set the stepper RPM
  motor_camera.step(100, FORWARD, SINGLE); //(I assume this initializes the motor)
  motor_camera.release();

  Serial.begin(9600);   // set up Serial library at 9600 bps
  Serial.println("Greenscree Turntable Test!");

  delay(1000);
}

void loop() {
  if(digitalRead(port_1) == LOW && digitalRead(port_2) == HIGH) { //Check for button press on A0
    if(buttonState_1 == 0) {
      buttonState_1 = 1;
    }
    if(motorDir_1 == 0) {
      motor_table.step(motorTravel, FORWARD, DOUBLE);
      Serial.println("Table Motor FORWARD!");
    }
    else if(motorDir_1 == 1) {
      motor_table.step(motorTravel, BACKWARD, DOUBLE);
      Serial.println("Table Motor BACKWARD!");
    }
  }
  else if(buttonState_1 == 1 && digitalRead(port_1) == HIGH && digitalRead(port_2) == HIGH) {   //Check for button release
    buttonState_1 = 0;
    motorDir_1 = -1 * (motorDir_1 - 1);
    motor_table.release();
    Serial.println("Table Motor STOPPED!");
  }
  else if(digitalRead(port_1) == HIGH && digitalRead(port_2) == LOW) { //Check for button press on A5
    if(buttonState_2 == 0) {
      buttonState_2 = 1;
    }
    if(motorDir_2 == 0) {
      motor_camera.step(motorTravel, FORWARD, DOUBLE);
      Serial.println("Camnera Motor FORWARD!");
    }
    else if(motorDir_2 == 1) {
      motor_camera.step(motorTravel, BACKWARD, DOUBLE);
      Serial.println("Camera Motor BACKWARD!");
    }
  }
  else if(buttonState_2 == 1 && digitalRead(port_1) == HIGH && digitalRead(port_2) == HIGH) {   //Check for button release
    buttonState_2 = 0;
    motorDir_2 = -1 * (motorDir_2 - 1);   //Toggle motor direction.
    motor_camera.release();
    Serial.println("Camera Motor STOPPED!");
  }
}
