/*
 * This program uses an Adafruit Motor Shield V1 to control a single 28BYJ-48 Stepper Motor via button press.
 * 
 * OPERATION:
 * a) Press button connected to A0 to repeatedly rotate the stepper CCW while the button is held down.
 * b) Press button connected to A5 to repeatedly rotate the stepper CW while the button is held down.
 * c) Release the button and the motor will stop after its current travel motion. (Stepper movement is a BLOCKING operation - meaning that the program will not prgress until the stepper mvement is complete.)
 * d) Pressing both buttons simultaneously will stop the motor (after its current travel motion) and reset the state of each button.
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

//REMEMBER TO SET THE CORRECT MOTOR PORT YOU WISH TO CONTROL:
AF_Stepper motor(32, 1);    //MOTOR PORT: For motor port M1 and M2 set the second parameter to 1. For motor ports M3 and M4, set the second parameter to 2.
                            //PARAMETER FORMAT: (STEPPER_STEPS, MOTOR_SHIELD_PORT). Set stepper steps to 360 divided by the step angle for the stepper. FOR EXAMPLE: 360 / 11.25 = 32

int port_1 = A0;    //Set button pins
int port_2 = A5;
int buttonState_1 = 0;  //Set default button states
int buttonState_2 = 0;
int motorTravel = 10;   //Amount of travel the stepper should rotate



void setup() {
  pinMode(port_1, INPUT_PULLUP); //Butons connect to GND upon press, so set default to be pulled up.
  pinMode(port_2, INPUT_PULLUP); 

  motor.setSpeed(1000);  // Use to set the stepper rotor RPM (1000 is the max)
  motor.step(100, FORWARD, SINGLE); //(I assume this statement is to initializes the motor)
  motor.release();

  Serial.begin(9600);   // set up Serial library at 9600 bps
  Serial.println("Stepper Motor Button Test!");

  delay(1000);
}

void loop() {
  if(digitalRead(port_1) == LOW && digitalRead(port_2) == HIGH) { //Check for button press on A0
    if(buttonState_1 == 0) {
      buttonState_1 = 1;    //Set buttonstate to "pressed"
      Serial.println("Motor FORWARD!");
    }
    motor.step(motorTravel, FORWARD, DOUBLE);
  }
  else if(buttonState_1 == 1 && digitalRead(port_1) == HIGH && digitalRead(port_2) == HIGH) { //Check for button release
    buttonState_1 = 0;
    motor.release();    //ALWAYS release motor when not in use to prevent overheating from constant current flow to "hold" stepper position
    Serial.println("Motor STOPPED!");
  }
  else if(digitalRead(port_1) == HIGH && digitalRead(port_2) == LOW) { //Check for button press on A5
    if(buttonState_2 == 0) {
      buttonState_2 = 1;    //Set buttonstate to "pressed"
      Serial.println("Motor BACKWARD!");
    }
    motor.step(motorTravel, BACKWARD, DOUBLE);
  }
  else if(buttonState_2 == 1 && digitalRead(port_1) == HIGH && digitalRead(port_2) == HIGH) { //Check for button release
    buttonState_2 = 0;
    motor.release();    //ALWAYS release motor when not in use to prevent overheating from constant current flow to "hold" stepper position
    Serial.println("Motor STOPPED!");
  }
  else if(digitalRead(port_1) == LOW && digitalRead(port_2) == LOW) {   //Stop motor if both burrons are pressed
    if(buttonState_1 == 1 || buttonState_2 == 1) {    //Check "pressed" flags to only print to the serial port the first time
      Serial.println("Motor STOPPED!");
    }
    buttonState_1 = 0;    //Reset button state flags
    buttonState_2 = 0;
    motor.release();    //ALWAYS release motor when not in use to prevent overheating from constant current flow to "hold" stepper position
  }
//  delay(100);   //Uncomment delay if you want each motor movement to feel discrete with a slight pause between each.
}
