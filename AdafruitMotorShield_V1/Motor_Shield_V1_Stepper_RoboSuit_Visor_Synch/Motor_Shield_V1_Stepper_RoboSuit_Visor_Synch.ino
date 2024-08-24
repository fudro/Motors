/*
 * This program uses an Adafruit Motor Shield V1 and a 28BYJ-48 Stepper Motor.
 * Buttons connected to pins A0 and A5 provide SYNCHRONIZED Robosuit control of both the visor and the mic.
 * 
 * OPERATION:
 * a) Press button connected to A0 to automatically activate the visor and mic with SYNCHRONIZED movements. Visor and Mic direction of motino will toggle with each press.
 * b) Press button connected to A5 to operate the mic boom INDEPENDENTLY of the visor. (The program automatically checks that the visor positon will not obstruct mic boom movement.)
 * NOTE: Pressing and holding either button will not cause repeated activation. Each button MUST be released before it can activated again.
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

AF_Stepper motor_visor(32, 2);  //MOTOR PORT: For motor port M1 and M2 set the second parameter to 1. For motor ports M3 and M4, set the second parameter to 2.
AF_Stepper motor_mic(32, 1);    //PARAMETER FORMAT: (STEPPER_STEPS, MOTOR_SHIELD_PORT). Set stepper steps to 360 divided by the step angle for the stepper. FOR EXAMPLE: 360 / 11.25 = 32
int visorPin = A0;
int visorbuttonActive = 0; 
int visorPosition = 1;   //Default position for visor is UP = 1
double visorTravel = 4500;   //Amount of travel the stepper should rotate

int micPin = A5;
int micbuttonActive = 0; 
int micPosition = 1;   //Default position for visor is UP = 1
int micTravel = 400;   //Amount of travel the stepper should rotate

void setup() {
  pinMode(visorPin, INPUT_PULLUP); //Buttons connect to GND upon press, so set default to be pulled up.
  pinMode(micPin, INPUT_PULLUP); 
  Serial.begin(9600);   // set up Serial library at 9600 bps
  Serial.println("Robosuit Synchonized Visor and Mic Test!");

  motor_visor.setSpeed(1000);  // Use to set the stepper internal rotor RPM (1000 is the max value for best combination of speed and non-skipping under reasonable load.)
  motor_visor.step(100, FORWARD, SINGLE); //(I assume this initializes the motor)
  motor_visor.release();

  motor_mic.setSpeed(1000);
  motor_mic.step(100, FORWARD, SINGLE); 
  motor_mic.release();
  delay(1000);
}

void loop() {
  if(visorbuttonActive == 0 && digitalRead(visorPin) == LOW) { //Check for button press on sensor pin assigned to the visor
    visorbuttonActive = 1;
    if(visorPosition == 1) {    //If visor is in UP position...
      if(micPosition == 1) {    //Check if mic is in the UP position. (This is important because once the visor is down, it impedes movement of the mic boom.)
        micbuttonActive = 1;    //If so, set the mic button state to active and automatically move the mic to the downward position.
        Serial.println("Mic Moving Down!");
        motor_mic.step(micTravel, BACKWARD, DOUBLE);
        micPosition = 0; //Set mic state to DOWN 
      }
      Serial.println("Visor Moving Down!");
      motor_visor.step(visorTravel, FORWARD, DOUBLE);   //Move the visor after the mic has finished moving.
      motor_visor.release();
      visorPosition = 0; //Set visor state to DOWN 
    }
    else if(visorPosition == 0) {   //If visor is in DOWN position... go ahead and move it upward.
      Serial.println("Visor Moving Up!");

//      motor_visor.step(visorTravel, BACKWARD, DOUBLE);    //NEED TO TEST IF THE PHYSICAL REBUILD WILL ALLOW US TO USE THIS SINGLE COMMAND
      motor_visor.step(1800, BACKWARD, DOUBLE);    //USE "MULTI-MODE" COMMAND TO PREVENT MOTOR SKIPPING
      motor_visor.step(700, BACKWARD, MICROSTEP);  //Use MicroStep for stronger torque at the hardest part of the lifting motion
      motor_visor.step(2000, BACKWARD, DOUBLE);

      motor_visor.release();
      visorPosition = 1; //Set visor state to UP 
      
      if(micPosition == 0) {    //If mic position id DOWN, automatically raise it after the visor upward movement is complete.
        micbuttonActive = 1;
        Serial.println("Mic Moving Up!");
        motor_mic.step(micTravel, FORWARD, DOUBLE);
        motor_mic.release();
        micPosition = 1; //Set mic state to UP 
        micbuttonActive = 0;
      }
    }
  }
  else if(visorbuttonActive == 1 && digitalRead(visorPin) == HIGH) {  //Check for visor button release
    visorbuttonActive = 0;  //Reset button value for additional button presses
    Serial.println("Visor Button Ready!");
  }
  else if(micbuttonActive == 0 && visorPosition == 1 && digitalRead(micPin) == LOW) { //Check if the visor is UP and the mic button is pressed (This means the mic boom can move independent of the visor)
    micbuttonActive = 1;
    if(micPosition == 1) {
      Serial.println("Mic Moving Down!");
      motor_mic.step(micTravel, BACKWARD, DOUBLE);
      motor_mic.release();
      micPosition = 0; //Set mic state to DOWN 
    }
    else if(micPosition == 0) {
      Serial.println("Mic Moving Up!");
      motor_mic.step(micTravel, FORWARD, DOUBLE);
      motor_mic.release();
      micPosition = 1; //Set mic state to UP 
    }
  }
  else if(micbuttonActive == 1 && visorPosition == 1 && digitalRead(micPin) == HIGH) { //Check for mic button release
    micbuttonActive = 0;  //Reset button value for additional button presses
    Serial.println("Mic Button Ready!");
  }
}
