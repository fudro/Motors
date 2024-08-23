/*
 * This sketch is a test program for the Adafruit Motor Shield V1 and a 28BYJ-48 Stepper Motor.
 * The 28BYJ-48 has a step angle of 11.25 degrees, 32 steps per revolution of the motor rotor.
 * The gear train has a ratio of 64:1 for a total of 2048 steps per revolution of the motor axle.
 * MOTOR PORT WIRING SETUP:
 * Motor port wires from edge to edge: 
 *  Blue - outer screw terminal
 *  Yellow - second from edge screw terminal
 *  Red - Power center tap
 *  Orange - second from edge screw terminal
 *  Pink - outer screw terminal
 *  
 *  REFERENCE:
 *  WIRING DIAGRAM AND GENERAL INFO: https://www.instructables.com/28BYJ-48-Stepper-Motor-Arduino-L293D-Motor-Shield-/
 *  Stepper 28BYJ-48 General Info: https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/#:~:text=According%20to%20the%20data%20sheet,*64)%20steps%20per%20revolution.
 *  ADAFRUIT PAGE FOR THE MOTOR SHIELD V1: https://learn.adafruit.com/adafruit-motor-shield/using-stepper-motors
 */

#include <AFMotor.h>
                                //MOTOR PORT: For motor port M1 and M2 set the second parameter to 1. For motor ports M3 and M4, set the second parameter to 2.
                                //PARAMETER FORMAT: (STEPPER_STEPS, MOTOR_SHIELD_PORT). Set stepper steps to 360 divided by the step angle for the stepper. FOR EXAMPLE: 360 / 11.25 = 32
AF_Stepper motor_visor(32, 2);  
AF_Stepper motor_mic(32, 1);
int visorPin = A0;
int visorbuttonActive = 0; 
int visorPosition = 1;   //Default position for visor is UP = 1
double visorTravel = 4500;   //Amount of travel the stepper should rotate

int micPin = A5;
int micbuttonActive = 0; 
int micPosition = 1;   //Default position for visor is UP = 1
int micTravel = 400;   //Amount of travel the stepper should rotate

void setup() {
  pinMode(visorPin, INPUT_PULLUP); //Butons connect to GND upon press, so set default to be pulled up.
  pinMode(micPin, INPUT_PULLUP); 
  Serial.begin(9600);   // set up Serial library at 9600 bps
  Serial.println("Stepper Button Test!");

  motor_visor.setSpeed(1000);  // Use to set the stepper RPM
  motor_visor.step(100, FORWARD, SINGLE); //(I assume this initializes the motor)
  motor_visor.release();

  motor_mic.setSpeed(1000);  // Max value is 10000. Higher values do not result in a faster rotation.
  motor_mic.step(100, FORWARD, SINGLE); 
  motor_mic.release();
  delay(1000);
}

void loop() {
  if(visorbuttonActive == 0 && digitalRead(visorPin) == LOW) { //Check for button press on sensor pin assigned to the visor
    visorbuttonActive = 1;
    if(visorPosition == 1) {
      if(micPosition == 1) {
        micbuttonActive = 1;
        Serial.println("Mic Moving Down!");
        motor_mic.step(micTravel, BACKWARD, DOUBLE);
        micPosition = 0; //Set mic state to DOWN 
      }
      Serial.println("Visor Moving Down!");
      motor_visor.step(visorTravel, FORWARD, DOUBLE);
      visorPosition = 0; //Set visor state to DOWN 
    }
    else if(visorPosition == 0) {
      Serial.println("Visor Moving Up!");
//      motor_visor.step(visorTravel, BACKWARD, DOUBLE);
      
      motor_visor.step(1800, BACKWARD, DOUBLE);
      motor_visor.step(700, BACKWARD, MICROSTEP);  //Use MicroStep for stronger torque at the hardest part of the lifting motion
      motor_visor.step(2000, BACKWARD, DOUBLE);
      
      visorPosition = 1; //Set visor state to UP 
      if(micPosition == 0) {
        micbuttonActive = 1;
        Serial.println("Mic Moving Up!");
        motor_mic.step(micTravel, FORWARD, DOUBLE);
        micPosition = 1; //Set mic state to UP 
        micbuttonActive = 0;
      }
    }
    visorbuttonActive = 0;  //Reset button value for additional button presses
    Serial.println("Visor Button Ready!");
  }

  if(micbuttonActive == 0 && visorPosition == 1 && digitalRead(micPin) == LOW) { //Check for button press on sensor pin assigned to the visor
    micbuttonActive = 1;
    if(micPosition == 1) {
      Serial.println("Mic Moving Down!");
      motor_mic.step(micTravel, BACKWARD, DOUBLE);
      micPosition = 0; //Set mic state to DOWN 
    }
    else if(micPosition == 0) {
      Serial.println("Mic Moving Up!");
      motor_mic.step(micTravel, FORWARD, DOUBLE);
      micPosition = 1; //Set mic state to UP 
    }
    micbuttonActive = 0;  //Reset button value for additional button presses
    Serial.println("Mic Button Ready!");
  }
}
