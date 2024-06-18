/*
 * This sketch is a test program for the Adafruit Motor Shield V1 and a 28BYJ-48 Stepper Motor.
 * When tested, the stepper motor has the following rotational values:
 * MICROSTEP: 45 Degrees = Step Value of 530
 * 
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
 *  ADAFRUIT PAGE FOR THE MOTOR SHIELD V1: https://learn.adafruit.com/adafruit-motor-shield/using-stepper-motors
 */

#include <AFMotor.h>


AF_Stepper motor(64, 2);  //PARAMETER FORMAT: (STEPPER_STEPS, MOTOR_SHIELD_PORT). Set stepper steps to 360 divided by the step angle for the stepper. FOR EXAMPLE: 360 / 5.625 = 64
                          //MOTOR PORT: For motor port M1 and M2 set the second parameter to 1. For motor ports M3 and M4, set the second parameter to 2.
int buttonActive = 0; 
int visorPosition = 0;   //Default position for visor is UP = 0
int visorTravel = 10;   //Amount of travel the stepper should rotate

void setup() {
  pinMode(A5, INPUT_PULLUP); 
  Serial.begin(9600);   // set up Serial library at 9600 bps
  Serial.println("Stepper Button Test!");
  /*Set stepper speed:
   * Camera Arm: 30
   * Turntable: 1000
   */
  motor.setSpeed(1000);  // Max value is 10000. Higher values do not result in a faster rotation.
}

void loop() {
  if(buttonActive == 0 && digitalRead(A5) == LOW) { //Check for button press on analog pin A0
    buttonActive = 1;
    if(visorPosition == 1) {
      Serial.println("Camera Moving Down!");
      motor.step(visorTravel, FORWARD, MICROSTEP);
//      visorPosition = 0; //Set visor state to DOWN 
    }
    else if(visorPosition == 0) {
      Serial.println("Camera Moving Up!");
      motor.step(visorTravel, BACKWARD, MICROSTEP);
//      visorPosition = 1; //Set visor state to UP 
    }
    buttonActive = 0;  //Reset button value for additional button presses
    Serial.println("Button Ready!");
  }
}
