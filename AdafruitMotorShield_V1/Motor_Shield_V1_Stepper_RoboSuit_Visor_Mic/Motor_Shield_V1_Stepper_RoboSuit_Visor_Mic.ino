/*
 * This program uses an Adafruit Motor Shield V1 and a 28BYJ-48 Stepper Motor.
 * Buttons connected to A0 and A5 provide INDEPENDENT motorized control of the Visor and Mic.
 * NOTE:  The program does not explicitly account for simultaneious presses of both buttons.
 *        However, the normal BLOCKING behavior of the stepper movement automatically prevents operation of both motors at the same time.
 * 
 * OPERATION:
 * a) Press button connected to A0 to activate the visor motor. Motor direction will toggle for each press.
 * b) Press button connected to A5 to activate the mic motor. Motor direction will toggle for each press.
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
  Serial.println("Robotsuit Visor and Mic Test!");

  motor_visor.setSpeed(1000);  // Use to set the stepper internal rotor RPM (1000 provides best performance)
  motor_visor.step(100, FORWARD, SINGLE); //(I assume this initializes the motor)
  motor_visor.release();

  motor_mic.setSpeed(1000);  // Max value is 1000 for best combination of speed and non-skipping under load
  motor_mic.step(100, FORWARD, SINGLE); 
  motor_mic.release();
  delay(1000);
}

void loop() {
  if(visorbuttonActive == 0 && digitalRead(visorPin) == LOW) { //Check for button press on sensor pin assigned to the visor
    visorbuttonActive = 1;
    if(visorPosition == 1) {
      Serial.println("Visor Moving Down!");
      motor_visor.step(visorTravel, FORWARD, DOUBLE);
      visorPosition = 0; //Set visor state to DOWN 
    }
    else if(visorPosition == 0) {
      Serial.println("Visor Moving Up!");
      motor_visor.step(visorTravel, BACKWARD, DOUBLE);      
      visorPosition = 1; //Set visor state to UP 
    }
    visorbuttonActive = 0;  //Reset button value for additional button presses
    motor_visor.release();
    Serial.println("Visor Button Ready!");
  }

  if(micbuttonActive == 0 && digitalRead(micPin) == LOW) { //Check for button press on sensor pin assigned to the visor
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
    motor_mic.release();
    Serial.println("Mic Button Ready!");
  }
}
