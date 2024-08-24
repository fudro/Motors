/*
 * This sprograms tests basic motorized control of the Robosuit Visor.
 * 
 * OPERATION:
 * The Visor uses an Adafruit Motor Shield V1 and a 28BYJ-48 Stepper Motor.
 * a) Press the button connected to A0 to activate the visor.
 * b) The visor will toggle its movement direction with each press of the button.
 * c) Holding the button down will cause the visor to "ping-pong" back and forth until the button is released.
 * NOTE: This test program has not been adjusted to provide the proper amount of visor travel required when mounted on the helment.
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
 *  Stepper 28BYJ-48 GENERAL INFO:  https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/#:~:text=According%20to%20the%20data%20sheet,*64)%20steps%20per%20revolution.
 *                                  https://www.instructables.com/28BYJ-48-Stepper-Motor-Arduino-L293D-Motor-Shield-/
 *  ADAFRUIT PAGE FOR THE MOTOR SHIELD V1: https://learn.adafruit.com/adafruit-motor-shield/using-stepper-motors
 */

#include <AFMotor.h>


AF_Stepper motor(32, 1);  //PARAMETER FORMAT: (STEPPER_STEPS, MOTOR_SHIELD_PORT). Set stepper steps to 360 divided by the step angle for the stepper. FOR EXAMPLE: 360 / 5.625 = 64
                          //MOTOR PORT: For motor port M1 and M2 set the second parameter to 1. For motor ports M3 and M4, set the second parameter to 2.
int buttonActive = 0; 
int visorPosition = 1;   //Default position for visor is UP = 1
int visorTravel = 1000;   //Amount of travel the stepper should rotate

void setup() {
  pinMode(A0, INPUT_PULLUP);  //Buton connects to GND upon press, so set default to be pulled up.
  Serial.begin(9600);   // set up Serial library at 9600 bps
  Serial.println("Robosuit Visor Test!");

  motor.setSpeed(1000);  // Max value is 1000. Higher values do not result in a faster rotation.
  motor.step(100, FORWARD, SINGLE); //Initialize motor
  motor.release();
  delay(1000);
}

void loop() {
  if(buttonActive == 0 && digitalRead(A0) == LOW) { //Check for button press on analog pin A0
    buttonActive = 1;
    if(visorPosition == 1) {
      Serial.println("Visor Moving Down!");
      motor.step(visorTravel, FORWARD, MICROSTEP);
      visorPosition = 0; //Set visor state to DOWN 
    }
    else if(visorPosition == 0) {
      Serial.println("Visor Moving Up!");
      motor.step(visorTravel, BACKWARD, MICROSTEP);
      visorPosition = 1; //Set visor state to UP 
    }
    buttonActive = 0;  //Reset button value for additional button presses
    motor.release();
    Serial.println("Button Ready!");
  }
}
