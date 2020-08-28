/*
 * This program accepts input from a game controller to control a robot.
 * 
 * HARDWARE:
 * Arduino Uno (ATMega328P)
 * Sony PS2 Wireless Force 2 Controller
 * Cytron PS2 Shield: http://www.cytron.com.my/p-shield-ps2
 * SeeedStudio Motorshield v2.0 https://wiki.seeedstudio.com/Motor_Shield_V2.0/
 * Drive Motors: Vex 393 2-Wire Motors
 * 
 * CONTROL FEATURES:
 * 
*/

#include <SoftwareSerial.h>
#include "Cytron_PS2Shield.h"
#include "MotorDriver.h"

#define DEBUG_DRIVE

 
//Create PS2Shield object
Cytron_PS2Shield ps2(2, 3); // SoftwareSerial: assign Rx and Tx pin
//Cytron_PS2Shield ps2; // Alternatively, use this call to create object without paramaters to use the default hardware serial pins 0(RX) and 1(TX)
// Create the motor shield object
MotorDriver motor;

int joystick_left = 0;       //left joystick position in Y axis (FORWARD and BACKWARD)
int joystick_right = 0;       //left joystick position in X axis (LEFT and RIGHT)
float drive_left = 0.0;     //base speed component in the FORWARD/BACKWARD direction - derived from "joystick_left"
float drive_right = 0.0;      //speed adjustment value for left and right wheels while turning


void setup()
{
  ps2.begin(115200);          //Start remote control shield and set baud rate (baudrate must be the same as the jumper setting at PS2 shield)
  motor.begin();               //Start motorshield object
  delay(5000);
  motor.stop(0);
  motor.stop(1);
  Serial.begin(115200);       //Set baudrate for serial monitor
  Serial.println("Robot Start!");
}

void loop()
{ 
  /****************************
      JOYSTICKS
  ****************************/
  //LEFT joystick position
  joystick_left = 128 - ps2.readButton(PS2_JOYSTICK_LEFT_Y_AXIS);  //Get joystick difference from center position (FORWARD is positive)
  if (joystick_left > 10 || joystick_left < -10) {       //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD.
    drive_left = map(joystick_left, 0, 128, 0, 100);    //Map values to get full power delivery using only half of joystick travel (center to extremity)
    motor.speed(0, drive_left);
  }
  else {
    drive_left = 0;
    motor.brake(0);
    motor.stop(0);
  }

  //RIGHT joystick position
  joystick_right = 128 - ps2.readButton(PS2_JOYSTICK_RIGHT_Y_AXIS);  //Get joystick difference from center position (RIGHT is positive)
  if (joystick_right > 10 || joystick_right < -10) {       //Create "dead zone" for when joystick is centered (with independent adjustment values for FORWARD and BACKWARD.
    drive_right = map(joystick_right, 0, 128, 0, 100);    //Map values to get full power delivery using only half of joystick travel (center to extremity)
    motor.speed(1, drive_right);
  }
  else {
    drive_right = 0;
    motor.brake(1);
    motor.stop(1);
  }

  #ifdef DEBUG_DRIVE
  Serial.print ("joystick left: ");
  Serial.print (joystick_left);
  Serial.print ("\t");
  Serial.print ("drive left: ");
  Serial.print (drive_left);
  Serial.print ("\t");
  Serial.print ("joystick right: ");
  Serial.print (joystick_right);
  Serial.print ("\t");
  Serial.print ("drive right: ");
  Serial.println (drive_right);
  #endif
  
//  delay(50);    //Master delay between cycles
}
