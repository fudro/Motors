/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    MeMegaPiDCMotorTest.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2016/05/17
 * @brief   Description: this file is sample code for MegaPi DC motor device.
 *
 * Function List:
 *    1. void MeMegaPiDCMotorTest::run(int16_t speed)
 *    2. void MeMegaPiDCMotorTest::stop(void)
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2016/05/17    1.0.0          build the new
 * Anthony Fudd 2021/03/28    1.0.1          Added notes to program description.
 * </pre>
 * 
 * HARDWARE COMPATIBILITY:
 * MegaPi Microcontroller
 * Encoder/DC Motor Driver Board (both MegaPi and MegaPi Pro versions)
 * IMPORTANT:
 * There are 2 versions of the Encoder/DC motor driver board. Both versions will work with this program.
 * MegaPi Version has a larger chip on the board and is capable of 1A continuous and 2A peak.
 * MegaPi Pro Version has a smaller chip on the board and is capable of 3A continous and 5.5A peak.
 * 
 * NOTES:
 * This program works ONLY with MegaPi microcontoller to control up to 8 DC motors.
 * The MegaPi has 4 motor "ports". Each port accepts a detachable Encoder/DC motor driver board.
 * Each driver board is capable of driving two DC motors.
 * 
 */
#include "MeMegaPi.h"

MeMegaPiDCMotor armGrip(PORT1A); //Arm Gripper
MeMegaPiDCMotor armWrist(PORT1B); //Wrist
MeMegaPiDCMotor driveRight(PORT2A); //Drive Right, based on the sonar unit being the "front" of the robot
MeMegaPiDCMotor driveLeft(PORT2B); //Drive Left
MeMegaPiDCMotor tailGrip(PORT3A); //Tail Gripper
MeMegaPiDCMotor turnTable(PORT3B); //Turntable
MeMegaPiDCMotor elbow(PORT4A); //Elbow
MeMegaPiDCMotor shoulder(PORT4B); //Shoulder

//Direction Constants
const int OPEN = 0;
const int CLOSE = 1;
const int H = 0;    //HORIZONTAL
const int V = 1;    //VERTICAL
const int CW = 0;
const int CCW = 1;
const int FW = 1; //Forward
const int BW = 0; //Backward
//Pin Assignments
//TODO: Add Sonar Pins
//TODO: Add Lidar Pins
//TODO: Add IMU Pins
const int WRIST_HALL = 30;
const int WRIST_SWITCH = 28;
const int DRIVE_RIGHT_ENCODER = A11;
const int DRIVE_LEFT_ENCODER = A10;
const int ELBOW_POT = A9;
const int SHOULDER_POT = A8;
const int TURNTABLE_ENCODER = A7;
const int TURNTABLE_HALL = 26;
const int TURNTABLE_SWITCH = 24;
//Time Constants
const int ARMGRIPTIME = 1200; //Time for arm gripper at max speed 255 to go from fully open to fully closed and vice versa.
const int WRISTTIME = 1000; //Wrist rotation at max speed 255 for 1000 milliseconds is approximately 180 degres of rotation (or one-half turn)
const int TAILGRIPTIME = 400;  //Time for tail gripper at HALF SPEED (128) to go from fully open to fully closed and vice versa.
//Potentiometer Limits
const int ELBOW_MIN = 80; //Limit values for sensor
const int ELBOW_MAX = 700;
const int SHOULDER_MIN = 375;
const int SHOULDER_MAX = 600;
//Encoder Limits
const int TURNTABLE_ANALOG_MAX = 875;
const int TURNTABLE_ANALOG_MIN = 650;
const int DRIVE_LEFT_ANALOG_MAX = 700;
const int DRIVE_LEFT_ANALOG_MIN = 400;
const int DRIVE_RIGHT_ANALOG_MAX = 700;
const int DRIVE_RIGHT_ANALOG_MIN = 400;
//State Variables
int wristSwitch = 0;  //store state of wrist switch
int wristState = H;   //current state of wrist (H or V), horizontal direction as default
int turnTableEncoder = 1; //state of turntable encoder. Set initial stae to 1 since this pin is pulled high
int turnTableSwitch = 0;  //state of turntable switch, default state is 0 (LOW)
int turnTableHall = 1;    //state of turntable hall effect sensor, default state is HIGH, sensor pulls pin LOW when active (magnet present)
int driveLeftEncoder = 1; //last state of encoder high(1) or low(0), for comparison
int driveRightEncoder = 1;
//Tracking variables
int turnTableCount = 1;  //store value of turntable position based on encoder ticks. Default to 1 for easy math (15 ticks ~ 90 degrees)
int driveLeftCount = 0;  //store encoder tick value to track movement
int driveRightCount = 0;
int driveLeftAnalog = 0;  //last analog value from drive encoder, raw analog value is used to set current state of encoder
int driveRightAnalog = 0;
int turnTableAnalog = 0;  //last analog value of turntable encoder

//the runArray[] determines which motor functions are active (0 = inactive, 1 = active). Event if code is called by the main loop, the flag array will prevent code execution if motor is not set to "active" (1)
int runArray[] = {0,  //runArray[0]: armGripper
                  1,  //runArray[1]: wrist
                  0,  //runArray[2]: elbow
                  0,  //runArray[3]: shoulder
                  0,  //runArray[4]: turntable
                  0,  //runArray[5]: tailGripper
                  0,  //runArray[6]: drive
                  0   //runArray[7]: NOT USED
};

/***********************************
 * FUNCTION PROTOTYPES
***********************************/
void armGripper(int gripState, int gripTime = ARMGRIPTIME); //gripState is OPEN or CLOSE
void wristRotate (int targetState, int wristDirection = CW, float wristRevolution = 0.0, int wristSpeed = 255);  //targetState is (V)ertical or (H)orizontal, wristRevolution is the number of full revolutions to be perfomed.
void elbowMove(int elbowPosition = 500, int elbowSpeed = 65);  //approximate center position and preferred default speed.
void shoulderMove(int shoulderPosition = 575, int shoulderSpeed = 127); //approximate center position and preferred default speed.
void turnTableMove(int turnDegrees = 0, int turnDirection = CW, int turnSpeed = 65);    //turnDegrees is the degrees of angular rotation from the current position
void tailGripper(int gripState, int gripTime = TAILGRIPTIME); //gripState is OPEN or CLOSE
void driveMove(int driveDistance = 20, int driveDirection = FW, int driveSpeed = 127);

void setup()
{
  //Setup sensor pins
  pinMode(WRIST_HALL, INPUT_PULLUP);  //wrist hall effect, set as pullup since sensor goes LOW when activated
  pinMode(WRIST_SWITCH, INPUT); //wrist switch
  pinMode(DRIVE_LEFT_ENCODER, INPUT_PULLUP);
  pinMode(DRIVE_RIGHT_ENCODER, INPUT_PULLUP);
  pinMode(ELBOW_POT, INPUT_PULLUP); //elbow potentiomter
  pinMode(SHOULDER_POT, INPUT_PULLUP); //shoulder potentiometer
  pinMode(TURNTABLE_SWITCH, INPUT); //turntable switch
  pinMode(TURNTABLE_HALL, INPUT_PULLUP); //turntable hall effect, set as pullup since sensor goes LOW when activated
  pinMode(TURNTABLE_ENCODER, INPUT_PULLUP); //turntable encoder
  Serial.begin(115200);
  Serial.println("HomeBot Sensor Ranges Test!");
  delay(1000);
  Serial.println("StartingTest in 3 seconds...");
  delay(3000);
}

void loop()
{
//TODO: Add IMU code
//TODO: Add Sonar code
//TODO: Add Lidar code

//  armGripper(CLOSE);      //GOOD
//  wristRotate(H);         //GOOD
//  shoulderMove();         //GOOD
//  turnTableReset();       //GOOD
//  elbowMove();            //GOOD
//  turnTableMove(180, CW); //GOOD
//  tailGripper(CLOSE);     //GOOD
//  driveMove(10, FW, 65);  //GOOD
  
  delay(10);
}

void turnTableReset() {
  if(runArray[4] == 1) {    //Check flag to prevent unnecessary re-triggering of the function
    shoulderMove(400);      //lift shoulder   
    elbowMove();            //center elbow 
    Serial.print("\n");
    Serial.println("Turntable Reset!");
    Serial.print("\n");
    int turnSpeed = 65;   //Set to one quarter maximum speed
    //Default direction is CCW which is what we want so no change necessary.
    turnTable.run(turnSpeed);
    while (turnTableSwitch < 1) {   //while turntable switch is not activated.
      turnTableSwitch = digitalRead(TURNTABLE_SWITCH);
      if(turnTableSwitch > 0) { //if turntable reaches physical limit (indicated by switch HIGH), then stop motor.
        //Brake motor once limit switch is activated
        turnTable.stop();
        turnTable.run(-turnSpeed); //Reverse motor direction to brake briefly
        delay(30);
        turnTable.run(0);    //Release motor by setting speed to zero
        turnTable.stop();
      }
      Serial.print("Turntable Switch: ");
      Serial.print(turnTableSwitch);
      Serial.print("\n");
      
      delay (10);
    }
    //Brake motor once encoder tick count is achaieved
    turnTable.stop();
    turnTable.run(-turnSpeed); //Reverse motor direction to brake briefly
    delay(30);
    turnTable.run(0);    //Release motor by setting speed to zero
    turnTable.stop();
    delay(1000);

    //Rotate CW until both turntable switch and hall effect sensor are LOW
    //Set direction for CCW
    turnSpeed = turnSpeed * -1;
    Serial.print("\n");
    Serial.println("Turntable to: Zero Position");
    Serial.print("\n");
    turnTable.run(turnSpeed);
    while (turnTableHall > 0) {   //while turntable switch is not activated.
      turnTableHall = digitalRead(TURNTABLE_HALL);
      
      Serial.print("Turntable Hall: ");
      Serial.print(turnTableHall);
      Serial.print("\n");
      
      delay (10);
    }
    delay(100); //pause briefly to allow arm to truly center. extra time is required due to detection angle of sensor and arm turn speed.
    //Brake motor once limit switch is activated
    turnTable.stop();
    turnTable.run(-turnSpeed); //Reverse motor direction to brake briefly
    delay(30);
    turnTable.run(0);    //Release motor by setting speed to zero
    turnTable.stop();

    runArray[3] = 1;  //reset shoulder motor flag to active so the shoulderMove function can run again
    shoulderMove();
    Serial.print("\n");
    Serial.print("Turntable and Arm at Zero Position!");
    runArray[4] = 0;
  }
}

void driveMove(int driveDistance = 20, int driveDirection = FW, int driveSpeed = 127) {
  if(runArray[6] == 1) {    //Check flag to prevent unnecessary re-triggering of the function
    float tickTarget = 1.0; //number of encoder ticks required to perform the movement

    //convert driveDistance to encoder ticks (90 degrees is approximately 15 ticks)
    if(driveDistance > 0) { //check for valid input for travel 
      tickTarget = driveDistance;  //TODO: apply some function to convert driveDistance to ticks
      Serial.print("Motor Distance: ");
      Serial.println(driveDistance);
      Serial.print("TickTarget: ");
      Serial.println(tickTarget);
    }
    
    //Set sign of motor speed based on desired rotation direction
    if(driveDirection == BW) {  //Forward
      driveSpeed = driveSpeed * -1;
      Serial.println("Backward");
    }
    else {
      Serial.println("Forward");
    }

    driveRight.run(driveSpeed); //right wheel based on sonar unit being the "front" of the robot
    driveLeft.run(driveSpeed); //left wheel
    while (driveLeftCount < tickTarget && driveRightCount < tickTarget) {   //Wait for both encoders to reach the target value.
      driveLeftAnalog = analogRead(DRIVE_LEFT_ENCODER);
      driveRightAnalog = analogRead(DRIVE_RIGHT_ENCODER);
      if(driveLeftAnalog > DRIVE_LEFT_ANALOG_MAX && driveLeftEncoder == 0) { //if encoder value passes the threshold for HIGH, and the current state of the sensor is LOW, set the sensor to HIGH and increment the tick count.
        driveLeftEncoder = 1;
        driveLeftCount++;   //Increment encoder tick count each time th sensor reads HIGH
      }
      else if(driveLeftAnalog < DRIVE_LEFT_ANALOG_MIN && driveLeftEncoder == 1) {  //if encoder value goes below the threshold for LOW, and the current state of the sensor is HIGH, set the sensor to LOW and wait for next trigger.
        driveLeftEncoder = 0;
      }
      Serial.print("L: ");
      Serial.print("A ");
      Serial.print(driveLeftAnalog);
      if(driveLeftAnalog < 100) { //formatting trick to add a tab if the value has only 2 digits (this causes display readability issues)
        Serial.print("\t");
      }
      Serial.print("\t");
      Serial.print("E ");
      Serial.print(driveLeftEncoder);
      Serial.print("\t");
      Serial.print("C ");
      Serial.print(driveLeftCount);
      Serial.print("\t");
      Serial.print("\t");

      if(driveRightAnalog > DRIVE_RIGHT_ANALOG_MAX && driveRightEncoder == 0) { //if encoder value passes the threshold for HIGH, and the current state of the sensor is LOW, set the sensor state to HIGH and increment the tick count.
        driveRightEncoder = 1;
        driveRightCount++;
      }
      else if(driveRightAnalog < DRIVE_RIGHT_ANALOG_MIN && driveRightEncoder == 1) {  //if encoder value goes below the threshold for LOW, and the current state of the sensor is HIGH, set the sensor state to LOW and wait for next trigger.
        driveRightEncoder = 0;
      }
      Serial.print("R: ");
      Serial.print("A ");
      Serial.print(driveRightAnalog);
      if(driveRightAnalog < 100) {
        Serial.print("\t");
      }
      Serial.print("\t");
      Serial.print("E ");
      Serial.print(driveRightEncoder);
      Serial.print("\t");
      Serial.print("C ");
      Serial.println(driveRightCount);
      delay (10);
    }
    
    //Brake motors once tick target is reached
    driveRight.stop();
    driveRight.run(-driveSpeed); //Reverse motor direction to brake briefly
    delay(30);
    driveRight.run(0);    //Release motor by setting speed to zero
    driveRight.stop();

    driveLeft.stop();
    driveLeft.run(-driveSpeed); //Reverse motor direction to brake briefly
    delay(30);
    driveLeft.run(0);    //Release motor by setting speed to zero
    driveLeft.stop();
    runArray[6] = 0;
  }
}

void tailGripper(int gripState, int gripTime = TAILGRIPTIME) {
   if (runArray[5] == 1) {  //Check flag to prevent unnecessary re-triggering of the function
    runArray[5] = 0;
    int gripSpeed = 128;   // value: between -255 and 255. It is rarely necessary to change the gripper speed, so it is only a local variable
    Serial.println("Tail Gripper...");
    if (gripState == OPEN) {
      Serial.println("Tail Open:");
      Serial.print("Speed: ");
      Serial.println(gripSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      Serial.print("\n");
      tailGrip.run(gripSpeed); 
      delay(gripTime);
      
      //Brake motor
      tailGrip.stop();
      tailGrip.run(-gripSpeed); //Reverse motor direction to brake briefly
      delay(10);
      tailGrip.run(0);    //Release motor by setting speed to zero
      tailGrip.stop();
    }
    else if (gripState == CLOSE) {
      Serial.println("Tail Close:");
      Serial.print("Speed: ");
      Serial.println(-gripSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      Serial.print("\n");
      tailGrip.run(-gripSpeed); 
      delay(gripTime);
      
      //Brake motor
      tailGrip.stop();
      tailGrip.run(gripSpeed); //Reverse motor direction to brake briefly
      delay(10);
      tailGrip.run(0);    //Release motor by setting speed to zero
      tailGrip.stop();
    }
    delay(1000);
  }
}

void turnTableMove(int turnDegrees = 0, int turnDirection = CW, int turnSpeed = 65) {   //the turntable will not move if no argument is given.
  if(runArray[4] == 1) {    //Check flag to prevent unnecessary re-triggering of the function
    float tickTarget = 1.0;   //number of encoder ticks required to perform the movement
    float conversionRate = 14.0;

    Serial.print("\n");
    //Set sign of motor speed based on desired rotation direction
    if(turnDirection == CW) {
      turnSpeed = turnSpeed * -1;
      conversionRate = 14.28;
      Serial.println("Turntable CW");
    }
    else {
      conversionRate = 14.24;   //adjust tick taget due to tension from the main cable.
      Serial.println("Turntable CCW");
    }
    
    //convert turnDegrees to encoder ticks (90 degrees is approximately 14 ticks)
    if(turnDegrees > 0) {
      tickTarget = round((turnDegrees / 90.0) * conversionRate);  //round up to nearest integer value
      Serial.print("Turn Degrees: ");
      Serial.print(turnDegrees);
      Serial.print("\n");
      Serial.print("Conversion Rate: ");
      Serial.print(conversionRate);
      Serial.print("\n");
      Serial.print("TickTarget: ");
      Serial.print(tickTarget);
      Serial.print("\n");
    }

    turnTable.run(turnSpeed);
    while (turnTableCount < tickTarget) {
      turnTableAnalog = analogRead(TURNTABLE_ENCODER);
      if(turnTableAnalog > TURNTABLE_ANALOG_MAX && turnTableEncoder == 0) { //if encoder value passes the threshold for HIGH, and the current state of the sensor is LOW, set sensor state to HIGH and increment the tick count.
        turnTableEncoder = 1;
        turnTableCount++;   //Increment encoder tick count each time th sensor reads HIGH
      }
      else if(turnTableAnalog < TURNTABLE_ANALOG_MIN && turnTableEncoder == 1) {  //if encoder value goes below the threshold for LOW, and the current state of the sensor is HIGH, set the sensor state to LOW and wait for next trigger.
        turnTableEncoder = 0;
      }
      Serial.print("Turntable: ");
      Serial.print("A ");
      Serial.print(turnTableAnalog);
      Serial.print("\t");
      Serial.print("E ");
      Serial.print(turnTableEncoder);
      Serial.print("\t");
      Serial.print("C ");
      Serial.println(turnTableCount);
      delay (10);
    }
    
    //Brake motor once encoder tick count is achaieved
    turnTable.stop();
    turnTable.run(-turnSpeed); //Reverse motor direction to brake briefly
    delay(30);
    turnTable.run(0);    //Release motor by setting speed to zero
    turnTable.stop();
    runArray[4] = 0;
  }
}

void armGripper(int gripState, int gripTime = ARMGRIPTIME) {
  if (runArray[0] == 1) {
    runArray[0] = 0;
    int gripSpeed = 255;   // value: between -255 and 255. It is rarely necessary to change the gripper speed, so it is only a local variable
    Serial.print("\n");
    Serial.println("Arm Gripper...");
    if (gripState == OPEN) {
      Serial.println("Arm Open:");
      Serial.print("Speed: ");
      Serial.println(gripSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      Serial.print("\n");
      armGrip.run(gripSpeed); 
      delay(gripTime);
      armGrip.stop();
    }
    else if (gripState == CLOSE) {
      Serial.print("\n");
      Serial.println("Arm Close:");
      Serial.print("Speed: ");
      Serial.println(-gripSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      Serial.print("\n");
      armGrip.run(-gripSpeed); 
      delay(gripTime);
      armGrip.stop();
    }
    delay(1000);
  }
}

/*
 * Rotate wrist by declaring the desired orientation state (REQUIRED), a rotation direction, the number of complete revolutions, and the desired speed.
 */
void wristRotate(int targetState, int wristDirection = CW, float wristRevolution = 0.0, int wristSpeed = 255) {
  if(runArray[1] == 1) {    //Check flag to prevent unnecessary re-triggering of the function
    int count = 0;    //iterator variable to count the number of hardware screws passed during rotation (number of times the wrist switch is activated)
    float switchCount = 0;  //number of times the wrist switch is activated. 

    Serial.print("\n");
    Serial.print("Wrist Rotation!");
    Serial.print("\n");
    Serial.print("Wrist Direction: ");
    if(wristDirection == CW) {
      Serial.print("CW");
    }
    else if(wristDirection == CCW) {
      Serial.print("CCW");
    }
    Serial.print("\n");
    Serial.print("Target Orientation: ");
    if(targetState == H) {
      Serial.print("H");
    }
    else if(targetState == V) {
      Serial.print("V");
    }
    Serial.print("\n");
    
    //Update wristState
    if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == LOW) {  //If switch AND hall effect sensors are both "activated" (hall sensor is LOW when active), gripper is HORIZONTAL (H)
      wristState = H;
    }
    else if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == HIGH) { //If switch is activated (HIGH) but hall effect sensor is NOT (hall sensor is LOW when active), gripper is VERTICAL (V)
      wristState = V;
    }
    Serial.print("Start Orientation: ");
    if(wristState == H) {
      Serial.print("H");
    }
    else if(wristState == V) {
      Serial.print("V");
    }
    Serial.print("\n");
    Serial.print("Switch State: ");
    Serial.print(digitalRead(WRIST_SWITCH));
    Serial.print("\n");
    Serial.print("\n");

    //Set sign of motor speed based on desired rotation direction
    if(wristDirection == CCW) {   //rotation direction is determined as if the arm is a part of your body an you are looking down the arm toward the gripper
      wristSpeed = wristSpeed * -1;
    }
    //Convert wristRevolution value to required count of switch activations (number of hardware screws that must be passed during rotation)
    switchCount = int(wristRevolution * 4);   //There are 4 hardware screws per one revolution. A fractional value for wristRevolution will produce an integer value less than 4.
    Serial.print("Switch Count: ");
    Serial.print(switchCount);
    Serial.print("\n");
    Serial.print("\n");
    //Initialize wrist postion to the nearest cardinal position (determined by switch activation by one of 4 hardware screws combined with state of hall effect sensor)
    if(digitalRead(WRIST_SWITCH) == LOW){     //If switch starts in UNPRESSED (LOW) state (switch is NOT currently positioned over a hardware screw)
      armWrist.run(wristSpeed);               //Run motor until a switch is activated (which means wrist is in one of the cardinal directions)
      while(digitalRead(WRIST_SWITCH) == LOW) {
        //Wait for switch to go HIGH (switch is PRESSED)
      }
      delay(50);  //Insert slight delay to allow switch roller to fully seat over the hardware screw.
      //Brake motor once switch is activated
      armWrist.stop();
      armWrist.run(-wristSpeed); //Breifly reverse motor direction to brake
      delay(30);
      armWrist.run(0);    //Release motor by setting speed to zero
      armWrist.stop();

      //Update wristState
      if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == LOW) {  //If switch AND hall effect sensors are both "activated" (hall sensor is LOW when active), gripper is HORIZONTAL (H)
        wristState = H;
      }
      else if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == HIGH) { //If switch is activated (HIGH) but hall effect sensor is NOT (hall sensor is LOW when active), gripper is VERTICAL (V)
        wristState = V;
      }
      if(wristState == targetState && switchCount < 1) {    //if wrist was already in the desired orientation (wrist did not need to move), set flag that movement is finished
        runArray[1] = 0;
      }
    }
    else if(digitalRead(WRIST_SWITCH) == HIGH){ //If switch starts in PRESSED state (switch IS currently positioned over a hardware screw)
      if(wristState == targetState && switchCount < 1) {    //Check if the wrist is already in the desired target state and no additional rotation is required, set flag that movement is finished
        runArray[1] = 0;
      }
      else {    //otherwise, if wrist movement IS required...
        if(switchCount >= 1) {
          armWrist.run(wristSpeed);
          while(count < switchCount){   //Wait to pass through the required number of screws
            while(digitalRead(WRIST_SWITCH) == HIGH) {
              //Wait for switch to deactivate (move off of current screw position)
            }
            while(digitalRead(WRIST_SWITCH) == LOW) {
              //Wait for switch to re-activate (move onto the next screw position)
            }
            count++;    //Update count for each new screw detected during rotation
            Serial.print("Switch Count: ");
            Serial.print(switchCount);
            Serial.print("\t");
            Serial.print("Current Count: ");
            Serial.print(count);
            Serial.print("\n");
          }
          delay(50);  //Insert slight delay to allow switch roller to fully seat over the hardware screw.
          //Brake motor once correct number of screws are passed through
          armWrist.stop();
          armWrist.run(-wristSpeed); //Brake
          delay(30);
          armWrist.run(0);
          armWrist.stop();
  
          //Update wristState once required rotation has completed
          if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == LOW) {  //If switch AND hall effect are "activated" (hall sensor is LOW when active), gripper is HORIZONTAL (H)
            wristState = H;
          }
          else if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == HIGH) { //If switch is activated but hall effect sensor is NOT (hall sensor is LOW when active), gripper is VERTICAL (V)
            wristState = V;
          }
        }
      }
      
      if(wristState == targetState) { //If targetState has been achieved after rotation, set flag to finish routine and prevent function from re-running
        runArray[1] = 0;
      }
      else {    //If target state has NOT been achieved after rotation
        armWrist.run(wristSpeed);   //Run motor until switch is re-activated (which means wrist has rotated 90 degrees to a new cardinal direction but different from the last)
        while(digitalRead(WRIST_SWITCH) == HIGH) {
          //Wait for switch to deactivate
        }
        while(digitalRead(WRIST_SWITCH) == LOW) {
          //Wait for switch to re-activate
        }
        delay(50);  //Insert slight delay to allow switch roller to fully seat over the hardware screw.
        //Brake motor once switch is activated
        armWrist.stop();
        armWrist.run(-wristSpeed); //Reverse motor direction to brake briefly
        delay(30);
        armWrist.run(0);    //Release motor by setting speed to zero
        armWrist.stop();
  
        //Update wristState
        if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == LOW) {  //If switch AND hall effect are "activated" (hall effect sensor is LOW when activated), gripper is HORIZONTAL (H)
          wristState = H;
        }
        else if(digitalRead(WRIST_SWITCH) == HIGH && digitalRead(WRIST_HALL) == HIGH) { //If switch is activated but hall effect sensor is NOT (hall effect sensor is LOW when activated), gripper is VERTICAL (V)
          wristState = V;
        }
        if(wristState == targetState) { //If targetState has been achieved after rotation, set flag to finish routine and prevent function from re-running
          runArray[1] = 0;
        }
      }
    }
  }
}

/*
 * ELBOW RANGES
 * UpperMax Gripper fully "up" against arm bar): 760
 * MidPoint (Gripper roughly "level" with arm bar): 500
 * LowerMax Gripper fully "down" compared to arm bar): 80
 */
void elbowMove(int elbowPosition = 500, int elbowSpeed = 65) { //Default values allow the function to be called without arguments to reset to a default position (at the default speed).
  if(runArray[2] == 1) {    //Check flag to prevent unnecessary re-triggering of the function
    if(elbowPosition >= ELBOW_MIN && elbowPosition <= ELBOW_MAX) {   //Check if command value is within allowed range
      int lastPosition = analogRead(ELBOW_POT);   //read encoder position
      if(elbowPosition < lastPosition) {  //If the desired postion is physically LOWER than the last read position.
        Serial.print("\n");
        Serial.println("Elbow Down");
        Serial.print("Target Position: ");
        Serial.print(elbowPosition);
        Serial.print("\n\n");
        while(elbowPosition < lastPosition) {   //while target positiion is still lower than the last read position
          elbow.run(-elbowSpeed);   //set motor direction
          delay(10);               //wait for small amount of elbow movement
          lastPosition = analogRead(ELBOW_POT);   //get new position
          Serial.print("Elbow Position: ");
          Serial.println(lastPosition);
        }
        //Brake motor once target position is reached
        elbow.stop();
        elbow.run(elbowSpeed); //Reverse motor direction to brake briefly
        delay(30);
        elbow.run(0);    //Release motor by setting speed to zero
        elbow.stop();
      }
      else if(elbowPosition > lastPosition) {  //If the desired postion is physically HIGHER than the last read position.
        Serial.print("\n");
        Serial.println("Elbow Up");
        Serial.print("Target Position: ");
        Serial.print(elbowPosition);
        Serial.print("\n\n");
        while(elbowPosition > lastPosition) {
          elbow.run(elbowSpeed);
          delay(10);
          lastPosition = analogRead(ELBOW_POT);
          Serial.print("Elbow Position: ");
          Serial.println(lastPosition);
        }
        //Brake motor once target position is reached
        elbow.stop();
        elbow.run(-elbowSpeed); //Reverse motor direction to brake briefly
        delay(30);
        elbow.run(0);    //Release motor by setting speed to zero
        elbow.stop();
      }
      runArray[2] = 0;
    }
  }
}

/*
 * SHOULDER RANGES
 * LowerMax (Lowest arm bar "down" position over rear of robot): 600
 * MidPoint (Gripper roughly "level" with arm bar): 575
 * UpperMax (Highest arm bar position): 375
 */
void shoulderMove(int shoulderPosition = 575, int shoulderSpeed = 127) {  //Default values allow the function to be called without arguments to reset to a default position (at the default speed).
  if(runArray[3] == 1) {    //Check flag to prevent unnecessary re-triggering of the function
    if(shoulderPosition >= SHOULDER_MIN && shoulderPosition <= SHOULDER_MAX) {   //Check if command value is within allowed range.
      int lastPosition = analogRead(SHOULDER_POT);    //get last reading from sensor
      if(shoulderPosition < lastPosition) {  //If the desired postion is physically HIGHER than the last read position.
        Serial.print("\n");
        Serial.println("Shoulder Up");
        Serial.print("Target Position: ");
        Serial.print(shoulderPosition);
        Serial.print("\n\n");
        while(shoulderPosition < lastPosition) {
          shoulder.run(-shoulderSpeed);     //set motor direction
          delay(70);                       //wait for small amount of shoulder movement
          lastPosition = analogRead(SHOULDER_POT);    //get new position
          Serial.print("Shoulder Postition: ");
          Serial.println(lastPosition);
        }
        //Brake motor once switch is activated
        shoulder.stop();
        shoulder.run(shoulderSpeed); //Reverse motor direction to brake briefly
        delay(30);
        shoulder.run(0);    //Release motor by setting speed to zero
        shoulder.stop();
      }
      else if(shoulderPosition > lastPosition) {  //If the desired postion is physically LOWER than the last read position.
        Serial.print("\n");
        Serial.println("Shoulder Down");
        Serial.print("Target Position: ");
        Serial.print(shoulderPosition);
        Serial.print("\n\n");
        while(shoulderPosition > lastPosition) {
          shoulder.run(shoulderSpeed);
          delay(70);
          lastPosition = analogRead(SHOULDER_POT);
          Serial.print("Shoulder Position: ");
          Serial.println(lastPosition);
        }
        //Brake motor once switch is activated
        shoulder.stop();
        shoulder.run(-shoulderSpeed); //Reverse motor direction to brake briefly
        delay(30);
        shoulder.run(0);    //Release motor by setting speed to zero
        shoulder.stop();
      }
      runArray[3] = 0;
    }
  }
}

  
