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
MeMegaPiDCMotor motor3(PORT2A); //Drive Left
MeMegaPiDCMotor motor4(PORT2B); //Drive Right
MeMegaPiDCMotor motor5(PORT3A); //Tail Gripper
MeMegaPiDCMotor motor6(PORT3B); //Turntable
MeMegaPiDCMotor elbow(PORT4A); //Elbow
MeMegaPiDCMotor shoulder(PORT4B); //Shoulder

uint8_t motorSpeed = 100;
const int OPEN = 0;
const int CLOSE = 1;
const int H = 0;    //HORIZONTAL
const int V = 1;    //VERTICAL
const int CW = 0;
const int CCW = 1;
const int WRIST_HALL = A6;
const int WRIST_SWITCH = A7;
const int ARMGRIPTIME = 1200;
const int WRISTTIME = 1000; //Wrist rotation at 255 speed for 1000 milliseconds is approximately 180 degres of rotation (or one-half turn)
int wristSwitch = 0;
int wristState = H; //horizontal direcction as default
int runFlag = 0;

/***********************************
 * FUNCTION PROTOTYPES
***********************************/
void armGripper(int gripState, int gripTime = ARMGRIPTIME);
void wristRotation (int targetState, int wristDirection = CW, float wristRotation = 0.0, int wristSpeed = 255);  //targetState is VERTICAL or HORIZONTAL, wristRotation is the number of full revolutions to be perfomed.
void elbowMove(int elbowPosition, int elbowSpeed = 255);
void shoulderMove(int shoulderPosition = 100, int shoulderSpeed = 255);

void setup()
{
  //Setup sensor pins
  pinMode(A6, INPUT_PULLUP);  //wrist hall effect
  pinMode(A7, INPUT); //wrist switch
  pinMode(A8, INPUT); //elbow potentiomter
  pinMode(A9, INPUT); //shoulder potentiometer
  Serial.begin(115200);
  Serial.println("HomeBot Sensor Ranges Test!");
  delay(1000);
  Serial.println("StartingTest in 3 seconds...");
  delay(3000);
}

void loop()
{
//  armGripper(OPEN);
//  wristRotation(V);
//  elbowMove(100, 255);
  shoulderMove(100, 255);



/***************************
 * OLD CODE
 * Use as basis for new updated functions.
 **************************/
//  Serial.println("Wrist...");
//  Serial.println("Wrist Clockwise");
//  armWrist.run(motorSpeed);
//  delay(500);
//  armWrist.stop();
//  delay(500);
//  Serial.println("Wrist Counter-Clockwise");
//  armWrist.run(-motorSpeed);
//  delay(500);
//  armWrist.stop();
//  Serial.print("\n");
//  delay(1000);
//
//  Serial.println("Drive Left...");
//  Serial.println("Reverse");
//  motor3.run(motorSpeed);
//  delay(500);
//  motor3.stop();
//  delay(500);
//  Serial.println("Forward");
//  motor3.run(-motorSpeed);
//  delay(500);
//  motor3.stop();
//  Serial.print("\n");
//  delay(1000);
//
//  Serial.println("Drive Right...");
//  Serial.println("Reverse");
//  motor4.run(motorSpeed);
//  delay(500);
//  motor4.stop();
//  delay(500);
//  Serial.println("Forward");
//  motor4.run(-motorSpeed);
//  delay(500);
//  motor4.stop();
//  Serial.print("\n");
//  delay(1000);
//
//  Serial.println("Tail Gripper...");
//  Serial.println("Tail Close");
//  motor5.run(motorSpeed);
//  delay(500);
//  motor5.stop();
//  delay(500);
//  Serial.println("Tail Open");
//  motor5.run(-motorSpeed);
//  delay(500);
//  motor5.stop();
//  Serial.print("\n");
//  delay(1000);
//
//  Serial.println("Turntable...");
//  Serial.println("Turntable Counter-Clockwise");
//  motor6.run(motorSpeed);
//  delay(500);
//  motor6.stop();
//  delay(500);
//  Serial.println("Turntable Clockwise");
//  motor6.run(-motorSpeed);
//  delay(500);
//  motor6.stop();
//  Serial.print("\n");
//  delay(1000);
//
//  Serial.println("Elbow...");
//  Serial.println("Elbow Up");
//  elbow.run(motorSpeed);
//  delay(500);
//  elbow.stop();
//  delay(500);
//  Serial.println("Elbow Down");
//  elbow.run(-motorSpeed);
//  delay(500);
//  elbow.stop();
//  Serial.print("\n");
//  delay(1000);
//
//  Serial.println("Shoulder...");
//  Serial.println("Shoulder Down");
//  shoulder.run(motorSpeed);
//  delay(500);
//  shoulder.stop();
//  delay(500);
//  Serial.println("Shoulder Up");
//  shoulder.run(-motorSpeed);
//  delay(500);
//  shoulder.stop();
//  Serial.print("\n");
//  Serial.print("\n");
//  delay(1000);
}

void armGripper(int gripState, int gripTime = ARMGRIPTIME) {
  if (runFlag == 0) {
    runFlag = 1;
    int gripSpeed = 255;   // value: between -255 and 255. It is rarely necessary to change the gripper speed, so it is only a local variable
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
 * Rotate wrist by declaring the desired end state (REQUIRED), a rotation direction, the number of complete revolutions, and the desired speed.
 */
void wristRotation(int targetState, int wristDirection = CW, float wristRotation = 0.0, int wristSpeed = 255) {
  if(runFlag == 0) {    //Check flag to prevent unnecessary re-triggering of the function
    int count = 0;
    float switchCount = 0;

    //Set sign of motor speed based on desired rotation direction
    if(wristDirection == CCW) {
      wristSpeed = wristSpeed * -1;
    }
    //Convert wristRotations to required count of switch activations (number of hardware screws that must be passed during rotation)
    switchCount = int(wristRotation * 4);   //There are 4 hardware screws per one revolution. A fractional value for wristRotation will produce an integer value less than 4.
    
    //Initialize wrist postion to the nearest cardinal position (determined by switch activation by one of 4 hardware screws combined with state of hall effect sensor)
    if(digitalRead(WRIST_SWITCH) == LOW){     //If switch starts in OPEN state (not activated by a hardware screw)
      armWrist.run(wristSpeed);               //Run motor until switch is activated (which means wrist is in one of the cardinal directions)
      while(digitalRead(WRIST_SWITCH) == LOW) {
        //Wait for switch to go HIGH (switch is PRESSED)
      }
      //Brake motor once switch is activated
      armWrist.stop();
      armWrist.run(-wristSpeed); //Breifly reverse motor direction to brake
      delay(30);
      armWrist.run(0);    //Release motor by setting speed to zero
      armWrist.stop();

      //Update wristState
      if(digitalRead(WRIST_SWITCH) == HIGH && analogRead(WRIST_HALL) < 50) {  //If switch AND hall effect sensors are "activated" (hall sensor is LOW when active), gripper is HORIZONTAL
        wristState = H;
      }
      else if(digitalRead(WRIST_SWITCH) == HIGH && analogRead(WRIST_HALL) > 1000) {
        wristState = V;
      }
      if(wristState == targetState && switchCount < 1) {
        runFlag = 1;
      }
    }
    else if(digitalRead(WRIST_SWITCH) == HIGH){ //If switch starts in PRESSED state
      if(wristState == targetState && switchCount < 1) {    //Check if the wrist is already in the desired target state and no additiona rotation is required
        runFlag = 1;
      }
      else {
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
          }
          //Brake motor once correct number of screws are passed through
          armWrist.stop();
          armWrist.run(-wristSpeed); //Brake
          delay(30);
          armWrist.run(0);
          armWrist.stop();
  
          //Update wristState once required rotation has completed
          if(digitalRead(WRIST_SWITCH) == HIGH && analogRead(WRIST_HALL) < 50) {  //If switch AND hall effect are "activated" (hall sensor is LOW when active), gripper is HORIZONTAL
            wristState = H;
          }
          else if(digitalRead(WRIST_SWITCH) == HIGH && analogRead(WRIST_HALL) > 1000) {
            wristState = V;
          }
        }
      }
      
      if(wristState == targetState) { //If targetState has been achieved after rotation, set flag to finish routine and prevent function from re-running
        runFlag = 1;
      }
      else {    //If target state has NOT been achieved after rotation
        armWrist.run(wristSpeed);   //Run motor until switch is re-activated (which means wrist has rotated 90 degrees to a new cardinal directions but different from the last)
        while(digitalRead(WRIST_SWITCH) == HIGH) {
          //Wait for switch to deactivate
        }
        while(digitalRead(WRIST_SWITCH) == LOW) {
          //Wait for switch to re-activate
        }
        //Brake motor once switch is activated
        armWrist.stop();
        armWrist.run(-wristSpeed); //Reverse motor direction to brake briefly
        delay(30);
        armWrist.run(0);    //Release motor by setting speed to zero
        armWrist.stop();
  
        //Update wristState
        if(digitalRead(WRIST_SWITCH) == HIGH && analogRead(WRIST_HALL) < 50) {  //If switch AND hall effect are activated, gripper is HORIZONTAL
          wristState = H;
        }
        else if(digitalRead(WRIST_SWITCH) == HIGH && analogRead(WRIST_HALL) > 1000) {
          wristState = V;
        }
        if(wristState == targetState) { //If targetState has been achieved after rotation, set flag to finish routine and prevent function from re-running
          runFlag = 1;
        }
      }
    }
  }
}

void elbowMove(int elbowPosition = 100, int elbowSpeed = 255) { //Default values allow the function to be called without arguments to reset to a default position (at the default speed).
  int lastPosition = analogRead(A8);
  if(elbowPosition < lastPosition) {  //If the desired postion is lower than the current position.
    Serial.println("Elbow Down");
    while(elbowPosition < lastPosition) {
      elbow.run(-elbowSpeed);
      delay(100);
      lastPosition = analogRead(A8);
    }
    //Brake motor once switch is activated
    elbow.stop();
    elbow.run(elbowSpeed); //Reverse motor direction to brake briefly
    delay(30);
    elbow.run(0);    //Release motor by setting speed to zero
    elbow.stop();
  }
  else if(elbowPosition > lastPosition) {  //If the desired postion is lower than the current position.
    Serial.println("Elbow Up");
    while(elbowPosition > lastPosition) {
      elbow.run(elbowSpeed);
      delay(100);
      lastPosition = analogRead(A8);
    }
    //Brake motor once switch is activated
    elbow.stop();
    elbow.run(-elbowSpeed); //Reverse motor direction to brake briefly
    delay(30);
    elbow.run(0);    //Release motor by setting speed to zero
    elbow.stop();
  }
}

void shoulderMove(int shoulderPosition = 100, int shoulderSpeed = 255) {  //Default values allow the function to be called without arguments to reset to a default position (at the default speed).
  int lastPosition = analogRead(A9);
  if(shoulderPosition < lastPosition) {  //If the desired postion is lower than the current position.
    Serial.println("Shoulder Down");
    while(shoulderPosition < lastPosition) {
      shoulder.run(-shoulderSpeed);
      delay(100);
      lastPosition = analogRead(A9);
    }
    //Brake motor once switch is activated
    shoulder.stop();
    shoulder.run(shoulderSpeed); //Reverse motor direction to brake briefly
    delay(30);
    shoulder.run(0);    //Release motor by setting speed to zero
    shoulder.stop();
  }
  else if(shoulderPosition > lastPosition) {  //If the desired postion is lower than the current position.
    Serial.println("Shoulder Up");
    while(shoulderPosition > lastPosition) {
      shoulder.run(shoulderSpeed);
      delay(100);
      lastPosition = analogRead(A9);
    }
    //Brake motor once switch is activated
    shoulder.stop();
    shoulder.run(-shoulderSpeed); //Reverse motor direction to brake briefly
    delay(30);
    shoulder.run(0);    //Release motor by setting speed to zero
    shoulder.stop();
  }
}

  
