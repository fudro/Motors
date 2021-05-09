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
MeMegaPiDCMotor motor7(PORT4A); //Elbow
MeMegaPiDCMotor motor8(PORT4B); //Shoulder

uint8_t motorSpeed = 100;

const int OPEN = 0;
const int CLOSE = 1;
const int HORIZONTAL = 0;
const int VERTICAL = 1;
const int CW = 0;
const int CCW = 1;
const int ARMGRIPTIME = 1200;
const int WRISTTIME = 1000; //Wrist rotation at 255 speed for 1000 milliseconds is approximately 180 degres of rotation (or one-half turn)
int wristSwitch = 0;
int wristState = HORIZONTAL;
int runFlag = 0;

/***********************************
 * FUNCTION PROTOTYPES
***********************************/
void armGripper(int gripDirection, int gripTime = ARMGRIPTIME);
//void armWristRotation(int wristDirection, int wristTime = WRISTTIME);
void armWristRotation(int targetState, int wristDirection = CW, int wristSpeed = 255, int wristRotation = 0);  //targetState is VERTICAL or HORIZONTAL, wristRotations is the number of full rotations that shoudl be perfomed once the arma reaches the target state.

void setup()
{
  pinMode(A6, INPUT_PULLUP);
  pinMode(A7, INPUT);
  Serial.begin(115200);
  Serial.println("HomeBot Sensor Ranges Test!");
  delay(1000);
  Serial.println("StartingTest in 3 seconds...");
  delay(3000);
}

void loop()
{
//  armGripper(OPEN);
  armWristRotation(VERTICAL);



/***************************
 * OLD CODE
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
//  motor7.run(motorSpeed);
//  delay(500);
//  motor7.stop();
//  delay(500);
//  Serial.println("Elbow Down");
//  motor7.run(-motorSpeed);
//  delay(500);
//  motor7.stop();
//  Serial.print("\n");
//  delay(1000);
//
//  Serial.println("Shoulder...");
//  Serial.println("Shoulder Down");
//  motor8.run(motorSpeed);
//  delay(500);
//  motor8.stop();
//  delay(500);
//  Serial.println("Shoulder Up");
//  motor8.run(-motorSpeed);
//  delay(500);
//  motor8.stop();
//  Serial.print("\n");
//  Serial.print("\n");
//  delay(1000);
}

void armGripper(int gripDirection, int gripTime = ARMGRIPTIME) {
  if (runFlag == 0) {
    runFlag = 1;
    motorSpeed = 255;   /* value: between -255 and 255. */
    Serial.println("Arm Gripper...");
    if (gripDirection == OPEN) {
      Serial.println("Arm Open:");
      Serial.print("Speed: ");
      Serial.println(motorSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      Serial.print("\n");
      armGrip.run(motorSpeed); 
      delay(gripTime);
      armGrip.stop();
    }
    else if (gripDirection == CLOSE) {
      Serial.println("Arm Close:");
      Serial.print("Speed: ");
      Serial.println(-motorSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      Serial.print("\n");
      armGrip.run(-motorSpeed); 
      delay(gripTime);
      armGrip.stop();
    }
    delay(1000);
  }
}

void armWristRotation(int targetState, int wristDirection = CW, int wristSpeed = 255, int wristRotation = 0) {
  if(runFlag == 0) {
    runFlag = 1;
    if(digitalRead(A7) == LOW){
      armWrist.run(wristSpeed);
      while(digitalRead(A7) == LOW) {
      
      }
      armWrist.stop();
      armWrist.run(-wristSpeed); //Brake
      delay(30);
      armWrist.run(0);
      armWrist.stop();
    }
    else if(digitalRead(A7) == HIGH){
      armWrist.run(wristSpeed);
      while(digitalRead(A7) == HIGH) {
      
      }
      while(digitalRead(A7) == LOW) {
      
      }
      armWrist.stop();
      armWrist.run(-wristSpeed); //Brake
      delay(30);
      armWrist.run(0);
      armWrist.stop();
    }
  }
}


//void armWristRotation(int wristDirection, int wristTime = WRISTTIME) {
//  if (runFlag == 0) {
//    runFlag = 1;
//    motorSpeed = 255;   /* value: between -255 and 255. */
//    Serial.println("Arm Wrist...");
//    if (wristDirection == CW) {
//      Serial.println("Wrist Clockwise:");
//      Serial.print("Speed: ");
//      Serial.println(motorSpeed);
//      Serial.print("Time: ");
//      Serial.println(wristTime);
//      Serial.print("\n");
//      armWrist.run(motorSpeed); 
////      delay(wristTime);
//      
//      int count = 0;
//      int hallSensor = 0;
//      while (count < 2) {
//        hallSensor = analogRead(A6);
//        Serial.print("Hall Sensor: ");
//        Serial.println(hallSensor);
//        delay(10);
//        if(hallSensor < 50 && wristState == VERTICAL) {   //if hall sensor detects magnet...
//          wristState = HORIZONTAL;
//          Serial.print("Wrist State: ");
//          Serial.println(wristState);
//          count++;
//        }
//        else if(hallSensor > 50 && wristState == HORIZONTAL) {
//          wristState = VERTICAL;
//          Serial.print("Wrist State: ");
//          Serial.println(wristState);
//        }
//      }
//      
//      armWrist.stop();
//    }
//    else if (wristDirection == CCW) {
//      Serial.println("Wrist Counter-Clockwise:");
//      Serial.print("Speed: ");
//      Serial.println(-motorSpeed);
//      Serial.print("Time: ");
//      Serial.println(wristTime);
//      Serial.print("\n");
//      armWrist.run(-motorSpeed); 
//      delay(wristTime);
//      armWrist.stop();
//    }
//    delay(1000);
//  }
//}
  
