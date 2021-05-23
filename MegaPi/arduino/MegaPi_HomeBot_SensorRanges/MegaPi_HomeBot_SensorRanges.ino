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
MeMegaPiDCMotor tailGrip(PORT3A); //Tail Gripper
MeMegaPiDCMotor turnTable(PORT3B); //Turntable
MeMegaPiDCMotor elbow(PORT4A); //Elbow
MeMegaPiDCMotor shoulder(PORT4B); //Shoulder

//uint8_t motorSpeed = 100;
const int OPEN = 0;
const int CLOSE = 1;
const int H = 0;    //HORIZONTAL
const int V = 1;    //VERTICAL
const int CW = 0;
const int CCW = 1;
const int WRIST_HALL = A14;  //sensor pin assignment
const int WRIST_SWITCH = A15;
const int ELBOW_POT = A8;
const int SHOULDER_POT = A9;
const int TURNTABLE_HALL = A11;
const int TURNTABLE_ENCODER = A10;
const int ARMGRIPTIME = 1200; //Time for arm gripper at max speed 255 to go from fully open to fully closed and vice versa.
const int WRISTTIME = 1000; //Wrist rotation at max speed 255 for 1000 milliseconds is approximately 180 degres of rotation (or one-half turn)
const int TAILGRIPTIME = 400;  //Time for tail gripper at max speed 255 to go from fully open to fully closed and vice versa.
const int ELBOW_MIN = 80; //Limit values for sensor
const int ELBOW_MAX = 760;
const int SHOULDER_MIN = 330;
const int SHOULDER_MAX = 600;
const int TURNTABLE_ANALOG_MAX = 300;
const int TURNTABLE_ANALOG_MIN = 150;
int wristSwitch = 0;  //store value of hardware switch
int wristState = H; //current state of wrist, horizontal direction as default
int turnTableAnalog = 0;  //analog value of turntable encoder
int turnTableEncoder = 1; //state of turntable encoder. Set initial stae to 1 since this pin is pulled high
int turnTableCount = 1;  //store value of turntable position based on encoder ticks. Default to 1 for easy math (15 ticks ~ 90 degrees)
int turnTableHall = 0; //state of turntable hall effect sensor
int runFlag = 0;

/***********************************
 * FUNCTION PROTOTYPES
***********************************/
void armGripper(int gripState, int gripTime = ARMGRIPTIME); //gripState is OPEN or CLOSE
void wristRotation (int targetState, int wristDirection = CW, float wristRotation = 0.0, int wristSpeed = 255);  //targetState is (V)ertical or (H)orizontal, wristRotation is the number of full revolutions to be perfomed.
void elbowMove(int elbowPosition = 500, int elbowSpeed = 127);  //approximate center position and preferred default speed.
void shoulderMove(int shoulderPosition = 515, int shoulderSpeed = 127); //approximate center position and preferred default speed.
void turnTableMove(int turnDegrees = 0, int turnDirection = CW, int turnSpeed = 65);    //turnDegrees is the degrees of angular rotation from the current position
void tailGripper(int gripState, int gripTime = TAILGRIPTIME); //gripState is OPEN or CLOSE

void setup()
{
  //Setup sensor pins
  pinMode(WRIST_HALL, INPUT_PULLUP);  //wrist hall effect
  pinMode(WRIST_SWITCH, INPUT); //wrist switch
  pinMode(ELBOW_POT, INPUT); //elbow potentiomter
  pinMode(SHOULDER_POT, INPUT); //shoulder potentiometer
  pinMode(TURNTABLE_HALL, INPUT_PULLUP); //turntable hall effect
  pinMode(TURNTABLE_ENCODER, INPUT_PULLUP); //turntable encoder
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
//  elbowMove();
//  shoulderMove();
//  turnTableMove(45, CCW);
  tailGripper(CLOSE);
  





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
//  tailGrip.run(motorSpeed);
//  delay(500);
//  tailGrip.stop();
//  delay(500);
//  Serial.println("Tail Open");
//  tailGrip.run(-motorSpeed);
//  delay(500);
//  tailGrip.stop();
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

void tailGripper(int gripState, int gripTime = TAILGRIPTIME) {
   if (runFlag == 0) {
    runFlag = 1;
    int gripSpeed = 128;   // value: between -255 and 255. It is rarely necessary to change the gripper speed, so it is only a local variable
    Serial.println("Tail Gripper...");
    if (gripState == CLOSE) {
      Serial.println("Tail Open:");
      Serial.print("Speed: ");
      Serial.println(gripSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      Serial.print("\n");
      tailGrip.run(gripSpeed); 
      delay(gripTime);
      
      //Brake motor once switch is activated
      tailGrip.stop();
      tailGrip.run(-gripSpeed); //Reverse motor direction to brake briefly
      delay(10);
      tailGrip.run(0);    //Release motor by setting speed to zero
      tailGrip.stop();
    }
    else if (gripState == OPEN) {
      Serial.println("Tail Close:");
      Serial.print("Speed: ");
      Serial.println(-gripSpeed);
      Serial.print("Time: ");
      Serial.println(gripTime);
      Serial.print("\n");
      tailGrip.run(-gripSpeed); 
      delay(gripTime);
      
      //Brake motor once switch is activated
      tailGrip.stop();
      tailGrip.run(gripSpeed); //Reverse motor direction to brake briefly
      delay(10);
      tailGrip.run(0);    //Release motor by setting speed to zero
      tailGrip.stop();
    }
    delay(1000);
  }
}

void turnTableMove(int turnDegrees = 0, int turnDirection = CW, int turnSpeed = 65) {
  if(runFlag == 0) {    //Check flag to prevent unnecessary re-triggering of the function
    float tickTarget = 1.0;

    //convert turnDegrees to encoder ticks (90 degrees is approximately 15 ticks)
    if(turnDegrees > 0) {
      tickTarget = round((turnDegrees / 90.0) * 15);  //round up to nearest integer value
      Serial.print("Turn Degrees: ");
      Serial.println(turnDegrees);
      Serial.print("TickTarget: ");
      Serial.println(tickTarget);
    }
    
    //Set sign of motor speed based on desired rotation direction
    if(turnDirection == CW) {
      turnSpeed = turnSpeed * -1;
      Serial.println("Turntable CW");
    }
    else {
      Serial.println("Turntable CCW");
    }

    turnTable.run(turnSpeed);
    while (turnTableCount < tickTarget) {
      turnTableAnalog = analogRead(TURNTABLE_ENCODER);
      if(turnTableAnalog > TURNTABLE_ANALOG_MAX && turnTableEncoder == 0) {
        turnTableEncoder = 1;
        turnTableCount++;
      }
      else if(turnTableAnalog < TURNTABLE_ANALOG_MIN && turnTableEncoder == 1) {
        turnTableEncoder = 0;
      }
      Serial.print(turnTableAnalog);
      Serial.print("\t");
      Serial.print(turnTableEncoder);
      Serial.print("\t");
      Serial.println(turnTableCount);
      delay (10);
    }
    
    //Brake motor once switch is activated
    turnTable.stop();
    turnTable.run(-turnSpeed); //Reverse motor direction to brake briefly
    delay(30);
    turnTable.run(0);    //Release motor by setting speed to zero
    turnTable.stop();
    runFlag = 1;
  }
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

/*
 * ELBOW RANGES
 * UpperMax Gripper fully "up" against arm bar): 760 sensor reads 770 at "up"
 * MidPoint (Gripper roughly "level" with arm bar): 500 sensor reads ~490 at "level"
 * LowerMax Gripper fully "down" compared to arm bar): 80 sensor reads 65 at 90 degrees "down" compared to arm bar
 */
void elbowMove(int elbowPosition = 500, int elbowSpeed = 127) { //Default values allow the function to be called without arguments to reset to a default position (at the default speed).
  if(elbowPosition >= ELBOW_MIN && elbowPosition <= ELBOW_MAX) {   //Check ranges
    if(runFlag == 0) {    //Check flag to prevent unnecessary re-triggering of the function
      int lastPosition = analogRead(ELBOW_POT);
      if(elbowPosition < lastPosition) {  //If the desired postion is lower than the current position.
        Serial.println("Elbow Down");
        while(elbowPosition < lastPosition) {
          elbow.run(-elbowSpeed);
          delay(100);
          lastPosition = analogRead(ELBOW_POT);
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
          lastPosition = analogRead(ELBOW_POT);
        }
        //Brake motor once switch is activated
        elbow.stop();
        elbow.run(-elbowSpeed); //Reverse motor direction to brake briefly
        delay(30);
        elbow.run(0);    //Release motor by setting speed to zero
        elbow.stop();
      }
      runFlag = 1;
//  Serial.print("Elbow Position: ");
//  Serial.println(analogRead(ELBOW_POT));
    }
  }
}

/*
 * SHOULDER RANGES
 * LowerMax (Lowest arm bar "down" position over rear of robot): 600 sensor reads 515 at lowest arm bar position over rear of robot
 * MidPoint (Gripper roughly "level" with arm bar): 515 sensor reads ~520 at "level"
 * UpperMax (Highest arm bar position): 330 sensor reads ~330 at "highest" position
 */
void shoulderMove(int shoulderPosition = 515, int shoulderSpeed = 127) {  //Default values allow the function to be called without arguments to reset to a default position (at the default speed).
  if(shoulderPosition >= SHOULDER_MIN && shoulderPosition <= SHOULDER_MAX) {   //Check ranges
    if(runFlag == 0) {    //Check flag to prevent unnecessary re-triggering of the function
      int lastPosition = analogRead(SHOULDER_POT);
      if(shoulderPosition < lastPosition) {  //If the desired postion is lower than the current position.
        Serial.println("Shoulder Down");
        while(shoulderPosition < lastPosition) {
          shoulder.run(-shoulderSpeed);
          delay(100);
          lastPosition = analogRead(SHOULDER_POT);
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
          lastPosition = analogRead(SHOULDER_POT);
        }
        //Brake motor once switch is activated
        shoulder.stop();
        shoulder.run(-shoulderSpeed); //Reverse motor direction to brake briefly
        delay(30);
        shoulder.run(0);    //Release motor by setting speed to zero
        shoulder.stop();
      }
      runFlag = 1;
//  Serial.print("Shoulder Position: ");
//  Serial.println(analogRead(SHOULDER_POT));
    }
  }
}

  
