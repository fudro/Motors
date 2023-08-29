/**
 * \par Copyright (C), 2012-2017, MakeBlock
 * @file    MeMegaPiProDCMotorTest.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2017/05/10
 * @brief   Description: this file is sample code for MegaPi Pro DC motor device.
 *
 * Function List:
 *    1. void MeMegaPiProDCMotorTest::run(int16_t speed)
 *    2. void MeMegaPiProDCMotorTest::stop(void)
 *
 * \par History:
 * <pre>
 * <Author>        <Time>        <Version>      <Descr>
 * Lan weiting     2017/05/10     1.0.0          build the new
 * Zzipeng         2017/05/12     1.0.1          build the new
 * Anthony Fudd    2021/03/28     1.0.2          Added notes to the program description
 * </pre>
 * 
 * HARDWARE COMPATIBILITY:
 * MegaPi Pro Microcontroller
 * MegaPi Pro 4DC Motor Driver Expansion Module
 * IMPORTANT:
 * The MegaPi Pro 4DC Motor Expansion Module is capable of 3A continous and 5.5A peak.
 * 
 * NOTES:
 * This program works ONLY with MegaPi Pro microcontoller to control up to 4 motors via the expansion module.
 * The expansion module prvides an additional 4 DC motor ports.
 * 
 */
#include "MeMegaPiPro.h"

MeDCMotor motor1(M9);

MeDCMotor motor2(M10);

MeDCMotor motor3(M11);

MeDCMotor motor4(M12);

uint8_t motorSpeed = 250;
int potValue = 0;
int targetValue = 500;
int runFlag = 0;

void setup()
{
  pinMode(A1, INPUT);
  Serial.begin(115200);
  Serial.println("MEGAPI PRO MOTOR TEST");
}

void loop()
{
  shoulderMove();
}

void displayValue() {
  potValue = analogRead(A1);
  Serial.print("pot: ");
  Serial.print(potValue);
  Serial.print("\n");
  delay(100);
}

void shoulderMove() {
  if(runFlag == 0) {
    displayValue();
    Serial.println("Shoulder Move!");
    delay(1000);
    Serial.println("START!");
    motor1.run(motorSpeed); /* value: between -255 and 255. */
  //  motor2.run(motorSpeed); /* value: between -255 and 255. */
  //  motor3.run(motorSpeed);
  //  motor4.run(motorSpeed);
    delay(1500);
    motor1.stop();
  //  motor2.stop();
  //  motor3.stop();
  //  motor4.stop();
    displayValue();
    delay(1000);
    motor1.run(-motorSpeed);
  //  motor2.run(-motorSpeed);
  //  motor3.run(-motorSpeed);
  //  motor4.run(-motorSpeed);
    delay(1500);
    motor1.stop();
  //  motor2.stop();
  //  motor3.stop();
  //  motor4.stop();
    displayValue();
    runFlag = 1;
    Serial.println("DONE!");
  }
}
