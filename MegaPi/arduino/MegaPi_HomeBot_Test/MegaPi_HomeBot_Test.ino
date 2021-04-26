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

MeMegaPiDCMotor motor1(PORT1A);

MeMegaPiDCMotor motor2(PORT1B);

MeMegaPiDCMotor motor3(PORT2A);

MeMegaPiDCMotor motor4(PORT2B);

MeMegaPiDCMotor motor5(PORT3A);

MeMegaPiDCMotor motor6(PORT3B);

MeMegaPiDCMotor motor7(PORT4A);

MeMegaPiDCMotor motor8(PORT4B);

uint8_t motorSpeed = 100;

void setup()
{
}

void loop()
{
  motor1.run(motorSpeed); /* value: between -255 and 255. */
  delay(500);
  motor1.stop();
  delay(500);
  motor1.run(-motorSpeed);
  delay(500);
  motor1.stop();
  delay(1000);
  
  motor2.run(motorSpeed);
  delay(500);
  motor2.stop();
  delay(500);
  motor2.run(-motorSpeed);
  delay(500);
  motor2.stop();
  delay(1000);

  motor3.run(motorSpeed);
  delay(500);
  motor3.stop();
  delay(500);
  motor3.run(-motorSpeed);
  delay(500);
  motor3.stop();
  delay(1000);

  motor4.run(motorSpeed);
  delay(500);
  motor4.stop();
  delay(500);
  motor4.run(-motorSpeed);
  delay(500);
  motor4.stop();
  delay(1000);

  motor5.run(motorSpeed);
  delay(500);
  motor5.stop();
  delay(500);
  motor5.run(-motorSpeed);
  delay(500);
  motor5.stop();
  delay(1000);

  motor6.run(motorSpeed);
  delay(500);
  motor6.stop();
  delay(500);
  motor6.run(-motorSpeed);
  delay(500);
  motor6.stop();
  delay(1000);

  motor7.run(motorSpeed);
  delay(500);
  motor7.stop();
  delay(500);
  motor7.run(-motorSpeed);
  delay(500);
  motor7.stop();
  delay(1000);

  motor8.run(motorSpeed);
  delay(500);
  motor8.stop();
  delay(500);
  motor8.run(-motorSpeed);
  delay(500);
  motor8.stop();
  delay(1000);
}
  
