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
 * Zzipeng         2017/05/14     1.0.1          complete the code.s
 * Anthony Fudd    2021/03/28     1.0.2          Added notes to program description
 * </pre>
 * 
 * HARDWARE COMPATIBILITY:
 * MegaPi Pro Microcontroller
 * Encoder/DC Motor Driver Board (both MegaPi and MegaPi Pro versions)
 * IMPORTANT:
 * There are 2 versions of the Encoder/DC motor driver board. Both versions will work with this program.
 * MegaPi Version has a larger chip on the board and is capable of 1A continuous and 2A peak.
 * MegaPi Pro Version has a smaller chip on the board and is capable of 3A continous and 5.5A peak.
 * 
 * NOTES:
 * This program works ONLY with MegaPi Pro microcontoller to control up to 8 DC motors.
 * The MegaPi Pro has 4 main motor "ports". (An additional 4 ports can be added via an expansion module.)
 * Each main motor port accepts a detachable Encoder/DC motor driver board.
 * Each driver board is capable of driving two DC motors.
 * 
 */
#include "MeMegaPiPro.h"

MeMegaPiDCMotor motor1(PORT1A);

MeMegaPiDCMotor motor2(PORT1B);

MeMegaPiDCMotor motor3(PORT2A);

MeMegaPiDCMotor motor4(PORT2B);

MeMegaPiDCMotor motor5(PORT3A);

MeMegaPiDCMotor motor6(PORT3B);

MeMegaPiDCMotor motor7(PORT4A);

MeMegaPiDCMotor motor8(PORT4B);

uint8_t motorSpeed = 250;

void setup()
{
    TCCR1A = _BV(WGM10);//timer1 will be set to 490hz in setup function
    TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);//970hz
}

void loop()
{
  motor1.run(motorSpeed); /* value: between -255 and 255. */
  motor2.run(motorSpeed); /* value: between -255 and 255. */
  motor3.run(motorSpeed);
  motor4.run(motorSpeed);
  motor5.run(motorSpeed); /* value: between -255 and 255. */
  motor6.run(motorSpeed); /* value: between -255 and 255. */
  motor7.run(motorSpeed);
  motor8.run(motorSpeed);
  delay(2000);
  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  motor5.stop();
  motor6.stop();
  motor7.stop();
  motor8.stop();
  delay(1000);
  motor1.run(-motorSpeed);
  motor2.run(-motorSpeed);
  motor3.run(-motorSpeed);
  motor4.run(-motorSpeed);
  motor5.run(-motorSpeed); /* value: between -255 and 255. */
  motor6.run(-motorSpeed); /* value: between -255 and 255. */
  motor7.run(-motorSpeed);
  motor8.run(-motorSpeed);
  delay(2000);
  motor1.stop();
  motor2.stop();
  motor3.stop();
  motor4.stop();
  motor5.stop();
  motor6.stop();
  motor7.stop();
  motor8.stop();
  delay(1000);
}
