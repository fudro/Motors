//  Demo function:The application method to drive the DC motor.
//  Author:Loovee (luweicong@seeed.cc)
//  2016-3-11
 
#include "MotorDriver.h"

// new motor shield library
// by loovee
// 2016-3-11
#ifndef __MOTORDRIVER_H__
#define __MOTORDRIVER_H__

#include <Arduino.h>
/******Pins definitions*************/
#define MOTORSHIELD_IN1  8
#define MOTORSHIELD_IN2 11
#define MOTORSHIELD_IN3 12
#define MOTORSHIELD_IN4 13
#define SPEEDPIN_A    9
#define SPEEDPIN_B    10
/**************Motor ID**********************/
#define MOTORA      0
#define MOTORB      1

#define SPD_BUFF_STEPS        100

/**Class for Motor Shield**/
class MotorDriver {

  private:
    int speed0;
    int speed1;

    // motor pin numbers:
    int motor_pin_a_plus;
    int motor_pin_a_minus;
    int motor_pin_b_plus;
    int motor_pin_b_minus;

  public:

    void begin();

    // motor_num: 0, 1
    // speed: 0~100
    void speed(int motor_id, int _speed);
    void stop(unsigned char motor_id);
    void brake(unsigned char motor_id);

};


#endif


//  Author:Frankie.Chu
//  Date:20 November, 2012
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
//  Modified record:
//
/*******************************************************************************/
//#include "MotorDriver.h"
//#include "seeed_pwm.h"

#define PWM_FREQ    10000

void MotorDriver::begin() {
    speed0 = 0;
    speed1 = 0;
    pinMode(MOTORSHIELD_IN1, OUTPUT);
    pinMode(MOTORSHIELD_IN2, OUTPUT);
    pinMode(MOTORSHIELD_IN3, OUTPUT);
    pinMode(MOTORSHIELD_IN4, OUTPUT);
    digitalWrite(MOTORSHIELD_IN1, LOW);
    digitalWrite(MOTORSHIELD_IN2, LOW);
    digitalWrite(MOTORSHIELD_IN3, LOW);
    digitalWrite(MOTORSHIELD_IN4, LOW);
    PWM.init();
}

void MotorDriver::stop(unsigned char motor_id) {
    speed(motor_id, 0);
}

void MotorDriver::brake(unsigned char motor_id) {
    switch (motor_id) {
        case 0:
            //move(0, 0);
            digitalWrite(MOTORSHIELD_IN1, HIGH);
            digitalWrite(MOTORSHIELD_IN2, HIGH);
            speed0 = 0;

            break;

        case 1:
            //move(1, 0);
            digitalWrite(MOTORSHIELD_IN3, HIGH);
            digitalWrite(MOTORSHIELD_IN4, HIGH);
            speed1 = 0;

            break;

        default:;
    }
}

void MotorDriver::speed(int motor_id, int _speed) {
    if (motor_id < 0 || motor_id > 1) {
        return;
    }

    _speed = _speed < -100 ? -100 : _speed;
    _speed = _speed > 100 ? 100 : _speed;

    if (motor_id == 0 && speed0 == _speed) {
        if (speed0 == _speed) {
            return ;
        } else {
            speed0 = _speed;
        }
    } else if (motor_id == 1 && speed1 == _speed) {
        if (speed1 == _speed) {
            return ;
        } else {
            speed1 = _speed;
        }
    }
    switch (motor_id) {
        case 0:

            if (_speed > 0) {
                digitalWrite(MOTORSHIELD_IN1, HIGH);
                digitalWrite(MOTORSHIELD_IN2, LOW);
                PWM.setPwm(9, _speed, PWM_FREQ);
            } else if (_speed < 0) {
                digitalWrite(MOTORSHIELD_IN1, LOW);
                digitalWrite(MOTORSHIELD_IN2, HIGH);
                PWM.setPwm(9, 0 - _speed, PWM_FREQ);
            } else {
                digitalWrite(MOTORSHIELD_IN1, LOW);
                digitalWrite(MOTORSHIELD_IN2, LOW);
                PWM.setPwm(9, _speed, PWM_FREQ);
            }

            break;

        case 1:

            if (_speed > 0) {
                digitalWrite(MOTORSHIELD_IN3, HIGH);
                digitalWrite(MOTORSHIELD_IN4, LOW);
                PWM.setPwm(10, _speed, PWM_FREQ);
            } else if (_speed < 0) {
                digitalWrite(MOTORSHIELD_IN3, LOW);
                digitalWrite(MOTORSHIELD_IN4, HIGH);
                PWM.setPwm(10, 0 - _speed, PWM_FREQ);
            } else {
                digitalWrite(MOTORSHIELD_IN3, LOW);
                digitalWrite(MOTORSHIELD_IN4, LOW);
                PWM.setPwm(10, _speed, PWM_FREQ);
            }
            break;

        default:;
    }
}
 
MotorDriver motor;
 
void setup()
{
    // initialize
    motor.begin();
}
 
void loop()
{
    motor.speed(0, 100);            // set motor0 to speed 100
    motor.speed(1, -100);
    delay(3000);
    motor.brake(0);                 // brake
    motor.brake(1);
    delay(2000);
    motor.speed(0, -100);           // set motor0 to speed -100
    motor.speed(1, 100);
    delay(3000);
    motor.stop(0);                  // stop
    motor.stop(1);
    delay(2000);
}
// END FILE
