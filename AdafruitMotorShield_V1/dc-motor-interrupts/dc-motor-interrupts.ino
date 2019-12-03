/*************************
 * This program demonstrates basic encoder based otoro control of a DC motor.
 * The hardware setup uses an Arduino Uno and an Adafruit Motorshield V1.
 * Motor rotation is tracked using a custom encoder module and hardware interrupt pins on the Arduino.
 * 
 * HARDWARE SETUP:
 * 1) Power sensor module using 3.3v
 * 2) Connect the digital (D0) pin of the sensor module to the Arduino hardware interrupt pin.
 * 3) Set pinMode of hardware interrupt pins to INPUT (do NOT use INPUT-PULLUP)
 * 4) Set the interrupt mode to HIGH (This setting provides the best results and will activate the interrupt pin whenever the sensor LED turns ON or OFF).
 * 
 * REFERENCE:
 * Documentation for the Adafruit Motorshield V1: https://learn.adafruit.com/adafruit-motor-shield/overview
 * Documentation for Arduino interrrupts: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 * Video tutorial about Arduino interrupts: https://youtu.be/oQQpAACa3ac
 * Sensor module used to create custom encoder: https://www.amazon.com/gp/product/B01MRELRS1/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1
 * 
 ************************/

 
#include "TimerOne.h"   //This library is used to control the timer-based interrupts.
#include <AFMotor.h>

AF_DCMotor motor4(4);    //Create motor object and set corresponding motor port (1-4)
AF_DCMotor motor3(3);

const byte M3 = 3;  //Assign motor port to a hardware interrupt pin.
const byte M4 = 2;
//IMPORTANT: Variables that need to be modified within an ISR MUST be declared as "volatile". See Arduino reference listed above.
volatile int encoder2 = 1; //count encoder disc "ticks" for each pulse of the the hardware interrupt pin.
volatile int encoder3 = 1;
float slots = 1.00; //Number of slots in encoder disc. Used to track rotation and calculate speed.

/**************INTERRUPT SERVICE ROUTINES******************/
void ISR_encoder2() {   //Track hardware interrupts on pin 2
  encoder2++;
  Serial.println(encoder2);
}

void ISR_encoder3() {   //Track hardware interrupts on pin 3
  encoder3++;
}

//A timer-based interrupt can be used to display encoder stats (OPTIONAL)
void ISR_timerone() {
  Timer1.detachInterrupt();   //Stop Timer1 to allow time for serial print out
  Serial.print("Encoder 2: ");
  float rotation2 = (encoder2 / slots) * 1.00;   //Calculate RPM: Since the timer interrupt is set for 1 second, the encoder2 variable gives the number of ticks per second. Divide this by number of slots on the encoder disc to get number of revolutions per second. Then, multiply by 60 to get revolutions per minute.
  Serial.print(encoder2);
  Serial.print(" ");
    Serial.print("Encoder 3: ");
  float rotation3 = (encoder3 / slots) * 1.00;
  Serial.print(encoder3);
  Serial.println(" ");
  Timer1.attachInterrupt(ISR_timerone);   //Restart timer after printing is complete.
}
 /*************END INTERRUPT SERVICE ROUTINES*************/


void setup() {
  Serial.begin(9600);
  Serial.println("Encoder Test!");

  // Setup motor on port M4
  motor4.setSpeed(255);   //Max motor speed is 255.
  motor4.run(RELEASE);    //Release motor to make it ready for new incoming commands
  // Setup motor on port M3
  motor3.setSpeed(255);
  motor3.run(RELEASE);

  //Setup hardware interrupt pins
  pinMode(M4, INPUT);  //Activate pullup resistors on the hardware interrupt pins
  pinMode(M3, INPUT_PULLUP);

  //Setup hardware interrupt ISRs
  attachInterrupt(digitalPinToInterrupt(M4), ISR_encoder2, HIGH);   //Attach interrupt service routines to hardware interrupt pins and set trigger mode.
  attachInterrupt(digitalPinToInterrupt(M3), ISR_encoder3, HIGH);

  //Setup timer-based interrupts (NOTE: Comment out these statements if the timer-based interrupt is not required.)
//  Timer1.initialize(1000000);   //Set timer for timer-based interrupt. Timer1 accepts parameter in microseconds. Set to one million for 1 second.
//  Timer1.attachInterrupt(ISR_timerone);   //Enable the timer-based interrupt by attaching the interrupt service routine.
}

void loop() {
  while(encoder2 < 200 && encoder2 != 0) {   //Set target encoder value
    motor4.run(BACKWARD);       //Start motors in the desired directions.
    motor3.run(BACKWARD);
  }
  if(encoder2 != 0) {           //Once encoder threshold has been reached, make sure encoder has moved...
    motor4.run(FORWARD);      //...then reverse direction of each motor for a brief amount of time to brake
    motor3.run(FORWARD);
    delay(15);                 
    /*NOTE: The delay value is the braking delay and may need to be adjusted to get best results. 
     * Too little time will allow the motors to coast after braking while too much time will cause the motors to reverse after braking. 
     *Since the encoder doesn't care which direction the motor is turning, both sceanrios can cause the encoder value to increase slightly beyond the desired stop value.)
     */
    motor4.run(RELEASE);
    motor3.run(RELEASE);
    Serial.println("STOP!");
    delay(1000);              //Pause to give motors enough time to settle and truly stop before resetting encoder values
    encoder2 = 0;
    encoder3 = 0;
  }
}
