#include "MeMegaPi.h"
#include <SoftwareSerial.h>
#include "TimerOne.h"

//Activate Motors
//#define DRIVE_MOTORS
//#define ARM_MOTOR

//Setup Debug Output
//#define DEBUG_DRIVE
#define DEBUG_ENCODER
//#define DEBUG_ARM
//#define DEBUG_DPAD

/*
int joystick_drive = 0;       //left joystick position in Y axis (FORWARD and BACKWARD)
int joystick_steer = 0;       //left joystick position in X axis (LEFT and RIGHT)
int last_drive_speed = 0;  //store last value for comparison - used to prevent huge jumps in motor current due to rapid joystick changes which can overload buck converter. 
int ramp_step = 7;            //maximum ammount the motor speed can change (up or down) per loop iteration
int joystick_arm = 0;         //right joystick position in Y axis (UP and DOWN). TODO:Direction is inverted - pull back to lift UP
int arm_scoop = 200;             //arm position for scooping up objects
int arm_extend = 250;         //best position to extend arm enough for clean release of object
int arm_up = 300;            //highest mechanically safe position for the arm (may vary depending on current attachment)
int arm_trimmer_low_threshold = 65;
int arm_trimmer_high_threshold = 170;
float drive_speed = 0.0;     //base speed component in the FORWARD/BACKWARD direction - derived from "joystick_drive"
float turn_speed = 0.0;      //speed adjustment value for left and right wheels while turning
float arm_speed = 0.0;        //speed of arm motor
float motor_speed_left = 0.0;   //calculated speed for LEFT wheels of robot (composed of drive_speed and turn_speed)
float motor_speed_right = 0.0;  //calculated speed for RIGHT wheels of robot (composed of drive_speed and turn_speed)
float motor_speed_arm = 0.0;
int relay_1 = 10;             //pin connected to relay input 1 (IN1)
int relay_2 = 11;             //pin connected to relay input 2 (IN2)
int relay_3 = 12;             //pin connected to relay input 3 (IN3)
int relay_4 = 13;             //pin connected to relay input 4 (IN4)
int arm_pot = A0;             //sensor feedback for arm position
int commandState = 0;     //Track if a command is currently being executed
*/

//Encoders
const byte MOTOR1 = 3;  //assign hardware interrupt pins to each motor
const byte MOTOR2 = 19;
//IMPORTANT: Variables that need to be modified within an ISR MUST be declared as "volatile". See Arduino reference listed above.
volatile int encoder1 = 0; //variables to count encoder disc "ticks" each time the encoder circuitry sends an interrupt pulse. 
volatile int encoder2 = 0;
uint8_t motorSpeed = 100;
long count = 0;
unsigned long time;
unsigned long last_time;

MeMegaPiDCMotor motor1(PORT1B);

/**************INTERRUPT SERVICE ROUTINES******************/
//Motor1 pulse count ISR
void ISR_encoder1() {
  encoder1++;
}

//Motor2 pulse count ISR
void ISR_encoder2() {
  encoder2++;
}

//TimerOne ISR
//void ISR_timerone() {
//  Timer1.detachInterrupt();   //Stop Timer1 to allow time for serial print out
//  #ifdef DEBUG_ENCODER
//  Serial.print("encoder1: ");
//  Serial.print(encoder1);
//  Serial.print("\t");
//  Serial.print("encoder2: ");
//  Serial.println(encoder2);
//  #endif
//  Timer1.attachInterrupt(ISR_timerone);
//}


void setup()
{
//    pinMode(MOTOR1, INPUT_PULLUP);
//    attachInterrupt(digitalPinToInterrupt(MOTOR1), blink, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR1), ISR_encoder1, RISING);   //Attach interrupt service routines to hardware interrupt pins and set trigger mode.
    attachInterrupt(digitalPinToInterrupt(MOTOR2), ISR_encoder2, CHANGE);
//    Timer1.initialize(100000);   //Timer1 accepts parameter in microseconds. Set to one million for 1 second.
//    Timer1.attachInterrupt(ISR_timerone);   //Enable the timer
    Serial.begin(115200);   
}

void loop()
{
    motor1.run(motorSpeed);   // value: between -255 and 255
//    delay(3000);
//    motor1.stop();
//    delay(5000);
    time = millis(); 
    if(time - last_time > 500)
    {
        blink();
        last_time = time;
   }
}

void blink()
{
  Serial.print("encoder1: ");
  Serial.print(encoder1);
  Serial.print("\t");
  Serial.print("encoder2: ");
  Serial.println(encoder2);
}
