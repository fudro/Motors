/*************************
 * This program uses interrupts to track motor rotation when using the Adafruit Motorshield V1.
 * 
 * REFERENCE:
 * Information about the Adafruit Motorshield V1: https://learn.adafruit.com/adafruit-motor-shield/overview
 * Information about interrupts: https://youtu.be/oQQpAACa3ac
 * 
 * Useful Facts about Arduino Interrupts:
 * The Arduino Uno has two pins that can handle interrupts from external hardware (INT0 and INT1). These pins are mapped to digital pins 2 and 3 respectively.
 * When the external hardware sends a pulse to the interrupt pin, the Arduino will break out of the main loop and execute code related to the interrupt. Then the main loop resumes execution.
 * The code perfomed by an interrupt is called an Interrupt Service Routine, or ISR. An ISR cannot have any parameters, and they shouldn’t return anything.
 * For best results, ISRs should be short and fast pieces of code. There are also some additional restrictions on the type of code that can be handled by an ISR. 
 * For example, functions that depend on the internal Arduino timer (e.g. delay(), tone(), millis(), etc.) will not work in an ISR. 
 * For additional restrictions and best practices, see Arduino reference here: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 * In addition to external interrupts, the Arduino Uno can also handle internal interrupts triggered by an internal timer. To use these timer based interrupts, you must include the TimerOne Library.
 ************************/
#include "TimerOne.h"
#include <AFMotor.h>

//#include <Adafruit_MotorShield.h>

AF_DCMotor motor4(4);    //Create motor object and set motor port (1-4)
AF_DCMotor motor3(3);

// Create the motor shield object with the default I2C address
//Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Connect a DC motor to port M2
//Adafruit_DCMotor *Motor2 = AFMS.getMotor(2);
// Connect a DC motor to port M3
//Adafruit_DCMotor *Motor3 = AFMS.getMotor(3);

const byte M3 = 3;  //Assign motor port to a hardware interrupt pin.
const byte M4 = 2;
//IMPORTANT: Variables that need to be modified within an ISR MUST be declared as "volatile". See Arduino reference listed above.
volatile int encoder2 = 1; //count encoder disc "ticks" each time the encoder circuitry sends an interrupt pulse. 
volatile int encoder3 = 1;
float slots = 1.00; //Number of slots in encoder disc. Used to track rotation and calculate speed.

/**************INTERRUPT SERVICE ROUTINES******************/
//Motor2 pulse count ISR
void ISR_encoder2() {
    encoder2++;
}

//Motor3 pulse count ISR
void ISR_encoder3() {
  encoder3++;
}

//TimerOne ISR
void ISR_timerone() {
  Timer1.detachInterrupt();   //Stop Timer1 to allow time for serial print out
  Serial.print("Encoder 2: ");
  float rotation2 = (encoder2 / slots) * 1.00;   //Calculate RPM. Since Timer1 is set for 1 second, we get the number of slot ticks per second. Divide this by number of slots on the encoder disc to get number of revolutions per second. Then, multiply by 60 to get revolutions per minute.
  Serial.print(encoder2);
  Serial.print(" ");
    Serial.print("Encoder 3: ");
  float rotation3 = (encoder3 / slots) * 1.00;
  Serial.print(encoder3);
  Serial.println(" ");
//  encoder2 = 0;
//  encoder3 = 0;
  Timer1.attachInterrupt(ISR_timerone);
}
 /*************END INTERRUPT SERVICE ROUTINES*************/


void setup() {
  Serial.begin(9600);
  Serial.println("Encoder Test!");
//  AFMS.begin();  // create with the default frequency 1.6KHz
    // Setup motor on port M4
  motor4.setSpeed(255);    //Max motor speed is 255.
  motor4.run(RELEASE);
    // Setup motor on port M3
  motor3.setSpeed(255);
  motor3.run(RELEASE);
  
  pinMode(M4, INPUT_PULLUP);  //Activate pullup resistors on the interrupt pins
  pinMode(M3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(M4), ISR_encoder2, FALLING);   //Attach interrupt service routines to hardware interrupt pins and set trigger mode.
  attachInterrupt(digitalPinToInterrupt(M3), ISR_encoder3, FALLING);
  Timer1.initialize(1000000);   //Timer1 accepts parameter in microseconds. Set to one million for 1 second.
  Timer1.attachInterrupt(ISR_timerone);   //Enable the timer
}

void loop() {
  while(encoder2 < 200 && encoder2 != 0) {   //Set target encoder value
    motor4.run(FORWARD);       //Start motors in the desired directions.
    motor3.run(BACKWARD);
  }
  if(encoder2 != 0) {           //Once encoder threshold has been reached, make sure encoder has moved...
    motor4.run(BACKWARD);      //...then reverse direction of each motor to brake momentarily
    motor3.run(FORWARD);
    delay(10);
    motor4.run(RELEASE);
    motor3.run(RELEASE);
    encoder2 = 0;
    encoder3 = 0;
  }
}
