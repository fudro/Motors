/************************************
 * This program demonstrates bi-directional push button control of a DC Motor using the Adafruit Motorshield V1.
 * A potentiometer value is captured and displayed to track the angular position of the motor-controlled joint.
 * 
 * REFERENCE:
 * https://learn.adafruit.com/adafruit-motor-shield/overview
 */

#include <AFMotor.h>

AF_DCMotor motor(4);    //Create motor object and set motor port (1-4)

const int buttonCCW = 14;   //Set buttonPin to the digital equivalinet of analog pin A0
const int buttonCW = 15;   //Set buttonPin to the digital equivalinet of analog pin A0
bool      buttonStateCCW = false;
bool      buttonStateCW = false;
int       potValue      = 0;

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("DC Motor Button Test!");

  // turn on motor
  motor.setSpeed(200);  //Set default motor speed. Speeds allowed from 0 to 255.
  motor.run(RELEASE);   //Reset the motor (clear any previous commands) by "releasing" it.

  pinMode (buttonCCW, INPUT);
  pinMode (buttonCW, INPUT);
  pinMode (A2, INPUT);
}

void loop() {
  if (digitalRead(buttonCCW) == HIGH && buttonStateCCW == false) {
    buttonStateCCW = true;
    motor.run(FORWARD);
    Serial.println ("CCW!");
  }
  else if (digitalRead(buttonCCW) == LOW && buttonStateCCW == true){
    buttonStateCCW = false;
    motor.run(BACKWARD);
    delay (10);
    motor.run(RELEASE);
    Serial.println ("OFF!");
    delay(1000);
  }
  else if (digitalRead(buttonCW) == HIGH && buttonStateCW == false) {
    buttonStateCW = true;
    motor.run(BACKWARD);
    Serial.println ("CW!");
  }
  else if (digitalRead(buttonCW) == LOW && buttonStateCW == true){
    buttonStateCW = false;
    motor.run(FORWARD);
    delay (10);
    motor.run(RELEASE);
    Serial.println ("OFF!");
    delay(1000);
  }
  if (buttonStateCW == true || buttonStateCCW == true) {
    potValue = analogRead(A2);
    Serial.println(potValue);
  }
}
