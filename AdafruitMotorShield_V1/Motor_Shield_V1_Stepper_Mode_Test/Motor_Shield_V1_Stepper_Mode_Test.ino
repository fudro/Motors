/*
 * This program uses an Adafruit Motorshield V1 and a 28BYJ-48 unipolar stepper motor.
 * 
 * OPERATION:
 * a) Press the button connected A0 to test different modes of motor control.
 * b) Remember to manually uncomment the mode you want to test.
 * c) Each button press will use the uncommented mode to rotate the motor CW and CCW one time.
 * d) Press the button again to repeat the action.
 */

#include <AFMotor.h>

//REMEMBER TO SET THE CORRECT MOTOR PORT YOU WISH TO USE:
AF_Stepper motor(32, 2);

int stepperSpeed = 1000;
long stepCount = 1000;
int buttonActive = 0; 

void setup() {
  pinMode(A0, INPUT_PULLUP);  //Buton connects to GND upon press, so set default to be pulled up.
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Stepper Mode Test!");

  Serial.print("TotalSteps: ");
  Serial.println(stepCount);

  motor.setSpeed(stepperSpeed);  // "Motor internal rotor rpm"   
  Serial.print("Speed: ");
  Serial.println(stepperSpeed);

  motor.step(100, FORWARD, SINGLE); 
  motor.release();
  delay(1000);
}

void loop() {
  if(buttonActive == 0 && digitalRead(A0) == LOW) { //Check for button press on analog pin A0
    buttonActive = 1;
/*
 * BE SURE TO UNCOMMENT THE MODE YOU WANT TO TEST
 */
//    Serial.println("Single Mode");
//    motor.step(1000, FORWARD, SINGLE); 
//    motor.step(1000, BACKWARD, SINGLE); 
  
    Serial.println("Double Mode");
    motor.step(stepCount, FORWARD, DOUBLE);   //This mode seems to be best for strength and speed
    motor.step(stepCount, BACKWARD, DOUBLE);
  
//    Serial.println("Interleave Mode");
//    motor.step(1000, FORWARD, INTERLEAVE); //This mode performs about half the travel
//    motor.step(1000, BACKWARD, INTERLEAVE); 

//    Serial.println("Microstep Mode");
//    motor.step(1000, FORWARD, MICROSTEP); 
//    motor.step(1000, BACKWARD, MICROSTEP); 
  }
  else if(buttonActive == 1 && digitalRead(A0) == HIGH) {   //Check for button release
    buttonActive = 0;  //Reset button value for additional button presses
    motor.release();
    Serial.println("Button Ready!");
  }
}
