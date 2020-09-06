/*
 * DESCRIPTION:
 * This sketch provides uses a SeeedStudio motor controller to provide simple movement
 * of a two-motor robot chassis.
 * 
 * HARDWARE:
 * Arduino Uno
 * SeeedStudio 4A MotorShield
 * Four-wheel toy robot chassis using two motors (one motor for each side, left and right)
 * 
 * FUNCTIONALITY:
 * "Speed pins" control the PWM of the corresponding OUT channel.
 * SPEED_LEFT controls: OUT1, OUT2
 * SPEED_RIGHT controls: OUT3, OUT4
 * 
 * "Out pins" control the logic (HIGH or LOW) of the output ports that control the motors.
 * By setting the logic of the OUT pins, the polarity and thus directionality of the motor 
 * is changed.
 * 
 * REFERENCE:
 * Logic Table (located in Schematic, Ports section): https://wiki.seeedstudio.com/4A_Motor_Shield/
 */

int SPEED_LEFT = 9;   //PWM pin that controls the speed of the left motor.
int SPEED_RIGHT = 10; //PWM pin that controls the speed of the left motor.
int OUT1 = 5;
int OUT2 = 6;
int OUT3 = 7;
int OUT4 = 8;
int speed_left = 255;
int speed_right = 255;

void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(SPEED_LEFT, OUTPUT);
  pinMode(SPEED_RIGHT, OUTPUT);
  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(OUT3, OUTPUT);
  pinMode(OUT4, OUTPUT);
  Serial.println("Starting 4A MotorShield Test!");
  Serial.println("Setting Speed!");
  Serial.println("");
  analogWrite(SPEED_LEFT, speed_left);  //Left Wheel
  analogWrite(SPEED_RIGHT, speed_right);  //Right Wheel 
}

void loop() {
  Serial.println("Stopped");
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
  digitalWrite(OUT3, LOW);
  digitalWrite(OUT4, LOW);
  delay(2000);
  
  Serial.println("Forward");
  digitalWrite(OUT1, HIGH);
  digitalWrite(OUT2, LOW);
  digitalWrite(OUT3, HIGH);
  digitalWrite(OUT4, LOW);
  delay(3000);

  Serial.println("Stopped");
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
  digitalWrite(OUT3, LOW);
  digitalWrite(OUT4, LOW);
  delay(2000);

  Serial.println("Backward");
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, HIGH);
  digitalWrite(OUT3, LOW);
  digitalWrite(OUT4, HIGH);
  delay(3000);

  Serial.println("Stopped");
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
  digitalWrite(OUT3, LOW);
  digitalWrite(OUT4, LOW);
  delay(2000);

  Serial.println("Turn Left");
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, HIGH);
  digitalWrite(OUT3, HIGH);
  digitalWrite(OUT4, LOW);
  delay(3000);

  Serial.println("Stopped");
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
  digitalWrite(OUT3, LOW);
  digitalWrite(OUT4, LOW);
  delay(2000);

  Serial.println("Turn Right");
  digitalWrite(OUT1, HIGH);
  digitalWrite(OUT2, LOW);
  digitalWrite(OUT3, LOW);
  digitalWrite(OUT4, HIGH);
  delay(3000);

  Serial.println("Stopped");
  digitalWrite(OUT1, LOW);
  digitalWrite(OUT2, LOW);
  digitalWrite(OUT3, LOW);
  digitalWrite(OUT4, LOW);
  delay(2000);
}
