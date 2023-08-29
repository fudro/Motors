/*  
 * DESCRIPTION:
 * This sketch provides simple motor control for a two-motor robot chassis.
 * 
 * FUNTIONALITY:
 * Individual functions are provided for basic movement:
 * FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT
 * 
 * HARDWARE:
 * Arudino Uno
 * SparkFun Monster Moto Shield
 * 
 * REFERENCE:
 * https://www.sparkfun.com/products/retired/10182
 *
 * IMPORTANT:
 * Make sure the pin assignments are as follows: 
 * out1_A = 7, out1_B = 8, out2_A = 4, out2_B = 9
 * Otherwise the wheel rotation directions may not work as expected.
 */
 int out1_A = 7;
 int out1_B = 8;
 int out2_A = 4;
 int out2_B = 9;
 int pwm_1 = 5; //pins 5 and 6 have the highest pwm frequency on the Uno (980Hz)
 int pwm_2 = 6;
 int cs_1 = 2; //current sensing pins
 int cs_2 = 3;


void setup(){
  Serial.begin(9600);
  Serial.println("SparkFun MonsterMoto Test!");

  pinMode(out1_A, OUTPUT);
  pinMode(out1_B, OUTPUT);
  pinMode(out2_A, OUTPUT);
  pinMode(out2_B, OUTPUT);
  pinMode(pwm_1, OUTPUT);
  pinMode(pwm_2, OUTPUT);

  //initialize braked
  digitalWrite(out1_A, LOW);
  digitalWrite(out1_B, LOW);
  digitalWrite(out2_A, LOW);
  digitalWrite(out2_B, LOW);
}

 void loop() {
  moveForward(255); //enter pwm value between 0 and 255
  delay(3000);
  motorsOff();
  delay(2000);
  moveBackward(127);
  delay(3000);
  motorsOff();
  delay(2000);
  turnLeft(255, 255); //turnLeft(leftmotor-out1, rightmotor-out2)
  delay(3000);
  motorsOff();
  delay(2000);
  turnRight(255, 255);  //turnRight(leftmotor-out1, rightmotor-out2)
  delay(3000);
  motorsOff();
  delay(2000);
 }

 void moveBackward(int new_pwm) {
  analogWrite(pwm_1, new_pwm);
  analogWrite(pwm_2, new_pwm);
  digitalWrite(out1_A, LOW);
  digitalWrite(out1_B, HIGH);
  digitalWrite(out2_A, LOW);
  digitalWrite(out2_B, HIGH);
 }

 void moveForward(int new_pwm) {
  analogWrite(pwm_1, new_pwm);
  analogWrite(pwm_2, new_pwm);
  digitalWrite(out1_A, HIGH);
  digitalWrite(out1_B, LOW);
  digitalWrite(out2_A, HIGH);
  digitalWrite(out2_B, LOW);
 }

 void turnLeft(int left_pwm, int right_pwm) {
  analogWrite(pwm_1, left_pwm);
  analogWrite(pwm_2, right_pwm);
  digitalWrite(out1_A, LOW);
  digitalWrite(out1_B, HIGH);
  digitalWrite(out2_A, HIGH);
  digitalWrite(out2_B, LOW);
 }

 void turnRight(int left_pwm, int right_pwm) {
  analogWrite(pwm_1, left_pwm);
  analogWrite(pwm_2, right_pwm);
  digitalWrite(out1_A, HIGH);
  digitalWrite(out1_B, LOW);
  digitalWrite(out2_A, LOW);
  digitalWrite(out2_B, HIGH);
 }

 void motorsOff() {
  digitalWrite(out1_A, LOW);
  digitalWrite(out1_B, LOW);
  digitalWrite(out2_A, LOW);
  digitalWrite(out2_B, LOW);
  analogWrite(pwm_1, LOW);
  analogWrite(pwm_2, LOW);
 }
