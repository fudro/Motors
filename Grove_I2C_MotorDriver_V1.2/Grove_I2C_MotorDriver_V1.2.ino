/*
 * DESCRIPTION:
 * This sketch demonstrates I2C movement control for a two-motor robot chassis.
 * 
 * HARDWARE:
 * Arduino Uno
 * Grove I2C Motor Driver V1.2
 * 
 * FUNCTIONALITY:
 * Use the physical switches on the motor driver to set the I2C address.
 * the sswitches are labeled 1-4. The switch is HIGH when the switch lever is "away" from the labeled number.
 * Make sure the address matches the value of I2CMotorDriverAdd defined below.
 * Connect I2C from the microcontroller to the Grove style connector on the motor driver.
 * 
 * REFERENCE:
 * https://wiki.seeedstudio.com/Grove-I2C_Motor_Driver_V1.2/
 */

#include <Wire.h>

#define MotorSpeedSet             0x82
#define PWMFrequenceSet           0x84
#define DirectionSet              0xaa
#define MotorSetA                 0xa1
#define MotorSetB                 0xa5
#define Nothing                   0x01

#define I2CMotorDriverAdd         0x0f   // Set the address of the I2CMotorDriver (Equivalent to 0b00001111, or "all swtiches HIGH")

void MotorSpeedSetAB(unsigned char MotorSpeedA , unsigned char MotorSpeedB)  {  //set motor speed between 0 and 100
  MotorSpeedA=map(MotorSpeedA,0,100,0,255);
  MotorSpeedB=map(MotorSpeedB,0,100,0,255);
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(MotorSpeedSet);        // set pwm header 
  Wire.write(MotorSpeedA);              // send pwma 
  Wire.write(MotorSpeedB);              // send pwmb    
  Wire.endTransmission();    // stop transmitting
}

void MotorPWMFrequenceSet(unsigned char Frequence)  {    // the prescale frequency of PWM, 0x03 default.
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(PWMFrequenceSet);        // set frequence header
  Wire.write(Frequence);              //  send frequence 
  Wire.write(Nothing);              //  need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();    // stop transmitting
}

/*
 * MotorDirectionSet(0b1010);  //"0b1010" defines the output polarity, "10" means the M+ is "positive" while the M- is "negative"
 */
void MotorDirectionSet(unsigned char Direction)  {     //  Adjust the direction of the motors 0b0000 I4 I3 I2 I1
  Wire.beginTransmission(I2CMotorDriverAdd); // transmit to device I2CMotorDriverAdd
  Wire.write(DirectionSet);        // Direction control header
  Wire.write(Direction);              // send direction control information
  Wire.write(Nothing);              // need to send this byte as the third byte(no meaning)  
  Wire.endTransmission();    // stop transmitting 
}

void MotorDriectionAndSpeedSet(unsigned char Direction,unsigned char MotorSpeedA,unsigned char MotorSpeedB)  {  //you can adjust the driection and speed together
  MotorDirectionSet(Direction);
  MotorSpeedSetAB(MotorSpeedA,MotorSpeedB);  
}

void setup()  {
  Wire.begin(); // join i2c bus (address optional for master)
  delayMicroseconds(10000); //wait for motor driver to initialization
}

void loop()  {
  while(1)  {
    MotorSpeedSetAB(100,100);   //MotorA is the first parameter and corresponds to the left motor. MotorB is the second parameter and corresponds to the right motor
    delay(10);                  //this delay needed after setting speed
    MotorDirectionSet(0b0101);  //0b1010  Move in the Forward direction 
    delay(3000); 
    MotorSpeedSetAB(0,0);       //Stop motors
    delay(2000);
    MotorSpeedSetAB(50,50);     //change speed
    delay(10);
    MotorDirectionSet(0b1010);  //0b0101  Move in the Backward direction
    delay(3000);
    MotorSpeedSetAB(0,0);       //Stop motors
    delay(2000);
  }
}
