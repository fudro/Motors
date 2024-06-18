/*
* This program atttempts to ramp motor power using:
* - converter cable on Port A
* - 9V Technic Motor
*/

void rampUp() {
	/*
	//RAMPING THE MOTOR POWER DOES NOT WORK WHEN USING THE NXT MOTOR PORT AND A CONVERTER CABLE
	int targetSpeed = 100;
  int currentSpeed = 0;
  while(currentSpeed < targetSpeed) {
  	currentSpeed += 10;
  	motor[motorA] = currentSpeed;
  	//wait1Msec(1000);
  	sleep(1000);
	}
	*/
	motor[motorA] = 100;
	wait1Msec(5000);
	motor[motorA] = 0;
}

task main()
{
  rampUp();
}
