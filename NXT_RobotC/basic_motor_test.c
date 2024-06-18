/*--------------------------------------------------------------------------------------------------------*\
|*                                                                                                        *|
|*                                         - Moving  Forward -                                            *|
|*                                            ROBOTC on NXT                                               *|
|*                                                                                                        *|
|*  This program runs your robot forward for a designated amount of time.                                 *|
|*                                                                                                        *|
|*                                        ROBOT CONFIGURATION                                             *|
|*    NOTES:                                                                                              *|
|*    1)  To change the forward movement's speed, replace the two "100"'s with the desired speed number.  *|
|*    2)  To change the duration of the movement, replace the "4000" with the desired number of           *|
|*        milliseconds.                                                                                   *|
|*                                                                                                        *|
|*    MOTORS & SENSORS:                                                                                   *|
|*    [I/O Port]              [Name]              [Type]              [Description]                       *|
|*    Port C                  motorC              NXT                 Right motor                         *|
|*    Port B                  motorB              NXT                 Left motor                          *|
\*---------------------------------------------------------------------------------------------------4246-*/

task main()
{
   motor[motorA] = 100;		//motors can used default names or custom names set in the "Motor and Sensor Setup" Menu
	 //motor[motorB] = -100;  // Motor C is run at a power level of 100.
   //motor[motorC] = 100;   // Motor B is run at a power level of 100.
   wait1Msec(4000);       // The program waits 4000 milliseconds (4 seconds) before running further code
   //motor[motorA] = 100;		//Reduce Speed
   //wait1Msec(4000);
}
