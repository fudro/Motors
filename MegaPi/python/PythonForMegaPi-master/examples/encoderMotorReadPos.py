from megapi import *

def onRead(level):
	print("Encoder motor speed Value:%f" %level);

if __name__ == '__main__':
	bot = MegaPi()
	bot.start()
	bot.encoderMotorRun(4,100);
	sleep(1);
	while 1:
		bot.encoderMotorPosition(4,onRead);
		sleep(0.2);