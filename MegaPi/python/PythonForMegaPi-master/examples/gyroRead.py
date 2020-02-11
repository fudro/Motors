from megapi import *

def onRead(level):
	print("Gyro Value:%f" %level);

if __name__ == '__main__':
	bot = MegaPi()
	bot.start()
	while 1:
		sleep(0.1);
		bot.gyroRead(0,1,onRead);