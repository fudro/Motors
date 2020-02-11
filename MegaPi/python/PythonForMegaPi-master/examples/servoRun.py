from megapi import *

if __name__ == '__main__':
	bot = MegaPi()
	bot.start()
	while True:
		sleep(1);
		bot.servoRun(7,1,20);
		bot.servoRun(7,2,20);
		bot.servoRun(8,1,20);
		bot.servoRun(8,2,20);
		sleep(1);
		bot.servoRun(7,1,160);
		bot.servoRun(7,2,160);
		bot.servoRun(8,1,160);
		bot.servoRun(8,2,160);
