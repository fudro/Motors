from megapi import *

if __name__ == '__main__':
	bot = MegaPi()
	bot.start()
	bot.motorRun(M1,0);
	sleep(1);
	while 1:
		sleep(1);
		bot.motorRun(M1,50);
		sleep(1);
		bot.motorRun(M1,0);
		sleep(1);
		bot.motorRun(M1,-50);
		sleep(1);
		bot.motorRun(M1,0);
