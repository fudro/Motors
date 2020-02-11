from megapi import *

if __name__ == '__main__':
	bot = MegaPi()
	bot.start()
	while True:
		sleep(0.2);
		bot.digitalWrite(13,1);
		sleep(0.2);
		bot.digitalWrite(13,0);