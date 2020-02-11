from megapi import *

if __name__ == '__main__':
	bot = MegaPi()
	bot.start()
	t = 0.0;
	while True:
		bot.sevenSegmentDisplay(6,t);
		t+=0.13;
		if t>100:
			t = 0;
		sleep(0.1);