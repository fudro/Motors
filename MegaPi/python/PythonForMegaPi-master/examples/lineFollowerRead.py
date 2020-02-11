from megapi import *

def onRead(v):
	print("line:"+str(v));

if __name__ == '__main__':
	bot = MegaPi()
	bot.start()
	while 1:
		sleep(0.1);
		bot.lineFollowerRead(6,onRead);