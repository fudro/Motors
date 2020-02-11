from megapi import *

def onForwardFinish(slot):
	sleep(0.4);
	bot.stepperMotorMoveTo(slot,3000,-6000,onBackwardFinish);

def onBackwardFinish(slot):
	sleep(0.4);
	bot.stepperMotorMoveTo(slot,3000,6000,onForwardFinish);

if __name__ == '__main__':
	bot = MegaPi()
	bot.start()
	sleep(1);
	onForwardFinish(1);
	while 1:
		continue;