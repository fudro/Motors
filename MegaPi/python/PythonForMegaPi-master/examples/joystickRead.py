from megapi import *

axisX = 0;
axisY = 0;

def onReadX(v):
	global axisX;
	axisX = v;
	bot.joystickRead(port,2,onReadY);

def onReadY(v):
	global axisY;
	axisY = v;
	print(axisX,axisY);

if __name__ == '__main__':
	bot = MegaPi()
	bot.start()
	port = 6;
	while True:
		bot.joystickRead(port,1,onReadX);
		sleep(0.1);