from megapi import *

if __name__ == '__main__':
	bot = MegaPi()
	bot.start()
	t = 16;
	while True:
		sleep(0.06);
		bot.ledMatrixMessage(6,t,0,"Hello World");
		t-=1;
		if(t<-68):
			t = 16;
		