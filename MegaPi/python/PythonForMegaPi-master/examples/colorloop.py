from megapi import *
import random
import math

if __name__ == '__main__':
	bot = MegaPi()
	bot.start()
	j = 0.0;
	f = 0.0;
	k = 0.0;
	port = 8;
	slot = 2;
	while True:
		for i in range(5):
			j += random.random();
			f += random.random();
			k += random.random();
			red = 32*(1.0+math.sin(((i/2.0)+(j/4.0))));
			green = 32*(1.0+math.sin(((i/1.0)+(f/9.0)+2.1)));
			blue = 32*(1.0+math.sin(((i/3.0)+(k/14.0)+4.2)));
			bot.rgbledDisplay(port,slot,i,red,green,blue);
			sleep(1);
		pass
