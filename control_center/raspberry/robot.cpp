#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <cstdarg>
#include <cmath> 
#include <fstream>
#include <iostream>
#include <iomanip> 
#include <sstream>
#include <queue>
#include <unistd.h>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/file.h>
#include <pthread.h> 
#include <termios.h>

#include "wiringPi.h"
#include "radio.h"

using namespace std;

void colorOut(string s, int32_t color, bool ln = false)
{
	cout << "\033[1;" << color << "m" << s << "\033[0m";
	if (ln) cout << endl;
}

void rxstring(string str)
{
	colorOut(">" + str, 32, 1);
}

void rxfile(string str)
{
	colorOut("File>" + str, 32, 1);
}

int main (void)
{
	radioinit();
	SendText("Hello!");
	//SendFile
	regRXstrfile(&rxstring, &rxfile);
	
	while(true)
	{
		string s;
		getline (cin, s);
		if (s == "update")
		{
			colorOut(s, 33, 1);
			if (SendFile("../robot/robot", "robot"))
			{
				colorOut("Send ok", 33, 1);
			}
			SendText("reboot");
		}
		else SendText(s);
		//addcommand(s);
	}
	return 0;
}