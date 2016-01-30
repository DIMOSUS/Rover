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

#include "io.h"
#include "radio.h"
#include "camera.h"
#include "logic.h"

using namespace std;
	
void rxstring(string str)
{
	addcommand(str);
}

void rxfile(string str)
{
	cout << "Recive File: " << str << endl;
	SendText("Recive File: " + str);
}

int32_t main (void)
{
	radioinit();
	SendText("Hello!");
	regRXstrfile(&rxstring, &rxfile);
	ioinit();
	motorsinit();
	camerainit();
	logicinit();
	
	while(true)
	{
		string s;
		getline (cin, s);
		addcommand(s);
	}
	return 0;
}