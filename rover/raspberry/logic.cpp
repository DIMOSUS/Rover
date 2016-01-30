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
#include "logic.h"
#include "radio.h"
#include "io.h"
#include "camera.h"

#define byte uint8_t

using namespace std;

enum state_type { _stand=1, _irscan} logic_state;
queue<string> command;
pthread_mutex_t logic_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_t logicthread;

uint32_t allDistance = 0;

void colorOut(string s, int32_t color, bool ln = false)
{
	cout << "\033[1;" << color << "m" << s << "\033[0m";
	if (ln) cout << endl;
}

void* logicProc(void *arg)
{
	colorOut("Robot ready", 32, 1);

	while(true)
	{
		string c;
		pthread_mutex_lock(&logic_mutex);
			if(!command.empty())
			{
				c = command.front();
				command.pop();
			}
		pthread_mutex_unlock(&logic_mutex);
		if (c.length() != 0)
		{
			//cout << "<" << c << ">" << endl;
			istringstream iss(c);
			string comm;
			int32_t v0, v1, v2, v3;
			iss >> comm;
			iss >> v0;
			iss >> v1;
			iss >> v2;
			iss >> v3;
			
			if (comm == "move")
			{
				SendText(c);
				move(v0, v1);
				allDistance += v0;
				while (!moveFin()) {delay(200);}
			}
			else if (comm == "rot")
			{
				SendText(c);
				rotate_cmp(v0, 200);
				while (!moveFin()) {delay(200);}
			}
			else if (comm == "servo")
			{
				SendText(c);
				servo(v0, v1);
			}
			else if (comm == "shot")
			{
				SendText(c);
				string f = takeShot(v0 > 0);
				SendFile(f);
			}
			else if (comm == "irshot")
			{
				SendText(c);
				string f = takeIRShot();
				SendFile(f);
			}
			else if (comm == "panoram")
			{
				SendText(c);
				string f = takePanoram(v0 > 0);
				SendFile(f);
			}
			else if (comm == "status")
			{
				SendText(c);
				ostringstream str;
				str << "T " << Temp << "; Pr " << Pres << "; Hm " << Humid << "; Mag " << Mag << "; Lux " << lux() << "; MLX " << mlxTemp() << "; Bat " << vbat() << "v/" << cbat(Temp) << "%; Sat " << Satellites << "; Dst " << dist();
				SendText(str.str());
			}
			else if (comm == "ir")
			{
				SendText(c);
				setirt(v0);
			}
			else if (comm == "dist")
			{
				SendText(c);
				ostringstream str;
				str << allDistance << "mm";
				SendText(str.str());
			}
			else
			{
				SendText("Fail command: " + c);
			}
		}
		delay(100);
	}
	return 0;
}

void logicinit(void)
{
	pthread_create(&logicthread, NULL, logicProc, NULL);
}

void addcommand(string c)
{
	if (c.length() == 0) return;
	
	if (c == "clean")
	{
		SendText(c);
		pthread_mutex_lock(&logic_mutex);
			while(!command.empty()) command.pop();
		pthread_mutex_unlock(&logic_mutex);
		logic_state = _stand;
		return;
	}
	if (c == "reboot")
	{
		SendText(c);
		system("reboot");
		return;
	}
	if (c == "halt")
	{
		SendText(c);
		system("halt");
		exit(0);
		return;
	}
	if (c.find("lag") != string::npos)
	{
		SendText(c);
		string comm;
		int32_t param = 0;
		istringstream iss(c);
		iss >> comm;
		iss >> param;
		
		setLag(param);
		return;
	}
	
	pthread_mutex_lock(&logic_mutex);
		command.push(c);
	pthread_mutex_unlock(&logic_mutex);
}