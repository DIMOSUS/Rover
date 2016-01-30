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
#include <limits.h>

#include "wiringPi.h"
#include "wiringPiI2C.h"
#include "io.h"
#include "PID.h"

#define PI 3.1415926535897932384626433832795

using namespace std;

pthread_t motorsthread;

//servos

#define step 40
#define inittick 8
volatile int32_t s_setval[3] = {1480, 1480, 1545};
volatile int32_t s_curval[3] = {s_setval[0]+20, s_setval[1]+20, s_setval[2]+20};
volatile double s_deg[3] = {90, 90, 90};//
static int32_t sInv[3] = {1,1,1};

//motors
volatile enum TmoveState { none, straight, rotation, rot_cmp } moveState = none;
pthread_mutex_t l_mutex = PTHREAD_MUTEX_INITIALIZER, r_mutex = PTHREAD_MUTEX_INITIALIZER;
#define speedLerp 0.6
#define Lstep 3.12
#define Rstep 3.12
#define braktime 0.5
volatile uint32_t pretmL, pretmR;//previous interrupt time
volatile int32_t distL = 0, distR = 0;
volatile int32_t dirL = 1, dirR = 1;
volatile double speedL = 0, speedR = 0, tempPreL = 0, tempPreR = 0, lps = 0, rps = 0;
double vall = 0, valr = 0;
uint32_t timer = 0;
TPID lPID, rPID;
//
volatile double setDist = 0, setSpeed = 0, setAngle = 0;

template <typename T>
int32_t sign(T Val) {
  //if (Val == 0.)  return 0;
  if (Val < 0.)  return -1;
  else return 1;
}

uint32_t GetDelta(uint32_t preTime)
{
	uint32_t m = micros();
	uint32_t delta = 0;
	if (m < preTime) delta =  (UINT_MAX-preTime) + m;
	else  delta = m - preTime;
	return delta;
}

void myInterrupt0 (void) {
pthread_mutex_lock(&l_mutex);
	uint32_t delta = GetDelta(pretmL);
	if (delta > 4000)
	{
		//cout << delta << endl;
		distL += dirL;
		pretmL = micros();
		double tempSpeed = (((double)dirL)*Lstep*1000000)/((double)delta);//mm/s
		
		if(abs(tempPreL) > 10 && abs(tempSpeed) > 10)
		{
			//speedL = KL.KalmanCorrect((tempPreL+tempSpeed)*0.5);
			lps = lps*speedLerp + (1-speedLerp)*(tempPreL+tempSpeed)*0.5;
		}
		//cout << delta << " "  << (tempPreR+tempSpeed)*0.5 << endl;
		
		tempPreL = tempSpeed;
	}
	//cout << "L " << delta << " " << distL << endl;
	
pthread_mutex_unlock(&l_mutex);
}

void myInterrupt2 (void) {
pthread_mutex_lock(&r_mutex);
	uint32_t delta = GetDelta(pretmR);
	
	if (delta > 4000)
	{
		//cout << delta << endl;
		distR += dirR;
		pretmR = micros();
		double tempSpeed = (((double)dirR)*Rstep*1000000)/((double)delta);//mm/s
		
		if(abs(tempPreR) > 10 && abs(tempSpeed) > 10)
		{
			//speedR = KR.KalmanCorrect((tempPreR+tempSpeed)*0.5);
			rps = rps*speedLerp + (1-speedLerp)*(tempPreR+tempSpeed)*0.5;
		}
		//cout << delta << " "  << (tempPreR+tempSpeed)*0.5 << endl;
		
		tempPreR = tempSpeed;
	}
	
	//cout << "R " << delta << " " << distR << endl;
	
pthread_mutex_unlock(&r_mutex);
}

int32_t servomap(double angle, double amin, double amax, int32_t imin, int32_t imax)
{
	angle = max(amin, angle);
	angle = min(amax, angle);
	angle = (angle-amin)/(amax - amin);
	return imin + (imax-imin)*angle;
}

double invservomap(int32_t angle, double amin, double amax, int32_t imin, int32_t imax)
{
	double val = ((double)(angle-imin))/((double)(imax-imin));
	return amin + val*(amax - amin);
}

bool servo(int32_t i, double angle)
{
	if (i < 0 || i > 2) return true;
	int32_t val;
	if (i == 0)
		val = servomap(angle, 20, 160, 732, 2370);
	else if (i == 1)
		val = servomap(angle, 20, 160, 2340, 712);
	else
		val = servomap(angle, 10, 170, 760, 2490);
	s_setval[i] = val;	
	//cout << i << " " << val << " " << s_deg[i] << endl;
	return abs(angle - s_deg[i]) < 0.5;
}

double getServo(int32_t i)
{
	return s_deg[i];
}

double GetDeltaAngle(double a1, double a2)
{
	double delta = a2-a1;
	if (delta >= PI)
	{
		delta = delta - PI*2;
	}
	else if (delta <= -PI)
	{
		delta = delta + PI*2;
	}
	return delta;
}

bool moveFin()
{
	if (moveState == none) return true;
	return false;
}

void move(double dist, double speed)
{
	if (abs(dist) > 10 && speed > 30)
	{
		distL = 0; distR = 0;
		setDist = dist;
		setSpeed = speed*sign(dist);
		moveState = straight;
	}
	else
		moveState = none;
	vall = 0; valr = 0;
}

void rotate(double dist, double speed)
{
	if (abs(dist) > 10 && speed > 30)
	{
		distL = 0; distR = 0;
		setDist = dist;
		setSpeed = speed*sign(dist);
		moveState = rotation;
	}
	else
		moveState = none;
	vall = 0; valr = 0;
}

void rotate_cmp(double angle, double speed)
{
	if (abs(angle) > 1  && abs(angle) <= 180 && speed > 30)
	{
		distL = 0; distR = 0;
		setAngle = yaw - angle/180.0*PI;
		
		if (setAngle >= PI)
			setAngle = setAngle - PI*2;
		else if (setAngle <= -PI)
			setAngle = setAngle + PI*2;
		
		//setDist = GetDeltaAngle(yaw, setAngle);
		setSpeed = speed;
		moveState = rot_cmp;
	}
	else
		moveState = none;
	vall = 0; valr = 0;
}

void* motorsProc(void *arg)
{
	while(true)
	{
		{//servos
			for (int32_t i = 0; i < 3; i++)
			{
				if (abs (s_setval[i] - s_curval[i]) > 4)
				{
					int32_t delta = s_setval[i] - s_curval[i];
					delta = min(step, delta);
					delta = max(-step, delta);
					s_curval[i] += delta*sInv[i];
					rawservo(i, s_curval[i]);
					
					if (i == 0)
						s_deg[i] = invservomap(s_curval[i], 20, 160, 732, 2370);
					else if (i == 1)
						s_deg[i] = invservomap(s_curval[i], 20, 160, 2340, 712);
					else
						s_deg[i] = invservomap(s_curval[i], 10, 170, 760, 2490);
					//cout << "grad: " << i << " " << s_deg[i] << " " << s_setval[i] << " " << s_curval[i] << endl;
				}
			}
		}
		{//motors speed

			{
			pthread_mutex_lock(&l_mutex);
				uint32_t delta = GetDelta(pretmL);
				double tempSpeed = (Lstep*1000000)/delta;
				if (tempSpeed < 20)
					lps = lps*speedLerp;
			pthread_mutex_unlock(&l_mutex);
			}
			
			{
			pthread_mutex_lock(&r_mutex);
				uint32_t delta = GetDelta(pretmR);
				double tempSpeed = (Rstep*1000000)/delta;
				if (tempSpeed < 20)
					rps = rps*speedLerp;
			pthread_mutex_unlock(&r_mutex);
			}

			speedR = speedR*speedLerp + (1-speedLerp)*rps;
			speedL = speedL*speedLerp + (1-speedLerp)*lps;
		}
		{//motor control
			double deltaL = abs(setDist) - abs(distL*Lstep);
			double deltaR = abs(setDist) - abs(distR*Rstep);
			//cout << deltaL << " " << deltaR << endl;
			double s = 1;
			if (moveState == rotation || moveState == rot_cmp) s = -1;
			
			if (moveState == rot_cmp)
			{
				double delta_angle = GetDeltaAngle(yaw, setAngle);
				deltaL = abs(delta_angle*40);
				deltaR = deltaL;
				setSpeed = abs(setSpeed)*sign(-delta_angle);
			}
			
			if (moveState == straight || moveState == rotation|| moveState == rot_cmp)
			{//straight
				double syncErr = (distR*Rstep - s*distL*Lstep)*0.01*abs(setSpeed);
				double setSpeedL = setSpeed, setSpeedR = s*setSpeed;
				if (deltaL < 0 || deltaR < 0) syncErr = 0;
				if (deltaL < 0) setSpeedL = 0;
				if (deltaR < 0) setSpeedR = 0;
				
				//brakes
				double dl = deltaL/abs(setSpeed);
				double dr = deltaR/abs(setSpeed);
				if (dl > 0 && dl < braktime) setSpeedL = lerp(sign(setSpeed)*20, setSpeed, pow(dl/braktime,2));
				if (dr > 0 && dr < braktime) setSpeedR = lerp(sign(s*setSpeed)*20, s*setSpeed, pow(dr/braktime,2));
				
				vall += lPID.GetInfluence(speedL, setSpeedL + syncErr, false);
				valr += rPID.GetInfluence(speedR, setSpeedR - syncErr, false);

				if ((deltaL < 0 && deltaR < 0) || (moveState == rot_cmp && abs(GetDeltaAngle(yaw, setAngle)) < 0.05))
				{//fihish
					valr = 0;
					vall = 0;
					moveState = none;
					lPID.Init();
					rPID.Init();
					//cout << "Finish" << endl;
				}
			}

			vall = max(min(vall, 254), -254);
			valr = max(min(valr, 254), -254);
			dirL = sign(vall);
			dirR = sign(valr);
			rawmotor(0,vall);//L
			rawmotor(1,valr);//R
		}
		
		/*double tick = (double)GetDelta(timer)/1000.0;
		cout << tick << endl;
		timer = micros();*/
		delay(20);
	}
	return 0;
}

void motorsinit (void)
{
	pretmL = micros();
	pretmR = micros();

	//KL.KalmanInit(0, 45, 10, 500, 1, 1);
	//KR.KalmanInit(0, 45, 10, 500, 1, 1);
	
	lPID.Init();
	rPID.Init();
	
	wiringPiISR(0, INT_EDGE_BOTH, &myInterrupt0);
	wiringPiISR(2, INT_EDGE_BOTH, &myInterrupt2);
	
	timer = micros();
	pthread_create(&motorsthread, NULL, motorsProc, NULL);
	
	servo(0, 90);
	servo(1, 90);
	servo(2, 90);

}