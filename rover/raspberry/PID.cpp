#include <stdint.h>
#include <cmath> 
//#include <iostream>
#include "PID.h"

#define lerp(a,b,c) (((b) - (a)) * (c) + (a))
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))

template <typename T>
int32_t sign(T Val) {
  //if (Val == 0.)  return 0;
  if (Val < 0.)  return -1;
  else return 1;
}

using namespace std;

double TPID::GetInfluence(double Speed, double SetSpeed, bool debug)
{
	float Delta = sign(SetSpeed - Speed)*min(pow(SetSpeed - Speed, 2)*p, abs(SetSpeed)/100)
		+ (SetSpeed - Speed)*0.001;
//cout << Speed << " ";
//cout << Delta << " ";
	error += SetSpeed-Speed;
	error = min(ilimit, error);
	error = max(-ilimit, error);
	Delta += error*i;
//cout << error*i << " ";
	double diff = Speed - pspeed;
	diff *= d;
	Delta -= diff;
//cout << diff << endl;
	pspeed = Speed;
	return Delta;
}

void TPID::Init()
{
	this->p = 0.0003;
	
	this->i = 0.005;
	this->ilimit = 10;
	
	this->d = 0.02;
	
	this->error = 0;
	this->pspeed = 0;
}

TPID::TPID() { }

TPID::~TPID() { }