#ifndef PIDCNTRL
#define PIDCNTRL

class TPID
{
private:
	double error;
	double pspeed;
	double p;
	double i;
	double d;
	double ilimit;

public:
	TPID();
	~TPID();
	double GetInfluence(double, double, bool);
	void Init();
};

#endif