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
#include "wiringPiI2C.h"
#include "bcm2835.h"
//#include "I2Cdev.h"
#include "MPU9150.h"
#include "MadgwickAHRS.h"
#include "bmp085.h"
#include "PCA9685.h"

#include "io.h"

#define byte uint8_t

#define ADDR_HTU 0x40
#define ADDR_328 0x04
#define ADDR_BMP 0x77
#define PI 3.1415926535897932384626433832795

#define BatFull 4.17
#define BatEmpty 3.2
#define BatCap 3000

using namespace std;

//3.3|gnd|scl|sda|5|gnd1|gnd2|A2|A1|gnd

volatile double Humid = 0, Temp = 0, Pres = 0, Mag = 0, PreMLX = 0;
volatile int32_t Altitude = 0, Satellites = 0;
volatile double roll, pitch, yaw;

volatile double table[4][TableSize];
volatile double t_counter = 0;

pthread_t gpsthread, mputhread, sensorthread;
pthread_mutex_t i2c_mutex = PTHREAD_MUTEX_INITIALIZER;

MPU9150 accelgyro;
int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
int32_t SerialPort;

byte data[4]={0,0,0,0}, in_data[4]={0,0,0,0};

//gps
enum GPSStatusEnum { GPRMC,	GPGSV,	GPGSA,	GPGGA,	wait } GPSReadStatus = wait;
volatile int32_t YY = 0, MM = 0, DD = 0, hh = 0, mm = 0, ss = 0, GPSbloc = 0;
string NS = "", EW = "", Latitude = "", Longitude  = "", GPStmp = "";
volatile bool GPSinit = true, Valid = false;

volatile double LatitudeDec = 0, LongitudeDec = 0;

double vref0 = 5.0, vref1 = 5.0;

int32_t preL = -1, preR = -1, mtrEn = 0, rotDirL = 0;

PCA9685 pca(0x42);

int32_t getHash(int32_t baytes)
{
	int32_t sum = 0;
	for (int32_t i = 0; i < baytes; i++)
		sum += data[i];
	return sum % 256;
}

void rawmotor(int32_t R, int32_t Speed)
{
	if (R < 1 && Speed == preL) return;
	if (R > 0 && Speed == preR) return;
	pthread_mutex_lock(&i2c_mutex);
		data[0] = 1 + (R > 0);
		data[1] = Speed < 0;
		data[2] = min(abs(Speed), 254);
		int32_t hash = getHash(3);
		data[3] = hash;
		
		bcm2835_i2c_set_baudrate(100000);
		bcm2835_i2c_setSlaveAddress(0x06);
		int32_t i = 100;
		while(true)
		{
			bcm2835_i2c_write ((char*)data, 4);
			bcm2835_i2c_read((char*)in_data, 1);
			if (in_data[0] == hash) break;
			if(i-- < 0) break;
			//else cout << (int32_t)in_data[0] << endl;
		}
	pthread_mutex_unlock(&i2c_mutex);
	if (R < 1 ) preL = Speed;
	else preR = Speed;
}

void rawservo(int32_t ind, int32_t val)
{
//	cout << "Servo " << ind << " " << val << endl;
	if (ind > -1 && ind < 3)
	{
	pthread_mutex_lock(&i2c_mutex);
	
		int32_t v = (double)val/20000.0*4095;
		int32_t s_ind = 7;
		if (ind == 0) s_ind = 5;
		if (ind == 1) s_ind = 6;
		pca.setPWM(s_ind, v);
		pca.setPWM(s_ind, v);
		//cout << endl << ind << " " << val << endl;
	pthread_mutex_unlock(&i2c_mutex);
	}
}

void gnd(int32_t ind, int32_t val)
{
	//cout << "gnd " << ind << " " << val << endl;
	if (ind > 11 && ind < 14)
	{
	pthread_mutex_lock(&i2c_mutex);
		data[0] = ind;
		data[1] = val;
		int32_t hash = getHash(2);
		data[2] = hash;

		bcm2835_i2c_set_baudrate(100000);
		bcm2835_i2c_setSlaveAddress(0x04);
		int32_t i=100;
		while(true)
		{
			bcm2835_i2c_write ((char*)data, 3);
			delay(1);
			bcm2835_i2c_read((char*)in_data, 1);
			if (in_data[0] == hash) break;
			if(i-- < 0) break;
			//else cout << (int32_t)in_data[0] << endl;
		}
	pthread_mutex_unlock(&i2c_mutex);
	}
}

double aRead(int32_t port, int32_t ard)
{
	if(port > -1 && port < 5)
	{
	pthread_mutex_lock(&i2c_mutex);
		data[0] = port+3;
		data[1] = data[0];
		
		bcm2835_i2c_set_baudrate(100000);
		bcm2835_i2c_setSlaveAddress(0x04 + min(ard,1)*2);
		int32_t i=100;
		while(true)
		{
			bcm2835_i2c_write ((char*)data, 2);
			delay (3);
			bcm2835_i2c_read((char*)in_data, 3);
			if (in_data[2] == port+3 && in_data[0] < 4) break;
			if(i-- < 0) break;
		}
		double val = in_data[0] * 256.0 + in_data[1];
	pthread_mutex_unlock(&i2c_mutex);

		if (ard < 1) return val/1023.0*vref0;
		else return val/1023.0*vref1;
	}
	else return 0;
}

double aReadFiltr(int32_t port, int32_t ard)
{
	const int32_t mes = 12;
	double measures [mes];
	
	for (int32_t i = 0; i < mes; i++)
		measures[i] = aRead(port, ard);
	
	double avg = 0;
	for (int32_t i = 0; i < mes; i++)
		avg += measures[i];
	avg /= mes;
	
	
	int32_t misses [mes];
	int32_t mis = 0;
	for(int32_t i = 0; i < mes/3; i++)
	{
		int32_t ind = 0;
		double preabs = 0;
		
		for (int32_t j = 0; j < mes; j++)
		{
			bool ok = true;
			for (int32_t k = 0; k < mis; k++)
				if (misses[k] == j) ok = false;
			
			if (ok && preabs < abs(measures[j]-avg))
			{
				preabs = abs(measures[j]-avg);
				ind = j;
			}
		}
		misses[mis] = ind;
		mis++;
	}

	double favg = 0;
	int32_t find = 0;
	for (int32_t j = 0; j < mes; j++)
	{
		bool ok = true;
		for (int32_t k = 0; k < mis; k++)
			if (misses[k] == j) ok = false;
		if (ok)
		{
			favg += measures[j];
			find++;
		}
	}
	
	return favg/find;
}

double vbat(void)
{
	double e = aReadFiltr(0, 1);
	return round(e*100.0)*0.01;
}

double cbat(double temp)
{
	double bat = (aReadFiltr(0, 1) - BatEmpty)/(BatFull-BatEmpty);
	bat = max(0, bat);
	bat = min(1, bat);
	
	bat = lerp(bat, pow(bat, 0.37), bat*bat);//voltage correction
	
	temp = min(25, temp);
	temp = max(-20, temp);
	
	bat *= 0.54545454+pow((temp+20)/45, 0.7)*0.45454545;//temperature correction
	
	bat*=100;
	
	return round(bat);
}

double lux(void)
{
	double read = aRead(3, 0);
	double R = (9100*read)/(3.28-read);
	return double(137000000/(pow(R, 1.805)));
}

double dist(void)
{
	gnd(13,1);
	delay(50);
	double x = aReadFiltr(2, 0);
	gnd(13,0);
	return (16.2537 * pow(x,4) - 129.893 * pow(x,3) + 382.268 * pow(x,2) - 512.611 * pow(x,1) + 306.439);
}

double getAngle(string str)
{
	if (str.length() < 3) return 0;
	int32_t pind = str.find('.');
	double valh = 0, vall = 0;
	istringstream(str.substr(0,pind-2)) >> valh;
	istringstream(str.substr(pind-2)) >> vall;
	return valh + vall/60.0;
}

void procgps(char ch)
{
	if (ch == '$')
	{//start
		GPStmp = "";
		GPSbloc = 0;
		GPSReadStatus = wait;
	}
	
	if (ch == ',' || ch == '*')
	{//data ready
		if (GPStmp.length() > 0)
		{//process
			if (GPStmp == "GPRMC")
				GPSReadStatus = GPRMC;
			if (GPStmp == "GPGGA")
				GPSReadStatus = GPGGA;
			if (GPStmp == "GPGSV")
				GPSReadStatus = GPGSV;
			if (GPSReadStatus == GPRMC)
			{
				if (GPSbloc == 1)
				{
					hh = atoi(GPStmp.substr(0,2).c_str());
					mm = atoi(GPStmp.substr(2,2).c_str());
					ss = atoi(GPStmp.substr(4,2).c_str());
				}
				if (GPSbloc == 2)
				{
					if (GPStmp == "A" ) Valid = true;
					else 
					{
						Valid = false;
						Satellites = 0;
						GPSinit = true;
					}
				}
				if (Valid)
				{
					switch (GPSbloc) {
						case 3:	{
							Latitude = GPStmp;
						} break;
						case 4:	{
							NS = GPStmp;
						} break;
						case 5:	{
							Longitude = GPStmp;
						} break;
						case 6:	{
							EW = GPStmp;
						} break;
						case 9:	{
							DD = atoi(GPStmp.substr(0,2).c_str());
							MM = atoi(GPStmp.substr(2,2).c_str());
							YY = atoi(GPStmp.substr(4,2).c_str());
						}
					}
				}
			}
			if (GPSReadStatus == GPGGA)
			{
				if (GPSbloc == 9)
				{
					Altitude = atoi(GPStmp.c_str());
				}
			}
			if (GPSReadStatus == GPGSV)
			{
				if (GPSbloc == 3)
				{
					Satellites = atoi(GPStmp.c_str());
				}
			}
		}
		GPSbloc++;
		GPStmp = "";
	}
	else
	{
		if (ch != '$') GPStmp += ch;
	}
	
	if (ch == '*')
	{//end msg
		if (GPSReadStatus == GPRMC && Valid)
		{
			if (GPSinit)
			{
				LatitudeDec = getAngle(Latitude);
				LongitudeDec = getAngle(Longitude);
				GPSinit = false;
			}
			else
			{
				double Kstab = 0.125;
				LatitudeDec = getAngle(Latitude)*Kstab + (1-Kstab)*LatitudeDec;
				LongitudeDec = getAngle(Longitude)*Kstab + (1-Kstab)*LongitudeDec;
			}
			//cout << fixed << setprecision(10) << LatitudeDec << " " << LongitudeDec << endl;
			//cout << "Satellites " << Satellites << endl;
		}
		GPSReadStatus = wait;
	}
}

void* gpsProc(void *arg)
{
	while(true)
	{
		char buf[1024];
		int32_t sz = 0;
		do {
			sz = read(SerialPort, buf, 1024);
			for(int32_t i = 0; i < sz; i++)
				procgps(buf[i]);
				//cout << buf[i];
		} while (sz>0);
		delay(100);
	}
	return 0;
}

void updateSensors(void)
{
	{pthread_mutex_lock(&i2c_mutex);
		I2Cdev::writeByte(0x68, 0x37, 0x00); //set i2c bypass enable pin to true to access magnetometer
		delay(1);
	
		char buf [4];
		uint32_t tmp;
		const char reg = 0xF3;
		bcm2835_i2c_set_baudrate(100000);
		bcm2835_i2c_setSlaveAddress(ADDR_HTU);
		bcm2835_i2c_write (&reg, 1);
		delay (55);
		bcm2835_i2c_read(&buf[0], 3);
		//cout << (int32_t) buf[0] << " " << (int32_t) buf[1] << " " << (int32_t) buf[2] << endl;
		tmp = (buf [0] << 8 | buf [1]) & 0xFFFC;
		double tSensorTemp = tmp / 65536.0;
		Temp = -46.85 + (175.72 * tSensorTemp);
		
		I2Cdev::writeByte(0x68, 0x37, 0x02); //set i2c bypass dissable pin to true to access magnetometer
		delay(1);
	pthread_mutex_unlock(&i2c_mutex);}
	
	delay(1);
	
	{pthread_mutex_lock(&i2c_mutex);
		I2Cdev::writeByte(0x68, 0x37, 0x00); //set i2c bypass enable pin to true to access magnetometer
		delay(1);
	
		char buf [4];
		uint32_t tmp;
		const char reg = 0xF5;
		bcm2835_i2c_set_baudrate(100000);
		bcm2835_i2c_setSlaveAddress(ADDR_HTU);
		bcm2835_i2c_write (&reg, 1);
		delay (55);
		bcm2835_i2c_read(&buf[0], 3);
		//cout << (int32_t) buf[0] << " " << (int32_t) buf[1] << " " << (int32_t) buf[2] << endl;
		tmp = (buf [0] << 8 | buf [1]) & 0xFFFC;
		double tSensorHumid = tmp / 65536.0;
		Humid = -6.0 + (125.0 * tSensorHumid);
		
		I2Cdev::writeByte(0x68, 0x37, 0x02); //set i2c bypass dissable pin to true to access magnetometer
		delay(1);
	pthread_mutex_unlock(&i2c_mutex);}

	delay(1);
	
	{pthread_mutex_lock(&i2c_mutex);
	
		uint32_t pres = 0, temp = 0;
		bcm2835_i2c_set_baudrate(400000);
		bmp085Calibration();
		delay (2);
		bmp085Convert(&temp, &pres);
		Pres = ((double)pres)/100;
		
	pthread_mutex_unlock(&i2c_mutex);}
}

double mlxTemp()
{
	int32_t c = 5;
	double temp=-99;
	while(true)
	{
		c--;
	pthread_mutex_lock(&i2c_mutex);
		char buf[6];
		const char reg = 7;
		char rreg = 7;

		bcm2835_i2c_set_baudrate(50000);
		
		bcm2835_i2c_setSlaveAddress(0x5a);
		bcm2835_i2c_write (&reg, 1);
		bcm2835_i2c_read_register_rs(&rreg,&buf[0],3);
		
		temp = (double) (((buf[1]) << 8) + buf[0]);
		temp = (temp * 0.02)-0.01;
		temp = temp - 273.15;
	pthread_mutex_unlock(&i2c_mutex);
		if (temp > -60) break;
		else 
			if (c <  0)
			{
				temp = PreMLX;
				break;
			}
			else delay(1);
	}
	PreMLX = temp;
	return temp;
}

float raw2rad(int16_t val)
{
	return ((double)val)/65.5/180.0*PI;
}

void* mpuProc(void *arg)
{
	//double dx=0, dy=0, dz=0, lf=0.02;
	//double gdx=0, gdy=0, gdz=0;
	
	piHiPri(55);
	
	pthread_mutex_lock(&i2c_mutex);
			bcm2835_i2c_set_baudrate( 400000 );
			I2Cdev::enable(true);
			accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
			I2Cdev::enable(false);
	pthread_mutex_unlock(&i2c_mutex);
	Mag = sqrt(mx*mx + my*my + mz*mz + 0.1);

	while(true)
	{
		delay(30);
		pthread_mutex_lock(&i2c_mutex);
			bcm2835_i2c_set_baudrate( 400000 );
			I2Cdev::enable(true);
			accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
			I2Cdev::enable(false);
		pthread_mutex_unlock(&i2c_mutex);
		//MadgwickAHRSupdate(raw2rad(gx), raw2rad(gy), raw2rad(gz), ax, ay, az, mx, my, mz);
		MadgwickAHRSupdateIMU(raw2rad(gx), raw2rad(gy), raw2rad(gz), ax, ay, az);
		
		Mag = lerp(Mag, sqrt(mx*mx + my*my + mz*mz + 0.1), 0.005);
		/*
		dx = lerp(dx, ax, lf);
		dy = lerp(dy, ay, lf);
		dz = lerp(dz, az, lf);
		printf("%f %f %f       \r", dx, dy, dz);
		fflush(stdout);
		*/
		/*
		gdx = lerp(gdx, gx, lf);
		gdy = lerp(gdy, gy, lf);
		gdz = lerp(gdz, gz, lf);
		printf("%f %f %f              \r", gdx, gdy, gdz);
		fflush(stdout);
		*/
		
		yaw   = atan2((2*(q1*q2+q0*q3)),(pow(q0,2)+pow(q1,2)-pow(q2,2)-pow(q3,2)));
		pitch = asin(-2*(q1*q3-q0*q2));
		roll  = atan2(2*(q0*q1+q2*q3),pow(q0,2)-pow(q1,2)-pow(q2,2)+pow(q3,2));
	}
	return 0;
}

void* sensorsProc(void *arg)
{
	while(true)
	{
		//TODO в простое
		updateSensors();
		
		for (int32_t i = TableSize-1; i > 0; i--)
			for (int32_t k = 0; k < 4; k ++)
				table[k][i] = table[k][i-1];
			
		table[0][0] = Humid;
		table[1][0] = Temp;
		table[2][0] = Pres;
		table[3][0] = Mag;

		t_counter = min(t_counter+1, TableSize);
		
		delay(60*1000);
	}
	return 0;
}

void ioinit (void)
{
	wiringPiSetup();
	
	{//serial
		FILE* file;

		file = fopen("/dev/ttyAMA0", "r+");

		if (file == NULL) {
			perror("/dev/ttyAMA0");
			exit(EXIT_FAILURE);
		}
		SerialPort = fileno(file);

		struct termios toptions;
		speed_t brate = B9600;
		
		if (tcgetattr(SerialPort, &toptions) < 0)
		{
			perror("Can't get term attributes");
			exit(EXIT_FAILURE);
		}
		
		cfsetispeed(&toptions, brate);
		cfsetospeed(&toptions, brate);
		
		// 8N1
		toptions.c_cflag &= ~PARENB;
		toptions.c_cflag &= ~CSTOPB;
		toptions.c_cflag &= ~CSIZE;
		toptions.c_cflag |= CS8;
		// no flow control
		toptions.c_cflag &= ~CRTSCTS;

		toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
		toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

		toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
		toptions.c_oflag &= ~OPOST; // make raw

		toptions.c_cc[VMIN]  = 0;
		toptions.c_cc[VTIME] = 20;
			
		if (tcsetattr(SerialPort, TCSANOW, &toptions) < 0)
		{
			perror("Can't set term attributes");
			exit(EXIT_FAILURE);
		}
		
		fcntl(SerialPort, F_SETFL, FNDELAY);
	}

	I2Cdev::initialize();
	
	if (accelgyro.testConnection())
	{
		accelgyro.initialize();
		accelgyro.setFullScaleGyroRange(1);//FS_SEL=1 +-500

		accelgyro.setXAccelOffset(339);
		accelgyro.setYAccelOffset(1635);
		accelgyro.setZAccelOffset(750);
		accelgyro.setXGyroOffset(10);
		accelgyro.setYGyroOffset(18);
		accelgyro.setZGyroOffset(-14);
		
		pthread_create(&mputhread, NULL, mpuProc, NULL);
	}
	else {
		fprintf( stderr, "MPU9150 connection test failed!\n");
	}
cout << "MPU9150" << endl;
	//init
	gnd(12,0);
	gnd(13,0);
cout << "gnd" << endl;
	pca.setDgt(16, false);
	pca.setDgt(13, false);
cout << "pca" << endl;
	//calibrate
	double e = aReadFiltr(0, 0);
	vref0 = vref0*3.3/e;
	
	e = aReadFiltr(2, 1);
	vref1 = vref1*3.3/e;
cout << "calibrate" << endl;
	//cout << "vRef0 " << vref0 << endl;
	//cout << "vRef1 " << vref1 << endl;
	
	pthread_create(&gpsthread, NULL, gpsProc, NULL);
	pthread_create(&sensorthread, NULL, sensorsProc, NULL);
cout << "io thread" << endl;
}