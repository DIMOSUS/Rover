#include <Wire.h>

#define SLAVE_ADDRESS 0x04
#define min(a,b) (((a)<(b))?(a):(b))

byte buf [4], sent_buf [4];
int state = 0;

void setup()
{ 
	pinMode(12, OUTPUT); 
	pinMode(13, OUTPUT); 
	
	digitalWrite(12, LOW);
	digitalWrite(13, LOW);
	
	Wire.begin(SLAVE_ADDRESS);
	Wire.onReceive(receiveData);
	Wire.onRequest(sendData);
}

void loop()
{
    delay(1);
}

//03 analog 0
//04 analog 1
//05 analog 2
//06 analog 3
//12 gnd0
//13 gnd1

int getHash(int baytes)
{
	int sum = 0;
	for (int i = 0; i < baytes; i++)
	{
		sum += buf[i];
	}
	return sum % 256;
}

// callback for received data
void receiveData(int byteCount)
{
	int number = 0;
	while(Wire.available())
	{
		buf[number]  = Wire.read();
		number++;
	}
	if (byteCount == 2)
	{
		if(getHash(1) == buf[1] && buf[0] > 2 && buf[0] < 7)
		{//analog
			int val = analogRead(buf[0]-3);
			state = 3;
			sent_buf[0] = val>>8;
			sent_buf[1] = (val - sent_buf[0]*256);
			sent_buf[2] = buf[0];
		}
		else sent_buf[2] = 255;
	}
	else if (byteCount == 3)
	{
		int hash = getHash(2);
		if(hash == buf[2] && buf[0] > 11 && buf[0] < 14)
		{//gnd
			state = 2;
			sent_buf[0] = hash;
			if (buf[0] == 12) digitalWrite(12, buf[1] > 0);
			if (buf[0] == 13) digitalWrite(13, buf[1] > 0);
		}
	}
}

// callback for sending data
void sendData()
{
	if (state == 3)
		Wire.write(sent_buf, 3);
	if (state == 2)
		Wire.write(sent_buf, 1);
}