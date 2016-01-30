#include <Wire.h>

#define SLAVE_ADDRESS 0x06
#define min(a,b) (((a)<(b))?(a):(b))

byte buf [4], sent_buf [4];
int state = 0, l = 0, r = 0;

void setup()
{ 
	pinMode(0, OUTPUT); 
	pinMode(6, OUTPUT); 
	pinMode(9, OUTPUT); 
	pinMode(10, OUTPUT); 
	pinMode(11, OUTPUT);
	
	digitalWrite(0, LOW);
	digitalWrite(6, LOW);
	digitalWrite(9, LOW);
	digitalWrite(10, LOW);
	digitalWrite(11, LOW);
	
	Wire.begin(SLAVE_ADDRESS);
	Wire.onReceive(receiveData);
	Wire.onRequest(sendData);
}

void loop()
{
    delay(10);
}



int getHash(int baytes)
{
	int sum = 0;
	for (int i = 0; i < baytes; i++)
		sum += buf[i];
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
	else if (byteCount == 4)
	{
		int hash = getHash(3);
		state = 2;
		if (hash == buf[3])
		{
			sent_buf[0] = hash;
			if (buf[0] == 1)
			{//L
				if (buf[2] == 0)
				{//set 0
					l = 0;
					if (r == 0) digitalWrite(0, LOW);
					digitalWrite(10, LOW);
					digitalWrite(6, LOW);
				}
				else
				{
					digitalWrite(0, HIGH);
					if (buf[1] == 0)
					{//+
						l = buf[2];
						analogWrite(6, min(buf[2], 254));
						digitalWrite(10, LOW);
					}
					else
					{//-
						l = -buf[2];
						analogWrite(10, min(buf[2], 254));
						digitalWrite(6, LOW);
					}
				}
			}
			else if (buf[0] == 2)
			{//R
				if (buf[2] == 0)
				{//set 0
					r = 0;
					if (l == 0) digitalWrite(0, LOW);
					digitalWrite(11, LOW);
					digitalWrite(9, LOW);
				}
				else
				{
					digitalWrite(0, HIGH);
					if (buf[1] == 0)
					{//+
						r = buf[2];
						analogWrite(9, min(buf[2], 254));
						digitalWrite(11, LOW);
					}
					else
					{//-
						r = -buf[2];
						analogWrite(11, min(buf[2], 254));
						digitalWrite(9, LOW);
					}
				}
			}
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