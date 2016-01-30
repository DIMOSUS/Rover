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

#include "RF24/RF24.h"
#include "radio.h"

#define byte uint8_t

using namespace std;

struct tData {
	byte data[32];
    clock_t time;
};

RF24 radio(RPI_V2_GPIO_P1_22, RPI_V2_GPIO_P1_24, BCM2835_SPI_SPEED_4MHZ);
const clock_t clag = 100000;
const byte marker[32] = {172,57,168,0,255,21,140,160,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
const uint64_t pipes[2] = {	0xF0F0F0F0D2LL, 0xF0F0F0F0E1LL };
enum state_type { _stand=1, _str, _file, _fileName } RX_state;
queue<tData> rxQueue, txQueue;//очереди для задержки прием/передача
pthread_mutex_t rx_mutex = PTHREAD_MUTEX_INITIALIZER, tx_mutex = PTHREAD_MUTEX_INITIALIZER, recive_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_t thread, rxthread;
clock_t lag = clag * 0;//время лага
string RXs = "";
int32_t RXbytes = 0, RXpages = 0, RXtail = 0, nameLength = 0, namePages = 0, checksum = 0, arrPos = 0;

static void (*rxstring)(string);
static void (*rxfile)(string);

bool MarkerTest(byte *indata)
{
	for (int32_t i = 0; i < 8; i++)
		if (*(indata+i) != marker[i]) return false;
	return true;
}

void sendPacket(byte *indata)
{
	struct tData t;
	t.time = clock();
	for (int32_t i = 0; i < 32; i++)
		t.data[i] = *(indata+i);
	
	pthread_mutex_lock(&tx_mutex);
	txQueue.push(t);
	pthread_mutex_unlock(&tx_mutex);
}

void* rxtxThread(void *arg)
{
	int32_t Counter = 0;
	byte tx_data[32];
    while(true)
    {
		if (radio.available())
		{//rx
			//Принятые данные отправляем в очередь
			struct tData t;
			t.time = clock();
			radio.read(t.data, 32);
			
			pthread_mutex_lock(&rx_mutex);
			rxQueue.push(t);
			pthread_mutex_unlock(&rx_mutex);
			
			Counter = 0;
		}
		else
		{
			bool p_ready = false;
			pthread_mutex_lock(&tx_mutex);
			//проверяем очередь — есть что отправлять?
			if (!txQueue.empty() && clock() > txQueue.front().time + lag )
			{//tx
				for(int32_t i = 0; i < 32; i++)
					tx_data[i] = txQueue.front().data[i];
				txQueue.pop();
				p_ready = true;
				Counter = 0;
			}
			else
				Counter++;
			pthread_mutex_unlock(&tx_mutex);
			
			if (p_ready)
			{
				radio.stopListening();
				radio.write(&tx_data, 32);
				radio.startListening();
			}
		}
/*
		//Driver sleep
		if (Counter > 1000)
		{
			delay(1);
			Counter = 1001;
		}
		delayMicroseconds(100);*/
		delay(1);
    }
	return 0;
}

void SendText(string msg)
{
	byte dat[32];
	for (int32_t i = 0; i < 8; i++) dat[i] = marker[i];

	int32_t Length = msg.length();
	int32_t Pages = Length / 32; if (Length % 32 > 0) Pages++;
	byte Tail = (byte)(Length % 32); if (Tail == 0) Tail = 32;

	// 8 b маркер // 1 b тип пакета // 2 b количество пакетов // 1 b хвост в последнем пакете //
	dat[8] = 1;//mesage
	dat[9] = (byte)(Pages / 256);
	dat[10] = (byte)(Pages % 256);
	dat[11] = Tail;

	sendPacket(&dat[0]);

	for (int32_t p = 0; p < Pages; p++)
	{
		string piece = "";
		if (p < Pages - 1) piece = msg.substr(p * 32, 32);
		else piece = msg.substr(p * 32);

		for (int32_t i = 0; i < int32_t(piece.length()); i++)
			dat[i] = (byte)piece[i];

		sendPacket(&dat[0]);
	}
}

void SendFile(string patch)
{
	if (access(patch.c_str(), 0) == -1) return;
	byte dat[32];
	for (int32_t i = 0; i < 8; i++) dat[i] = marker[i];
	byte array[2097152];
	
	FILE *file;
	file = fopen(patch.c_str(), "rb");
	int32_t length = fread(array, sizeof(byte), 2097152, file);
	fclose(file);
	
	int32_t pages = length / 32; if (length % 32 > 0) pages++;
	byte tail = (byte)(length % 32); if (tail == 0) tail = 32;

	// 8 b маркер
	// 1 b тип пакета // 2 b количество пакетов // 1 b хвост в последнем пакете //
	dat[8] = 2;//file
	dat[9] = (byte)(pages / 256);
	dat[10] = (byte)(pages % 256);//2Mb max
	dat[11] = tail;

	int32_t cs = 0;
	for (int32_t i = 0; i < length; i++) cs += array[i];
	cs = cs % (256 * 256);
	int32_t namePages = patch.length() / 32; if (patch.length() % 32 > 0) namePages++;
	// 1b длинна имени // 1b пакетов с именем // 2b checksum
	dat[12] = (byte)(patch.length());
	dat[13] = (byte)(namePages);
	dat[14] = (byte)(cs / 256);
	dat[15] = (byte)(cs % 256);

	sendPacket(&dat[0]);

	for (int32_t i = 0; i < namePages; i++)
	{
		for (int32_t j = i * 32; j < (i + 1) * 32; j++)
		{
			if (j < int32_t(patch.length())) dat[j - i * 32] = (byte)patch[j];
			else dat[j - i * 32] = 0;
		}
		sendPacket(&dat[0]);
	}

	for (int32_t p = 0; p < pages; p++)
	{
		int32_t count = tail;
		if (p < pages - 1) count = 32;

		for (int32_t i = 0; i < count; i++)
			dat[i] = array[p * 32 + i];

		sendPacket(&dat[0]);
	}

}

void Recive_str()
{
	(*rxstring)(RXs);
}

void Recive_file()
{
	(*rxfile)(RXs);
}

void* rxProc(void *arg)
{
	byte tempArray[2097152];

	while(true)
	{
		pthread_mutex_lock(&rx_mutex);
		if (!rxQueue.empty() && clock() > rxQueue.front().time + lag )
		{
			byte t_buff[32];
			for(int32_t i = 0; i < 32; i ++)
				t_buff[i] = rxQueue.front().data[i];
			rxQueue.pop();
			pthread_mutex_unlock(&rx_mutex);
			//есть что обработать
			if (MarkerTest(&t_buff[0]))
			{
				if (t_buff[8] == 1)//message
				{
					RXpages = int32_t(t_buff[9]) * 256 + int32_t(t_buff[10]);
					RXtail = int32_t(t_buff[11]);
					RXbytes = RXpages * 32 + RXtail - 32;
					RX_state = _str;
					RXs = "";
				}
				if (t_buff[8] == 2)//file
				{
					RXpages = int32_t(t_buff[9]) * 256 + int32_t(t_buff[10]);
					RXtail = int32_t(t_buff[11]);
					RXbytes = RXpages * 32 + RXtail - 32;
					nameLength = int32_t(t_buff[12]);
					namePages = int32_t(t_buff[13]);
					checksum = int32_t(t_buff[14]) * 256;
					checksum += int32_t(t_buff[15]);
					RX_state = _fileName;
					RXs = "";
				}
			}
			else
			{
				if (RX_state == _str)
				{
					if (RXpages > 0)
					{
						RXpages--;
						if (RXpages == 0)
						{//end page
							if (RXtail == 0) RXtail = 32;
							for (int32_t i = 0; i < RXtail; i++)
								RXs += char(t_buff[i]);
							RX_state = _stand;
							Recive_str();
						}
						else
						{
							for (int32_t i = 0; i < 32; i++)
								RXs += char(t_buff[i]);
						}
					}
					else
					{//end
						RX_state = _stand;
						Recive_str();
					}
				}
				if (RX_state == _fileName)
				{
					for (int32_t i = 0; i < 32; i++)
					{
						RXs += char(t_buff[i]);
						if (int32_t(RXs.length()) == nameLength)
						{
							arrPos = 0;
							RX_state = _file;
							break;
						}
					}
				}
				else
				if (RX_state == _file)
				{
					//cout << "RXpage " << RXpages << endl;
					if (RXpages > 0)
					{
						RXpages--;
						if (RXpages == 0)
						{//end page
							if (RXtail == 0) RXtail = 32;
							for (int32_t i = 0; i < RXtail; i++)
							{
								tempArray[arrPos] = t_buff[i];
								arrPos++;
							}
							RX_state = _stand;

							int32_t cs = 0;
							for (int32_t i = 0; i < RXbytes; i++) cs += tempArray[i];
							cs = cs % (256 * 256);
							
							if (cs == checksum)
							{
								FILE *file;
								file = fopen(RXs.c_str(), "wb");
								fwrite(tempArray, sizeof(byte), RXbytes, file);
								fclose(file);
								Recive_file();
							}
						}
						else
						{
							for (int32_t i = 0; i < 32; i++)
							{
								tempArray[arrPos] = t_buff[i];
								arrPos++;
							}
						}
					}
				}
			}
		}
		else
		{
			pthread_mutex_unlock(&rx_mutex);
			delay(10);
		}
	}
	return 0;
}

void radioinit(void)
{
    radio.begin();
    radio.setAutoAck(1);
    radio.setRetries(15,15);
    radio.setDataRate(RF24_250KBPS);//RF24_1MBPS//RF24_250KBPS
    radio.setPALevel(RF24_PA_MAX);
    radio.setChannel(74);
    radio.setCRCLength(RF24_CRC_16);
	radio.openWritingPipe(pipes[0]);
	radio.openReadingPipe(1,pipes[1]);
    radio.startListening();
	
	pthread_create(&thread, NULL, rxtxThread, NULL);
	pthread_create(&rxthread, NULL, rxProc, NULL);
}

void regRXstrfile(void (*strf)(string), void (*filef)(string))
{
	rxstring = strf;
	rxfile = filef;
}

void setLag(uint32_t lagtime)
{
	lag = clag * lagtime;
}