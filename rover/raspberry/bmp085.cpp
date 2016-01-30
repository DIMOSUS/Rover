/*
 * bmp085.c:
 *	Extend wiringPi with the BMP085 I2C Barometer chip
 *	Copyright (c) 2013 Gordon Henderson
 *	Code Hacking P Bennett
 *	Based on ATmega328 code by: Jim Lindblom & Raspberry Pi code by: John Burns
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as
 *    published by the Free Software Foundation, either version 3 of the
 *    License, or (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with wiringPi.
 *    If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <pthread.h>

#include "wiringPi.h"
#include "wiringPiI2C.h"

#include "bmp085.h"

#define BMP085_Address 0x77         //I2C Bus address

/////=========Global Variables======////////////////////
int id = -1 ;
int ver = -1 ;
unsigned char OSS = 0 ;

short ac1 ;
short ac2 ;
short ac3 ;
unsigned short ac4 ;
unsigned short ac5 ;
unsigned short ac6 ;
short b1 ;
short b2 ;
short mb ;
short mc ;
short md ;

int bmp085Setup (const int i2cAddress)
{
  return wiringPiI2CSetup (i2cAddress) ;
}

int bmp085ReadInt (int fd, int reg)
{
  int data = wiringPiI2CReadReg16 (fd, reg) ;
  data = ((data<<8) & 0xFF00) | ((data>>8) & 0xFF) ;       //Change byte order
  return data ;
}

//
void bmp085Calibration ()
{
  int fd = bmp085Setup (BMP085_Address) ;
/* printf ("\nCalibration Information:\n") ;
  printf ("------------------------\n") ;*/
  ac1 = bmp085ReadInt (fd, 0xAA) ;
  ac1 = bmp085ReadInt (fd, 0xAA) ;
  ac2 = bmp085ReadInt (fd, 0xAC) ;
  ac2 = bmp085ReadInt (fd, 0xAC) ;
  ac3 = bmp085ReadInt (fd, 0xAE) ;
  ac4 = bmp085ReadInt (fd, 0xB0) ;
  ac5 = bmp085ReadInt (fd, 0xB2) ;
  ac6 = bmp085ReadInt (fd, 0xB4) ;
  b1 = bmp085ReadInt (fd, 0xB6) ;
  b2 = bmp085ReadInt (fd, 0xB8) ;
  mb = bmp085ReadInt (fd, 0xBA) ;
  mc = bmp085ReadInt (fd, 0xBC) ;
  md = bmp085ReadInt (fd, 0xBE) ;
  id = wiringPiI2CReadReg8 (fd, 0xD0) ;
  ver = wiringPiI2CReadReg8 (fd, 0xD1) ;
/* printf ("\tID = %X Hex\n", id) ;
  printf ("\tVer = %d\n", ver) ;
  printf ("------------------------\n") ;
  printf ("\tAC1 = %d\n", ac1) ;
  printf ("\tAC2 = %d\n", ac2) ;
  printf ("\tAC3 = %d\n", ac3) ;
  printf ("\tAC4 = %d\n", ac4) ;
  printf ("\tAC5 = %d\n", ac5) ;
  printf ("\tAC6 = %d\n", ac6) ;
  printf ("\tB1 = %d\n", b1) ;
  printf ("\tB2 = %d\n", b2) ;
  printf ("\tMB = %d\n", mb) ;
  printf ("\tMC = %d\n", mc) ;
  printf ("\tMD = %d\n", md) ;
  printf ("------------------------\n\n") ; */
  close (fd) ;
}

int bmp085GetId () 
{
  if (id == -1)
    bmp085Calibration () ;

  return id ;
}

int bmp085GetVer ()
{
  if (ver == -1)
    bmp085Calibration () ;

  return ver ;
}

char bmp085GetOSS ()
{
  return OSS;
}

void bmp085SetOSS (char os)
{
  if ((os >= 0) || (os < 4))
    OSS = os ;
}

unsigned int bmp085ReadTemp ()
{
  unsigned int utt[8] ;		//Average 8 readings.
  unsigned int ut = 0 ;
  int fd = bmp085Setup (BMP085_Address) ;
  int i ;
  for (i = 0; i < 8; i++) {
    wiringPiI2CWriteReg8 (fd, 0xF4, 0x2E) ;
    delay (5) ;                                                   // max time is 4.5ms
    utt[i] = bmp085ReadInt (fd, 0xF6) ;
    utt[i] &= 0x0000FFFF ;
//    ut += utt[i]/8 ;
  }
  ut = ((utt[0] + utt[1] + utt[2] + utt[3] + utt[4] + utt[5] + utt[6] + utt[7])/8) ; 
  close (fd) ;
  return ut ;
}

unsigned int bmp085ReadPressure () 
{
  unsigned int umsb[8], ulsb[8], uxlsb[8] ;
  unsigned int up = 0 ;
  unsigned int msb, lsb ,xlsb ;
  msb = lsb = xlsb = 0 ;
  int fd = bmp085Setup (BMP085_Address) ;
  int i ;
  for (i = 0; i < 8; i++) {
    wiringPiI2CWriteReg8 (fd, 0xF4, 0x34 + (OSS<<6)) ;
    delay (2 + (3<<OSS));
    umsb[i] = (unsigned int) wiringPiI2CReadReg8 (fd, 0xF6) ;
    ulsb[i] = (unsigned int) wiringPiI2CReadReg8 (fd, 0xF7) ;
    uxlsb[i] = (unsigned int) wiringPiI2CReadReg8 (fd, 0xF8) ;
  }
  msb = ((umsb[0] + umsb[1] + umsb[2] + umsb[3] + umsb[4] + umsb[5] + umsb[6] + umsb[7])/8) ;
  lsb = ((ulsb[0] + ulsb[1] + ulsb[2] + ulsb[3] + ulsb[4] + ulsb[5] + ulsb[6] + ulsb[7])/8) ;
  xlsb = ((uxlsb[0] + uxlsb[1] + uxlsb[2] + uxlsb[3] + uxlsb[4] + uxlsb[5] + uxlsb[6] + uxlsb[7])/8) ;
  up = (((unsigned int) msb << 16) | ((unsigned int) lsb << 8) | (unsigned int) xlsb) >> (8-OSS);
  up &= 0x0000FFFF ;
  close (fd) ;
  return up ;
}

void bmp085Convert (unsigned int* temperature, unsigned int* pressure)
{

  int x1, x2, x3, b3, b6, p ;
  int b5 ;
  unsigned int b4, b7 ;

  unsigned int ut = bmp085ReadTemp () ;
  unsigned int up = bmp085ReadPressure () ;

  x1 = (((int)ut - (int)ac6)*(int)ac5) >> 15 ;
  x2 = ((int)mc << 11)/(x1 + md) ;
  b5 = x1 + x2 ;

  *temperature = ((b5 + 8)>>4) ;

  b6 = b5 - 4000 ;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11 ;
  x2 = (ac2 * b6)>>11 ;
  x3 = x1 + x2 ;
  b3 = (((((int)ac1)*4 + x3)<<OSS) + 2)>>2 ;

  // Calculate B4
  x1 = (ac3 * b6)>>13 ;
  x2 = (b1 * ((b6 * b6)>>12))>>16 ;
  x3 = ((x1 + x2) + 2)>>2 ;
  b4 = (ac4 * (unsigned int)(x3 + 32768))>>15 ;

  b7 = ((unsigned int)(up - b3) * (50000>>OSS)) ;
  if (b7 < 0x80000000)
    p = (b7<<1)/b4 ;
  else
    p = (b7/b4)<<1 ;

  x1 = (p>>8) * (p>>8) ;
  x1 = (x1 * 3038)>>16 ;
  x2 = (-7357 * p)>>16 ;
  p += (x1 + x2 + 3791)>>4 ;
  *pressure = p ;
}