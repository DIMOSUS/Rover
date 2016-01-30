/*
 * bmp085.h:
 *	Extend wiringPi with the BMP085 Barometer chip
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
#ifndef BMP_085
#define BMP_085
extern int bmp085Setup (const int i2cAddress);
extern void bmp085Calibration (void);
extern int bmp085GetId (void);
extern int bmp085GetVer (void);
extern char bmp085GetOSS (void);
extern void bmp085SetOSS (char os);
extern void bmp085Convert (unsigned int* temperature, unsigned int* pressure);
#endif
