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
#include <unistd.h>
#include <stdint.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/file.h>
#include <pthread.h>
#include <termios.h>

#include "opencv2/opencv.hpp"
#include "wiringPi.h"

#include "io.h"
#include "cam_gui.h"

#define byte uint8_t
#define H 972
#define W 1296
#define fov 47.5
#define fov_cor 0.5
#define PI 3.1415926535897932384626433832795

using namespace std;
using namespace cv;

volatile int32_t ir_t = 4;
Mat cam_matrix, map_x, map_y;

void setirt(int32_t t)
{
	ir_t = t;
}

string getName(void)
{
	time_t t = time(NULL);
	tm* aTm = localtime(&t);
	
	ostringstream ost;
	ost << (aTm->tm_mon+1 < 10 ? "0" : "") << aTm->tm_mon+1 << ".";//mon
	ost << (aTm->tm_mday < 10 ? "0" : "") << aTm->tm_mday << "_";//day
	ost << (aTm->tm_hour < 10 ? "0" : "") << aTm->tm_hour << ".";//hour
	ost << (aTm->tm_min < 10 ? "0" : "") << aTm->tm_min << "'";//min
	ost << (aTm->tm_sec < 10 ? "0" : "") << aTm->tm_sec << "''.jpg";//sec

	return ost.str();
	
	free(aTm);
}

Mat getShot()
{
	Mat frame, undistframe;
	VideoCapture cap(0); // open the default camera
	
	if(!cap.isOpened())  // check if we succeeded
	{
        cout << "Camera Error" << endl;
		return frame;
	}
	
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, H);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,  W);
	//cap.set(CV_CAP_PROP_EXPOSURE,  0.01);
	
	float l = lux();
	if (l <= ir_t) gnd(12,1);
	if (l > 2)
		cap >> frame;
	else
	{
		system("raspistill -o s.jpg -w 1296 -h 972");
		frame = imread("s.jpg", CV_LOAD_IMAGE_COLOR);
	}
	gnd(12,0);
	
	remap(frame, undistframe, map_x, map_y, INTER_LANCZOS4, BORDER_CONSTANT, Scalar(0,0,0));
	
	//circle(undistframe, Point(W/2, H/2), 100, Scalar(0, 255, 255), 1, CV_AA);
	return undistframe;
}

Mat panoram()
{
	Mat outImg(Size(W*3, H), CV_8UC3);
	
	while (!servo(0, 87)) {delay(10);}
	//while (!servo(1, 100)) {delay(100);}
	float centre_2 = 87;
	
	while (!servo(2, centre_2+fov-fov_cor)) {delay(100);}
	delay(300);
	Mat shot = getShot();
	shot.copyTo(outImg(Rect(0, 0, W, H)));
	
	while (!servo(2, centre_2)) {delay(100);}
	delay(300);
	shot = getShot();
	shot.copyTo(outImg(Rect(W, 0, W, H)));
	
	while (!servo(2, centre_2-fov-fov_cor)) {delay(100);}
	delay(300);
	shot = getShot();
	shot.copyTo(outImg(Rect(W*2, 0, W, H)));
	//
	servo(2, centre_2);
	//
	return outImg;
}

string takeShot(bool gui)
{
	Mat image = getShot();
	if (gui) addgui(image);
	
	string name = getName();
	imwrite(name, image);
	return name;
}

string takePanoram(bool gui)
{
	Mat image = panoram();
	if (gui) addgui(image);
	string name = "P_" + getName();
	imwrite(name, image);
	return name;
}

string takeIRShot()
{
	while (!servo(2, 87)) {delay(100);}
	//while (!servo(1, 80)) {delay(100);}
	while (!servo(0, 87)) {delay(100);}

	double pre_s0 = getServo(0);
	double pre_s1 = getServo(1);
	double pre_s2 = getServo(2);
	
	delay(500);
	Mat image = getShot();
	Mat tMat = image.clone();
	
	const int32_t x_rez = fov/2.0;
	const int32_t y_rez = fov/2.0*0.75;
	Point2f data[x_rez][y_rez];
	
	//cout << x_rez << " " << y_rez << endl; 
	
	for(int32_t y = 0; y < y_rez; y++)
	{
		double yang = pre_s1 - fov*0.75*0.5 + (double)y/double(y_rez-1)*fov*0.75;
		//cout << "yang " << yang << endl;
		while (!servo(1, yang)) {delay(50);}
		while (!servo(2, pre_s2 + fov*0.5 - 1)) {delay(50);}
		delay(500);
			
		for(int32_t x = 0; x < x_rez; x++)
		{	
			double xang = pre_s2 + fov*0.5 - (double)x/double(x_rez-1)*fov - 1;
			while (!servo(2, xang)) {delay(10);}
			double dst = dist();
			data[x][y] = Point2f(mlxTemp(), dst);
			//cout << "x_y " << x << " " << y << " mlx "<< data[x][y].x << endl; 
		}
	}
	servo(2, pre_s2);
	servo(1, pre_s1);
	servo(0, pre_s0);
	
	double mlx_max = data[0][0].x;
	double mlx_min = data[0][0].x;
	double dst_max = data[0][0].y;
	double dst_min = data[0][0].y;
	
	for(int32_t y = 0; y < y_rez; y++)
	{
		for(int32_t x = 0; x < x_rez; x++)
		{	
			if(data[x][y].x > mlx_max) mlx_max = data[x][y].x;
			if(data[x][y].x < mlx_min) mlx_min = data[x][y].x;
			if(data[x][y].y > dst_max) dst_max = data[x][y].y;
			if(data[x][y].y < dst_min) dst_min = data[x][y].y;
		}
	}
	
	double sx = W/(double)x_rez;
	double sy = H/(double)y_rez;
	for(int32_t y = 0; y < y_rez; y++)
	{
		for(int32_t x = 0; x < x_rez; x++)
		{	
			Point2f p (sx*x, sy*y);
			double ltmp = (data[x][y].x-mlx_min)/(mlx_max-mlx_min);
			Scalar tcolor = Scalar(255,0,0)*(1-ltmp) + Scalar(0,0,255)*ltmp;
			rectangle(tMat, p, p + Point2f(sx, sy), tcolor, -1, CV_AA);
			ostringstream temp;
			temp << round(data[x][y].x*10)*0.1;
			putTextCorr(tMat, temp.str(), p + Point2f(sx, sy)*0.5, FONT_HERSHEY_DUPLEX, 0.4, Scalar(0,255,0), 1, CV_AA, Point2f(0.5, 0.5));
		}
	}
	
	addWeighted(image, 0.6, tMat, 0.4, 0.0, image);
	
	string name = "IR_" + getName();
	
	imwrite(name, image);
	return name;
}

void camerainit(void)
{
	float _intrinsic_matrix[9];
	_intrinsic_matrix[1]=0;
	_intrinsic_matrix[3]=0;
	_intrinsic_matrix[6]=0;
	_intrinsic_matrix[7]=0;
	_intrinsic_matrix[8]=1;
	_intrinsic_matrix[0]=1427.5175015217899;// (focus_lenX)
	_intrinsic_matrix[2]=655.48703876483887; // (PrincipalX)
	_intrinsic_matrix[4]=1430.2628843844473;// (focus_lenY)
	_intrinsic_matrix[5]=469.52248300618572;// (PrincipalY)
	Mat intrinsic_matrix = Mat(3,3,CV_32FC1,_intrinsic_matrix);

	float _distortion_coeffs[4];
	_distortion_coeffs[0]=-0.42764434667917478; // (Dist1)
	_distortion_coeffs[1]=0.21471658822535375;  // (Dist2)
	_distortion_coeffs[2]=0.003071822374420839;// (Dist3)
	_distortion_coeffs[3]=0.00060503585818268526; // (Dist4)
	Mat distortion_coeffs = Mat(4,1,CV_32FC1,_distortion_coeffs);
	
	cam_matrix = Mat(3,3,CV_32FC1,_intrinsic_matrix);
	Size S = Size(W, H);
	
	initUndistortRectifyMap(intrinsic_matrix, distortion_coeffs,
		Mat::eye(3, 3, CV_32F), cam_matrix, S, CV_32FC1, map_x, map_y);
		
	///////////////////////////
	//Mat image = getShot();//imread("01.01_22.41'19''.jpg", CV_LOAD_IMAGE_COLOR);
	//addgui(image);
	//imwrite("out.jpg", image);
	//imwrite(getName(), Test());
}
