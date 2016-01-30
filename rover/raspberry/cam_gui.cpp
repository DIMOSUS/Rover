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

#define byte uint8_t
#define H 972
#define W 1296
#define fov 47.5
#define fov_cor 0.0
#define PI 3.1415926535897932384626433832795

using namespace std;
using namespace cv;

void putTextCorr(Mat& img, const string& text, Point org, int32_t fontFace, double fontScale, Scalar color, int32_t thickness=1, int32_t lineType=8, Point2f corr = Point2f(0, 0))
{
	int32_t baseline = 0;
	Size textSize = getTextSize(text, fontFace, fontScale, thickness, &baseline);
	Point2f c = Point2f(textSize.width, -textSize.height/2);
	c = Point2f(c.x*corr.x, c.y*corr.y);
	putText(img, text, Point2f(org.x , org.y) - c, fontFace, fontScale, color, thickness, lineType);
}

void getRotImg(const string file, Mat dst, Point2f pos, double angle)
{
	Mat right = imread(file, CV_LOAD_IMAGE_UNCHANGED);
	cvtColor(right, right, CV_BGRA2BGR);
	
	Point2f center(right.cols/2, right.rows/2);
	Mat rot = getRotationMatrix2D(center, -angle, 1.0);
	Rect bbox = RotatedRect(center,right.size(), -angle).boundingRect();
	rot.at<double>(0,2) += bbox.width/2.0 - center.x;
    rot.at<double>(1,2) += bbox.height/2.0 - center.y;
	Mat outmat;
	warpAffine(right, outmat, rot, bbox.size(), INTER_CUBIC);
	resize(outmat, outmat, Size((bbox.width/4)*2, (bbox.height/4)*2), 0.5, 0.5, INTER_AREA);
	Rect outRect(pos.x-outmat.cols/2, pos.y-outmat.rows/2, outmat.cols, outmat.rows);
	scaleAdd(outmat, 1, dst(outRect), dst(outRect));
	scaleAdd(outmat, 1, dst(outRect), dst(outRect));
}

void addRule(Mat dsc, Point2f p0, Point2f p1, double value, Scalar color, double minv = -90, double maxv = 90)
{
	line(dsc, Point(p0.x, p0.y),Point(p1.x, p1.y), color, 1, CV_AA);

	Point2f t = p1 - p0;
	double l = sqrt(t.x*t.x + t.y*t.y);
	t.x = t.x / l;
	t.y = t.y / l;
	Point2f n(t.y, t.x);
	if (n.ddot(Point2f(-1, 1)) > 0)
	{
		n.x *= -1;
		n.y *= -1;
	}
	
	for(int32_t i = minv; i <= maxv; i=i+45)
	{
		double lv = (i - minv)/(maxv-minv);
		Point2f pos(lerp(p0.x,p1.x,lv), lerp(p0.y,p1.y,lv));
		line(dsc, pos, pos+n*3, color, 1, CV_AA);
		
		ostringstream temp;
		temp<<i;
		putTextCorr(dsc, temp.str(), pos+n*5, FONT_HERSHEY_DUPLEX, 0.25, color, 1, CV_AA, Point2f(t.x*0.5, t.y*0.5));
	}
	
	double lv = (value - minv)/(maxv-minv);
	Point2f pos(lerp(p0.x,p1.x,lv), lerp(p0.y,p1.y,lv));
	Point2f pos1(pos-n*7+t*4);
	Point2f pos2(pos-n*7-t*4);
	
	line(dsc, pos, pos1, color, 1, CV_AA);
	line(dsc, pos, pos2, color, 1, CV_AA);
	line(dsc, pos1, pos2, color, 1, CV_AA);
	
	ostringstream temp;
	temp<<(int32_t)round(value);
	putTextCorr(dsc, temp.str(), pos - n*16 + Point2f(n.x*7, 0), FONT_HERSHEY_DUPLEX, 0.3, color, 1, CV_AA, Point2f(0.5+n.x*0.5, 0.5*n.x));
}

void addGraph(Mat image, Point2f pos, string name, Scalar color, volatile double* data, int32_t count)
{
	int32_t siz = 100;
	double minv = data[0], maxv = data[0];
	for (int32_t i = 1; i < count; i++)
	{
		if (data[i] > maxv) maxv = data[i];
		if (data[i] < minv) minv = data[i];
	}
	
	int32_t gmin = floor(minv-data[0]*0.02);
	int32_t gmax = ceil(maxv+data[0]*0.02);
	
	double prex = siz;
	double prey = (data[0]-gmin)/((double)(gmax-gmin))*((double)siz);
	
	for (int32_t i = 1; i < count; i++)
	{
		double x = siz-((double)i)/((double)TableSize)*((double)siz);
		double y = (data[i]-gmin)/((double)(gmax-gmin))*((double)siz);
		line(image, pos+Point2f(prex, siz-prey), pos+Point2f(x, siz-y), color, 1, CV_AA);
		prex = x;
		prey = y;
	}
	
	rectangle(image, pos, pos + Point2f(siz, siz), color, 1, CV_AA);
	
	ostringstream temp;
	temp<<gmin;
	putTextCorr(image, temp.str(), pos + Point2f(-4, siz), FONT_HERSHEY_DUPLEX, 0.3, color, 1, CV_AA, Point2f(1, 0.5));
	temp.str("");
	temp<<gmax;
	putTextCorr(image, temp.str(), pos + Point2f(-4, 0), FONT_HERSHEY_DUPLEX, 0.3, color, 1, CV_AA, Point2f(1, 0.5));
	
	putTextCorr(image, "0h", pos + Point2f(siz, siz+8), FONT_HERSHEY_DUPLEX, 0.3, color, 1, CV_AA, Point2f(0.5, 1));
	putTextCorr(image, "-1h", pos + Point2f(0, siz+8), FONT_HERSHEY_DUPLEX, 0.3, color, 1, CV_AA, Point2f(0.5, 1));
	
	temp.str("");
	temp << name;
	temp<<" ";
	temp<<round(data[0]*10)*0.1;
	
	putTextCorr(image, temp.str(), pos + Point2f(0, -10), FONT_HERSHEY_DUPLEX, 0.4, color, 1, CV_AA);
}

void addgui(Mat image)
{
	if (lux() < 0.1)
	{
		Mat gs_rgb(image.size(), CV_8UC1);
		
		cvtColor(image, gs_rgb, CV_BGR2GRAY);
		cvtColor(gs_rgb, image, CV_GRAY2RGB);
	}
	
	Scalar guiColor(100,255,55);
	Point2f guiRT((image.cols>W)*W+W, 0);
	Point2f guiLT(guiRT.x - W, guiRT.y);
	Point2f guiLD(guiRT.x - W, guiRT.y+H);
	
	Mat tMat = image.clone();
	rectangle(tMat, guiLT, guiLT + Point2f(160, 725), Scalar(0,0,0), -1, CV_AA);
	rectangle(tMat, guiRT, guiRT + Point2f(-330, 160), Scalar(0,0,0), -1, CV_AA);
	rectangle(tMat, guiRT, guiRT + Point2f(-165, 390), Scalar(0,0,0), -1, CV_AA);
	rectangle(tMat, guiLD+Point2f(0, -20), guiLD + Point2f(W, -60), Scalar(0,0,0), -1, CV_AA);
	addWeighted(image, 0.6, tMat, 0.4, 0.0, image);
	
	//yaw
	{
		line(image, Point(0,H - H/20), Point(W*3,H - H/20), guiColor, 1, CV_AA);
		for(int32_t i = -100; i <= 100; i = i+10)
		{
			double x = W/fov*i + guiLT.x + W*0.5;
			ostringstream temp;
			temp<<i;
			putTextCorr(image, temp.str(), Point(x, H-H/20+20), FONT_HERSHEY_DUPLEX, 0.4, guiColor, 1, CV_AA, Point2f(0.5, 0));
		}
		for(int32_t i = -100; i <= 100; i = i+1)
		{
			double lh = 3;
			if (i%5 == 0) lh*=1.7;
			if (i%10 == 0) lh*=1.5;
			double x = W/fov*i + guiLT.x + W*0.5;
			line(image, Point(x, H-H/20+lh), Point(x,H-H/20),guiColor, 1, CV_AA);
		}
	}
	
	//pitch, roll
	{
		getRotImg("img/right.png", image, guiRT - Point2f(255, -75), pitch*180/PI);
		getRotImg("img/back.png", image, guiRT - Point2f(85, -75), roll*180/PI);
		addRule(image, guiRT - Point2f(160, -30), guiRT - Point2f(160, -140), pitch*180/PI, guiColor);
		addRule(image, guiRT - Point2f(140, -160), guiRT - Point2f(30, -160), roll*180/PI, guiColor);
	}
	//servos
	for (int32_t i = 0; i < 3; i++)
	{
		Point2f refp = guiRT - Point2f(140, -240-60*i);
		addRule(image, refp, refp - Point2f(-110, 0), getServo(2-i), guiColor, 0, 180);
		ostringstream temp;
		temp << "Servo " << int32_t(2-i);
		putText(image, temp.str(), refp - Point2f(4, 16), FONT_HERSHEY_DUPLEX, 0.35, guiColor, 1, CV_AA);
	}
	
	//dist/mlx/lux
	{
		circle(image, guiLT + Point2f(W/2,H/2), 40, guiColor, 1, CV_AA);
		ostringstream temp;
		temp << round(mlxTemp()*100)*0.01 << "'C";
		putTextCorr(image, temp.str(), guiLT + Point2f(W/2,H/2 - 47), FONT_HERSHEY_DUPLEX, 0.4, guiColor, 1, CV_AA, Point2f(0.5,0.5));
		temp.str("");
		double d = dist();
		if (d < 250) temp << round(d)*0.01 << "m";
		else temp << ">2.5m";
		putTextCorr(image, temp.str(), guiLT + Point2f(W/2,H/2 + 51), FONT_HERSHEY_DUPLEX, 0.4, guiColor, 1, CV_AA, Point2f(0.5,0.5));
		temp.str("");
		temp << round(lux()) << "Lux";
		putTextCorr(image, temp.str(), guiLT + Point2f(W/2+44,H/2), FONT_HERSHEY_DUPLEX, 0.4, guiColor, 1, CV_AA, Point2f(0.0,0.5));
	}

	//bat
	{
		Point2f refp = guiLT + Point2f(40,30);
		const double b_w = 75, b_h = 20;
		rectangle(image, refp, refp + Point2f(b_w, b_h), guiColor, 1, CV_AA);
		ostringstream temp;
		temp << vbat() << "v/" << cbat(Temp) << "%";
		putTextCorr(image, temp.str(), refp + Point2f(b_w/2, b_h/2+2), FONT_HERSHEY_DUPLEX, 0.4, guiColor, 1, CV_AA, Point2f(0.5,0.5));
		refp += Point2f(b_w,b_h/3);
		rectangle(image, refp, refp + Point2f(b_w*0.05, b_h/3), guiColor, 1, CV_AA);
	}
	
	//gps
	{
		Point2f refp = guiLT + Point2f(40,80);
		ostringstream temp;
		temp << "SAT:" << Satellites << "  ALT:" << Altitude << "m";
		putTextCorr(image, temp.str(), refp, FONT_HERSHEY_DUPLEX, 0.4, guiColor, 1, CV_AA, Point2f(0.0,0.5));

	}
	
	//temp
	//hum
	//pre
	//mag
	{
		int32_t ya = 150;
		addGraph(image, guiLT + Point2f(40, 150+ya*0), "Hum", guiColor, &table[0][0], t_counter);
		addGraph(image, guiLT + Point2f(40, 150+ya*1), "Tmp", guiColor, &table[1][0], t_counter);
		addGraph(image, guiLT + Point2f(40, 150+ya*2), "Prs", guiColor, &table[2][0], t_counter);
		addGraph(image, guiLT + Point2f(40, 150+ya*3), "Mag", guiColor, &table[3][0], t_counter);
	}
	
	
	//time
	//pos
}