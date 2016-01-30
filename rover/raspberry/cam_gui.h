#ifndef CAMERA_GUI
#define CAMERA_GUI
#include <cstring>
#include "opencv2/opencv.hpp"
extern void addgui(cv::Mat);
extern void putTextCorr(cv::Mat& img, const std::string& text, cv::Point org, int32_t fontFace, double fontScale, cv::Scalar color, int32_t thickness=1, int32_t lineType=8, cv::Point2f corr = cv::Point2f(0, 0));
#endif