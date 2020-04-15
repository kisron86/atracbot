#ifndef MY_SUPER_ROSCPP_LIBRARY_H
#define MY_SUPER_ROSCPP_LIBRARY_H
#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/core/mat.hpp>

#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

#include <ros/ros.h>

void sayHello();

void glcm(const Mat img, vector<float> &vec_energy, bool isShow, bool isPrint);

#endif
