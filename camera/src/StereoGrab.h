//stereograb.h file header ketika mendefinisi
// untuk variable dan fungsi
#pragma once
#include<iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define WIDTH 320
#define HEIGHT 240


struct StereoGrab{

	void stereoGrabInitFrames();
	void stereGrabFrames();
	void stereoGrabStopCam();
	IplImage* imageLeft;
	IplImage* imageRight;

};