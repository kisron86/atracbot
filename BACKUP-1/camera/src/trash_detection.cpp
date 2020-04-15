#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <time.h> // to calculate time needed
#include <limits.h> // to get INT_MAX, to protect against overflow

#include <iomanip>
#include <iostream>
#include <stdlib.h>
//#include <Windows.h>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <fstream>

#include <sstream>
#include <iomanip>

//#include <conio.h>
//#include "HOG.h"
//#include "glcm.h"
#include <opencv2/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/core/mat.hpp>

//#include "StereoGrab.h" // deklaration camera
//#include "StereoFunction.h" // funsi eksecukis stereo

using namespace cv;
using namespace std;

Mat mag, ang;
char buffer[1000];
char buffer1[200];
Mat imgPar;
Mat reduced(1, 144, CV_32F);
int counter = 0, imcount = 0;
Size cutRoi(8, 8);
Mat imgToSvm(1, 12, CV_32F);
String className;
vector<float> vHOG;
vector<float> vec_glcm;
vector<float> vectorHogGlcm;
Mat ClassImg;
Mat image;
Mat pre_crop_image;
Mat crop_image;
int thresholdC = 100;
Point roi_center;
vector<float> meas_dist;
int flag_class_result = 0, flag_print = 0, flag_bukan_sampah = 1;
int f_counter = 0;


vector < vector <float> > vecSVM;
//CvSVM svm;

/// Parameter Feature Extraction
Size sizes(32, 32);
int nFile = 50; // n-file to extract
string folder = "D:\\Project\\TA\\Data_Trainer\\Clip_Images\\DataTraining\\Data_Testing\\";  // folder input
string suffix = ".jpg";
int lCount = 0;
int nameCount = 0;
int ppp = 0, nnn = 0, net = 0;

/// Parameter Object Detection
String waste_cascade_name = "D:\\Project\\TA\\Debug\\waste.xml";
//CascadeClassifier waste_cascade;
string window_name_detect = "Capture - waste detection";
Mat gray;
int fps_counter = 0, fps_max = 0;
RNG rng(12345);
vector<Rect> waste;
Point roi_top; Point roi_bot;
Mat frame_cpy;
int run = 0;

/// Parameter Tracking
Mat res;
bool found = false;
double ticks = 0;
int notFoundCount = 0;
int initialize = 0;
Point track_roi_top; Point track_roi_bot;

/// Parameter enchaneTracking
int detect_roi_top = 0;
int detect_roi_bot = 0;
int detect_roi_left = 0;
int detect_roi_right = 0;
Mat enchanceImage;

// >>>> Kalman Filter
int stateSize = 6;
int measSize = 4;
int contrSize = 0;
unsigned int type = CV_32F;
KalmanFilter kf(stateSize, measSize, contrSize, type);

Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
                //Mat procNoise(stateSize, 1, type)
                // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

                // Transition State Matrix A
                // Note: set dT at each processing step!
                // [ 1 0 dT 0  0 0 ]
                // [ 0 1 0  dT 0 0 ]
                // [ 0 0 1  0  0 0 ]
                // [ 0 0 0  1  0 0 ]
                // [ 0 0 0  0  1 0 ]
                // [ 0 0 0  0  0 1 ]
                // <<<< Kalman Filter

#define CALIBRATION 0

//StereoGrab* grab= new StereoGrab();
//StereoFunction* stereoFunc = new StereoFunction();


int main(int argc, char **argv)
{
  ros::init(argc, argv, "trash_detection");
  ros::NodeHandle nh;

  ROS_INFO("Hello world!");
}
