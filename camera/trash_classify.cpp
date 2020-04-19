#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <time.h> // to calculate time needed
#include <limits.h> // to get INT_MAX, to protect against overflow

#ifndef DWORD
#define WINAPI
typedef unsigned long DWORD;
typedef short WCHAR;
typedef void * HANDLE;
#define MAX_PATH    PATH_MAX
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned int BOOL;
#endif

#ifndef WIN32_NO_STATUS
# define WIN32_NO_STATUS
#endif

#include <iomanip>
#include <iostream>
#include <stdlib.h>
//#include <windows.h>

#ifndef WIN32_NO_STATUS
# define WIN32_NO_STATUS
#endif

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

//#include "StereoGrab.h" // deklaration camera
//#include "StereoFunction.h" // funsi eksecukis stereo


//#include <conio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/core/mat.hpp>
#include "HOG.h"
#include "glcm.h"

using namespace cv::ml;


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
Ptr<SVM> svm = SVM::create();

/// Parameter Feature Extraction
Size sizes(32, 32);
int nFile = 50; // n-file to extract
string folder = "D:\\Project\\TA\\Data_Trainer\\Clip_Images\\DataTraining\\Data_Testing\\";  // folder input
string suffix = ".jpg";
int lCount = 0;
int nameCount = 0;
int ppp = 0, nnn = 0, net = 0;

/// Parameter Object Detection
String waste_cascade_name = "/home/kisron/catkin_workspace/src/camera/src/FeatureXML/waste.xml";
CascadeClassifier waste_cascade;
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

double ecu_top=0.0;
double ecu_bot=0.0;
float distance_obj;
int f_detect = 10;


void thresh_callback(int, void*) {
    Canny(enchanceImage, enchanceImage, thresholdC, thresholdC * 2, 3);
}

void detectAndDisplay(Mat frame){
    Mat frame_gray;
    int red, green, blue;

    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
    equalizeHist(frame_gray, gray);
    //imshow("frame_gray", frame_gray);
    //imshow("Equalize", gray);

    //-- Detect waste
    waste_cascade.detectMultiScale(frame_gray, waste, 1.1, 2, 0 | CV_HAAR_FIND_BIGGEST_OBJECT, Size(24, 24));

    for (size_t i = 0; i < waste.size(); i++)
    {

        Point center(waste[i].x + waste[i].width / 2, waste[i].y + waste[i].height / 2);
        Point p_atas(waste[i].x, waste[i].y);
        Point p_bawah(waste[i].x + waste[i].width, waste[i].y + waste[i].height);
        roi_top = p_atas; roi_bot = p_bawah;

        Point center1(waste[i].x, waste[i].y);

        rectangle(frame, p_atas, p_bawah, Scalar(0, 255, 0), 2);
        //ellipse(frame, center, Size(3, 3), 0, 0, 360, Scalar(255, 0, 0), 2, 8, 0);
        //ellipse(frame, center, Size(waste[i].width / 2, waste[i].height / 2), 0, 0, 360, Scalar(0, 0, 255), 1, 8, 0);
        /*Vec3b intensity = frame.at<Vec3b>((waste[i].y + waste[i].height / 2), (waste[i].x + waste[i].width / 2));
        blue = intensity.val[0]; green = intensity.val[1]; red = intensity.val[2];
        String r = to_string(red);  String g = to_string(green); String b = to_string(blue);
        String rgb = "R:" + r + "  G:" + g + "  B:" + b;
        //putText(frame, rgb, p_atas, FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 0, 255), 1);*/

    }
    //-- Show what you got
    //imshow(window_name, frame);
    //printf("R: %d | G: %d | B: %d \n", red, green, blue);*/
}

void tracking() {

    double precTick = ticks;
    ticks = (double)getTickCount();

    double dT = (ticks - precTick) / getTickFrequency(); //seconds

    //cout << "Start Tracking..." << endl;
    setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //setIdentity(kf.processNoiseCov, Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    setIdentity(kf.measurementNoiseCov, Scalar(1e-1));

    if (found)
    {
        //cout << "found" << endl;

        // >>>> Matrix A
        kf.transitionMatrix.at<float>(2) = dT;
        kf.transitionMatrix.at<float>(9) = dT;
        // <<<< Matrix A

        //cout << "dT:" << dT << endl;

        state = kf.predict();
        //cout << "State post:" << endl << state << endl;

        Rect predRect;
        predRect.width = state.at<float>(4);
        predRect.height = state.at<float>(5);
        predRect.x = state.at<float>(0) - predRect.width / 2;
        predRect.y = state.at<float>(1) - predRect.height / 2;

        track_roi_top = Point(predRect.x, predRect.y);
        track_roi_bot = Point(predRect.x + predRect.width, predRect.y + predRect.height);

        if (track_roi_bot.x >= 320) track_roi_bot.x = 320;
        if (track_roi_bot.y >= 240) track_roi_bot.y = 240;
        if (track_roi_top.x <= 0) track_roi_top.x = 0;
        if (track_roi_top.y <= 0) track_roi_top.y = 0;

        Point center;
        center.x = state.at<float>(0);
        center.y = state.at<float>(1);
        //circle(frame_cpy, center, 2, CV_RGB(255, 0, 0), -1);
        rectangle(frame_cpy, predRect, CV_RGB(255, 0, 0), 2);

        //PointCenter(center);
        //ellipse(frame_cpy, center, Size(3, 3), 0, 0, 360, Scalar(255, 0, 0), 2, 8, 0);

        /*stringstream sstr;
        sstr << "(" << center.x << "," << center.y << ")";
        putText(frame_cpy, sstr.str(), Point(center.x + 3, center.y - 3), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 20, 20), 2);*/
    }

    // >>>>> Detection result
    for (size_t i = 0; i < waste.size(); i++)
    {

        rectangle(res, waste[i], CV_RGB(0, 255, 0), 2);

        Point center;
        center.x = waste[i].x + waste[i].width / 2;
        center.y = waste[i].y + waste[i].height / 2;
        circle(res, center, 2, CV_RGB(20, 150, 20), -1);

        stringstream sstr;
        sstr << "(" << center.x << "," << center.y << ")";
        //putText(frame_cpy, sstr.str(), Point(center.x + 3, center.y - 3), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20, 150, 20), 2);
    }

    // >>>>> Kalman Update
    if (waste.size() == 0)
    {
        notFoundCount++;
        //cout << "notFoundCount:" << notFoundCount << endl;
        if (notFoundCount >= 10)
        {
            found = false;
        }
        /*else
        kf.statePost = state;*/
    }
    else
    {
        notFoundCount = 0;

        meas.at<float>(0) = waste[0].x + waste[0].width / 2;
        meas.at<float>(1) = waste[0].y + waste[0].height / 2;
        meas.at<float>(2) = (float)waste[0].width;
        meas.at<float>(3) = (float)waste[0].height;

        if (!found) // First detection!
        {
            // >>>> Initialization
            kf.errorCovPre.at<float>(0) = 1; // px
            kf.errorCovPre.at<float>(7) = 1; // px
            kf.errorCovPre.at<float>(14) = 1;
            kf.errorCovPre.at<float>(21) = 1;
            kf.errorCovPre.at<float>(28) = 1; // px
            kf.errorCovPre.at<float>(35) = 1; // px

            state.at<float>(0) = meas.at<float>(0);
            state.at<float>(1) = meas.at<float>(1);
            state.at<float>(2) = 0;
            state.at<float>(3) = 0;
            state.at<float>(4) = meas.at<float>(2);
            state.at<float>(5) = meas.at<float>(3);
            // <<<< Initialization

            kf.statePost = state;

            found = true;
        }
        else
            //cout << "Kalman Correction" << endl;
            kf.correct(meas); // Kalman Correction

    }
    // <<<<< Kalman Update

    // Final result
    //imshow("Tracking", res);
}



void Classifier() {
    ///Classification whether data is positive, negative or non sampah
    int result = svm->predict(imgToSvm);
    if (flag_print == 1) {
        cout << "Classification..." << " ";
        cout << "result: " << result << endl;
    }

    ///Count data
    if (result < 10) {
        ppp++;
        className = "#Organik";
        flag_bukan_sampah = 1;
    }
    else if (result < 29) {
        nnn++;
        className = "#Non Organik";
        flag_bukan_sampah = 1;
    }
    else if (result > 28) {
        net++;
        className = "#Bukan Sampah";
        flag_bukan_sampah = 0;
    }

    Point p_atas(5, 15);
    /// Draw class in Image
    if (flag_class_result == 1) {
        putText(ClassImg, className, track_roi_top, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);
        if(flag_bukan_sampah == 1)
            arrowedLine(ClassImg, Point(160, 240), roi_center, Scalar(220, 20, 60), 2, 8, 0);
    }

    ///Write image
    /*sprintf_s(buffer, "D:\\Project\\TA\\Data_Trainer\\Clip_Images\\DataTraining\\Hasil_Testing\\Test%u.png", nameCount++);
    //imwrite(buffer, ClassImg);
    //printf(" positive/negative/netral = (%d/%d/%d) \n", ppp, nnn, net); */
}

void print_distance_angle() {
    if (ecu_top > f_detect || ecu_top > f_detect) flag_class_result = 0;
    else {
        flag_class_result = 1;
        Point line_center(160, roi_center.x);
        double line_a = 240 - roi_center.y;
        double line_b = roi_center.x - 160;
        float meas_degre = atan2(line_a, line_b);
        meas_degre = meas_degre * 180 / 3.14;
        if (meas_degre > 90) meas_degre = (meas_degre - 90)*-1;
        else meas_degre = 90 - meas_degre;

        /*stringstream sstr;
        sstr << "(" << distance_obj << " : " << meas_degre << ")";*/
        sprintf(buffer, "(%.2f : %.2f)", distance_obj, meas_degre);
        if(flag_bukan_sampah==1)
            putText(ClassImg, buffer, Point(roi_center.x, roi_center.y), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 20, 20), 2);
    }
}

/*void ExtractFeature(Mat capt, int looper) {
    ///load images
    image = crop_image.clone();
    cvtColor(image, image, COLOR_BGR2GRAY);
    resize(image, image, sizes);
    if (image.empty()) cout << "No image loaded" << endl;

    /// begin parsing image to 16 cell 8*8
    for (int i = 0; i < image.rows; i += cutRoi.height) {
        for (int j = 0; j < image.cols; j += cutRoi.width) {
            Rect roi(i, j, cutRoi.width, cutRoi.height);
            Mat crop = image(roi);
            sprintf(buffer, "ROI\\roi%u.png", counter++);
            imwrite(buffer, crop);
        }
    }
    counter = 0;

    /// read the parsing images
    for (int x = 0; x < 16; x++) {
        string folderROI = "ROI\\roi";	// read folder
        string suffix = ".png";	// file type
        stringstream ss;
        ss << setw(0) << setfill('0') << counter++; // 0000, 0001, 0002, etc...
        string number = ss.str();

        string name = folderROI + number + suffix;
        imgPar = imread(name);
        if (imgPar.empty()) cout << "No parsing image loaded...!" << endl;

        /// Extract Feature using HOG
        Mat mag, ang;
        computeMagAngle(imgPar, mag, ang);

        Mat wHogFeature;
        computeHOG(mag, ang, wHogFeature, 9, true);
    }
    counter = 0;
    featureVecFullPrint(vHOG, looper, false);

    /// Extract Feature using glcm
    imshow("image", image);
    glcm(image, vec_glcm, false, false);
    waitKey(1);
}
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "trash_classify");
    ros::NodeHandle nh;
    time_t start, end;
    int counter = 0;
    double sec;
    double fps;
    /// Load HOG
    FileStorage read_FeatureXml("/home/kisron/catkin_workspace/src/camera/src/FeatureXML/eigenValues_All_2.xml", FileStorage::READ);
    if (read_FeatureXml.isOpened()) cout << "HOGSampah xml loaded" << endl;

    ///Feature Mat
    Mat eMat;
    read_FeatureXml["Descriptor_of_images"] >> eMat;
    read_FeatureXml.release();

    /// Load trained SVM xml data
    //load opencv2
    //svm.load("/home/kisron/catkin_workspace/src/camera/src/FeatureXML/Svm_Feature_Linear_All_2.xml");

    //load opencv3
    Ptr<SVM> svm = Algorithm::load<SVM>("/home/kisron/catkin_workspace/src/camera/src/FeatureXML/Svm_Feature_Linear_All_2.xml");
    //  -- 1. Load the cascades
    if (!waste_cascade.load(waste_cascade_name)) { printf("--(!)Error loading xml\n"); };


    char choice='z';
        while(choice!='q'){
            // fps counter begin
            if (counter == 0){
                time(&start);
            }

            //stereoCorrelationControl();
            //grab->stereGrabFrames();
            //stereoFunc->stereoCorrelation(grab);
            if (cvWaitKey(1) == 27) {
                cout << "Capturing ROI..." << endl;
                sprintf(buffer1, "/home/kisron/Documents/Project_Bang_Salimi/DataTraining/img%u.png", imcount++);
                imwrite(buffer1, crop_image);
                imshow("Cropped image", crop_image);
            }

            if (cvWaitKey(1) == 32) {
                flag_print = 1;
            }

            Mat read_cam_left = imread("/home/kisron/Documents/Project_Bang_Salimi/bismillah vision 8 sukses/bismillah vision/cam_left.jpg");
            frame_cpy = read_cam_left.clone();
            enchanceImage = read_cam_left.clone();
            cv::cvtColor(enchanceImage, enchanceImage, CV_BGR2GRAY);
            cv::blur(enchanceImage, enchanceImage, Size(3, 3));
            pre_crop_image = frame_cpy.clone();
            res = frame_cpy.clone();

            cv::createTrackbar(" Canny thresh:", "Canny", &thresholdC, 255, thresh_callback);
            thresh_callback(0, 0);

            if (!frame_cpy.empty()) {
                if(run==0 || run%5==0)
                detectAndDisplay(frame_cpy);
                tracking(); run++;
                //enchanceTracking();
                imshow("Canny", enchanceImage);
                if (run == 10000) run = 0;
                ClassImg = frame_cpy.clone();

                if (track_roi_top.x <= 0 || track_roi_top.y <= 0 || track_roi_bot.x <= 0 || track_roi_bot.y <= 0) {
                    Rect ROI(roi_top, roi_bot);
                    crop_image = pre_crop_image(ROI);
                }
                else {
                    Rect ROI(track_roi_top, track_roi_bot);
                    crop_image = pre_crop_image(ROI);
                }

                imshow("crop_image", crop_image);

                //ExtractFeature(pre_crop_image, 0); /// Feature Extraction
                //reduceFeatureUsingPCAinSVM(eMat, reduced, vectorHogGlcm, false); /// Reduce HOG using PCA
                //for (auto i = vec_glcm.begin(); i != vec_glcm.end(); ++i) vectorHogGlcm.push_back(*i);
                //int no = 0;
                //for (auto i = vectorHogGlcm.begin(); i != vectorHogGlcm.end(); ++i) imgToSvm.at<float>(0, no++) = *i;
                //vec_glcm.clear(); vectorHogGlcm.clear(); /// Empty the vector
                //Classifier();
                //print_distance_angle();
            }

            time(&end);
            counter++;
            sec = difftime(end, start);

            fps = counter / sec;
            if (sec<60){
                if(flag_print==1) cout << "counter: " << counter << " sec: " << sec << " ";
            }

            /// Print fps
            String s_fps;
            Point p_fps(5, 15);
            s_fps = to_string(fps);
            s_fps = "fps" + s_fps;
            putText(ClassImg, s_fps, p_fps, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);

            /// Print frame counter
            Point p_fcount(260, 15);
            s_fps = to_string(f_counter++);
            s_fps = "#" + s_fps;
            putText(ClassImg, s_fps, p_fcount, FONT_HERSHEY_PLAIN, 1, Scalar(0, 255, 255), 2);

            /// overflow protection
            if (counter == (INT_MAX - 1000))
                counter = 0;
            ///fps counter end

            imshow("Classification", ClassImg);

        }

    return 0;
}




