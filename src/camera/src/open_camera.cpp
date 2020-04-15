#include "ros/ros.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

char fpsr_string[10];
char fpsl_string[10];

int main(){
  VideoCapture capr(2);  // kamera kanan
  VideoCapture capl(4);  // kamera kiri
  capr.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
  capr.set(CV_CAP_PROP_FRAME_WIDTH, 320);
  capl.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
  capl.set(CV_CAP_PROP_FRAME_WIDTH, 320);

  if(!capr.isOpened()) {
    cout << "Cannot open the video file. \n";
    return -1;
  }

  if(!capl.isOpened()) {
    cout << "Cannot open the video file. \n";
    return -1;
  }

  double fpsr;
  double fpsl;

  namedWindow("Kamera_Kanan",CV_WINDOW_AUTOSIZE);
  namedWindow("Kamera_Kiri",CV_WINDOW_AUTOSIZE);
  moveWindow("Kamera_Kanan",725,200);
  moveWindow("Kamera_Kiri",400,200);

  Mat framer;
  Mat framel;

  while(1){

    fpsr = capr.get(CV_CAP_PROP_FPS);
    fpsl = capl.get(CV_CAP_PROP_FPS);

    if (!capr.read(framer)){
      cout<<"\n Cannot read the video file. \n";
      break;
    }
    if (!capl.read(framel)){
      cout<<"\n Cannot read the video file. \n";
      break;
    }

    sprintf(fpsr_string,"%2.0lf", fpsr);
    sprintf(fpsl_string,"%2.0lf", fpsl);
    putText(framer,fpsr_string,Point2f(5,100),FONT_HERSHEY_PLAIN,2,Scalar(0,0,255),2,8,false);
    putText(framel,fpsl_string,Point2f(5,100),FONT_HERSHEY_PLAIN,2,Scalar(0,0,255),2,8,false);
    imshow("Kamera_Kanan", framer);
    imshow("Kamera_Kiri", framel);
    if(waitKey(30) == 27){ break; }   //27 ASCII Esc
  }
  return 0;
}
