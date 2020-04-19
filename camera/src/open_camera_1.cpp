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
  VideoCapture capr(4);  // kamera kanan
  capr.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
  capr.set(CV_CAP_PROP_FRAME_WIDTH, 320);

  if(!capr.isOpened()) {
    cout << "Cannot open the video file. \n";
    return -1;
  }

  double fpsr;
  double fpsl;

  namedWindow("Kamera_Kanan",CV_WINDOW_AUTOSIZE);
  moveWindow("Kamera_Kanan",725,200);

  Mat framer;

  while(1){

    fpsr = capr.get(CV_CAP_PROP_FPS);

    if (!capr.read(framer)){
      cout<<"\n Cannot read the video file. \n";
      break;
    }

    sprintf(fpsr_string,"%2.0lf", fpsr);
    sprintf(fpsl_string,"%2.0lf", fpsl);
    putText(framer,fpsr_string,Point2f(5,100),FONT_HERSHEY_PLAIN,2,Scalar(0,0,255),2,8,false);
    imshow("Kamera_Kanan", framer);
    if(waitKey(30) == 27){ break; }   //27 ASCII Esc
  }
  return 0;
}
