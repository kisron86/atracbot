#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>

using namespace std;
using namespace cv;

string cascadeName = "haarcascade_frontalface_default.xml";
CascadeClassifier cascade;
double scale = 1.0;

int main(){

  // Create a VideoCapture object and use camera to capture the video
  VideoCapture cap(0); 
  cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

  Mat frame;
  int i;

  // Check if camera opened successfully
  if(!cap.isOpened())
  {
    cout << "Error opening video stream" << endl; 
    return -1; 
  } 

  // Default resolution of the frame is obtained.The default resolution is system dependent. 
  int frame_width = cap.get(CV_CAP_PROP_FRAME_WIDTH); 
  int frame_height = cap.get(CV_CAP_PROP_FRAME_HEIGHT); 
  
   while(1)
  {  
    // Capture frame-by-frame 
    cap >> frame;
    if (frame.empty())
      break;
    
    vector<Rect> gopro;
    Mat small_img_gray, small_img(cvRound(frame.rows / scale), cvRound(frame.cols / scale), CV_8UC1);
		Point center;
    
    if (!cascade.load(cascadeName))
		{
			printf("--(!)Error loading cascade, please change cascade_name in source code.\n");
			return 0;
		};

    resize(frame, small_img, small_img.size(), 0, 0, 1);
		cvtColor(small_img, small_img_gray, CV_BGR2GRAY);
		equalizeHist(small_img_gray, small_img_gray);

    cascade.detectMultiScale(small_img_gray, gopro, 1.5, 4.5, 0 | CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(24 / scale, 24 / scale));
    for (i = 0; i < gopro.size(); i++)
      cv::rectangle(frame, cvPoint(cvRound(gopro[0].x)*scale, cvRound(gopro[0].y)*scale), cvPoint((cvRound(gopro[0].x + gopro[0].width)*scale), (cvRound(gopro[0].y + gopro[0].height)*scale)), cv::Scalar(255,0 , 0), 2, 8, 0);
      
		cv::imshow("Object Detection", frame);

		cv::waitKey(10);
 
    // Press  ESC on keyboard to  exit
    char c = (char)waitKey(1);
    if( c == 27 ) 
      break;
  }

  // When everything done, release the video capture and write object
  cap.release();
  //video.release();

  // Closes all the windows
  destroyAllWindows();
  return 0;
}