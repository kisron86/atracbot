#include <ros/ros.h>
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


//std::string cascadeName = "/home/kisron/catkin_workspace/src/camera/src/haarcascade_frontalface_default.xml";
static std::string cascadeName = "/home/kisron/Documents/cpp-test/opencv-haar-classifier-training-master/classifier_kertas/cascade.xml";

static cv::CascadeClassifier cascade;
static double scale = 1;

//Offline
//#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

int main()
{
  Mat image;
  image = imread("/home/kisron/Documents/testing_kertas/0000.png", CV_LOAD_IMAGE_COLOR);	// Ganti myface.jpg dengan gambar anda sendiri
  namedWindow("window1", 1);
  imshow("window1", image);

  // Load Face cascade (.xml file)
  CascadeClassifier face_cascade;
  face_cascade.load(cascadeName);	// Letakkan file xml tersebut di satu folder yang sama dengan debug/release

  if (face_cascade.empty())
  {
    cerr << "Error Loading XML file" << endl;
    return 0;
  }

  // Detect faces
  std::vector<Rect> faces;
  face_cascade.detectMultiScale(image, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(30, 30));

  // Draw circles on the detected faces
  for(int i = 0; i < faces.size(); i++)
  {
    Point center(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5);
    ellipse(image, center, Size(faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
  }
  imshow("Detected Object", image);
  waitKey(0);
  return 0;
}
/*
int main(int argc, char **argv)
{
  ros::init(argc, argv, "trash_detection");
  ros::NodeHandle nh;

  cv::VideoCapture cap(0);
      cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
      cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
      cv::Mat frame;
      int i;

      while (1)
      {
          cap >> frame;

          if (frame.empty())
              break;

          vector<cv::Rect> gopro;
          cv::Mat small_img_gray, small_img(cvRound(frame.rows / scale), cvRound(frame.cols / scale), CV_8UC1);
          cv::Point center;

          if (!cascade.load(cascadeName))
          {
              printf("--(!)Error loading cascade, please change cascade_name in source code.\n");
              return 0;
          };

          cv::resize(frame, small_img, small_img.size(), 0, 0, 1);
          cvtColor(small_img, small_img_gray, CV_BGR2GRAY);
          cv::equalizeHist(small_img_gray, small_img_gray);

          cascade.detectMultiScale(small_img_gray, gopro, 1.5, 4.5, 0 | CV_HAAR_FIND_BIGGEST_OBJECT, cv::Size(24 / scale, 24 / scale));

          for (i = 0; i < gopro.size(); i++)
              cv::rectangle(frame, cvPoint(cvRound(gopro[0].x)*scale, cvRound(gopro[0].y)*scale), cvPoint((cvRound(gopro[0].x + gopro[0].width)*scale), (cvRound(gopro[0].y + gopro[0].height)*scale)), cv::Scalar(255,0 , 0), 2, 8, 0);

          cv::imshow("Object Detection", frame);

          cv::waitKey(10);
      }

      return 0;
}
*/
