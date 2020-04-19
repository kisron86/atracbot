#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;

string filexml = "/home/kisron/catkin_workspace/src/kevinpkg/src/cascade_40x80.xml";
int banyak();
int satu();

int main(int argc, char **argv)
{
  ros::init(argc, argv, "detection1");
  ros::NodeHandle nh;

    banyak();
    //satu();
    return 0;
  }
  int banyak() {
    VideoCapture capture(2);
    if (!capture.isOpened())
      printf("Error when reading file");
    //namedWindow("window", 1);
    for (;;)
    {
      Mat image; capture >> image;
      if (image.empty()) break;

      // Load Face cascade (.xml file)
      CascadeClassifier face_cascade;
      face_cascade.load(filexml);
      if (!face_cascade.load(filexml))
      {
        cerr << "Error Loading XML file" << endl;
        return 0;
      }

      // Detect faces
      std::vector<Rect> faces;
      face_cascade.detectMultiScale(image, faces, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, Size(24, 24));

      // Draw circles on the detected faces
      for (int i = 0; i < faces.size(); i++)
      {
        Point center(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5);
        ellipse(image, center, Size(faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar(255, 0, 255), 4, 8, 0);
      }
      // for (i = 0; i < gopro.size(); i++)
        //			cv::rectangle(frame, cvPoint(cvRound(gopro[0].x)*scale, cvRound(gopro[0].y)*scale), cvPoint((cvRound(gopro[0].x + gopro[0].width)*scale), (cvRound(gopro[0].y + gopro[0].height)*scale)), cv::Scalar(255,0 , 0), 2, 8, 0);
        //

      imshow("Detected Apple", image);
      double fps = capture.get(CV_CAP_PROP_FPS);
      cout << fps << endl;
      //waitKey(1);
      if(cv::waitKey(30) == 27){
        break;
      }
    }
  }

  int satu() {
    cv::VideoCapture cap(2);
    cv::Mat frame;
    int i;
    std::string cascadeName = filexml;
    cv::CascadeClassifier cascade;
    double scale = 1;
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

      cv::imshow("Apple Detection", frame);
      //cv::waitKey(10);
      if(cv::waitKey(30) == 27){
        break;
      }
    }
}
