#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

long frameCounter = 0;

std::time_t timeBegin = std::time(0);
int tick = 0;

int main(int argc, char **argv){
  ros::init(argc, argv, "stereoCapture");
  ros::NodeHandle nh;

  cv::VideoCapture capr(0);  // kamera kanan
  cv::VideoCapture capl(4);  // kamera kiri

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

  cv::Mat framer;
  cv::Mat framel;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher pubr = it.advertise("camera/right/image_raw",1);
  image_transport::Publisher publ = it.advertise("camera/left/image_raw",1);

  sensor_msgs::ImagePtr msgr;
  sensor_msgs::ImagePtr msgl;

  ros::Rate loop_rate(10);

  while (ros::ok()) {
    if (!capr.read(framer)){
      cout << "\n Cannot read the video file. \n";
      break;
    }
    if (!capl.read(framel)){
      cout<<"\n Cannot read the video file. \n";
      break;
    }

    msgr = cv_bridge::CvImage(std_msgs::Header(),"bgr8", framer).toImageMsg();
    msgl = cv_bridge::CvImage(std_msgs::Header(),"bgr8", framel).toImageMsg();
    imshow("Kamera_Kanan", framer);
    imshow("Kamera_Kiri", framel);
    frameCounter++;

    std::time_t timeNow = std::time(0) - timeBegin;

    if (timeNow - tick >= 1)
    {
        tick++;
        cout << "Frames per second: " << frameCounter << endl;
        frameCounter = 0;
    }

    pubr.publish(msgr);
    publ.publish(msgl);

    ros::spinOnce();
    loop_rate.sleep();

    if(cv::waitKey(30) == 27){
      break;
    }
  }
}
