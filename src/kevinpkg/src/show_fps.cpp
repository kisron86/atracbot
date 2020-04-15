#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <time.h>
#include <chrono>

using namespace cv;
using namespace std;

int fps1(void);
int fps2(void);

int fps2(){
      cv::VideoCapture capr(0);  // kamera kanan
      cv::VideoCapture capl(2);  // kamera kiri

      capr.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
      capr.set(CV_CAP_PROP_FRAME_WIDTH, 320);
      capl.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
      capl.set(CV_CAP_PROP_FRAME_WIDTH, 320);

      if (!capr.open(0) || !capl.open(0))
          std::cout << "Problem connecting to cam " << std::endl;
      else
          std::cout << "Successfuly connected to camera " << std::endl;

      long frameCounter = 0;

      std::time_t timeBegin = std::time(0);
      int tick = 0;

      cv::Mat framer;
      cv::Mat framel;

      while (1)
      {
          capr.read(framer);
          capr.read(framel);

          cv::imshow("Kanan", framer);
          cv::imshow("Kiri", framer);
          cv::waitKey(1);

          frameCounter++;

          std::time_t timeNow = std::time(0) - timeBegin;

          if (timeNow - tick >= 1)
          {
              tick++;
              cout << "Frames per second: " << frameCounter << endl;
              frameCounter = 0;
          }
          if(cv::waitKey(30) == 27){
            break;
          }
      }

      return 0;
}

int fps1(){
  // Start default camera
  VideoCapture video(0);

  // With webcam get(CV_CAP_PROP_FPS) does not work.
  // Let's see for ourselves.

  double fps = video.get(CV_CAP_PROP_FPS);
  // If you do not care about backward compatibility
  // You can use the following instead for OpenCV 3
  // double fps = video.get(CAP_PROP_FPS);
  cout << "Frames per second using video.get(CV_CAP_PROP_FPS) : " << fps << endl;


  // Number of frames to capture
  int num_frames = 120;

  // Start and end times
  time_t start, end;

  // Variable for storing video frames
  Mat frame;

  cout << "Capturing " << num_frames << " frames" << endl ;

  // Start time
  time(&start);

  // Grab a few frames
  for(int i = 0; i < num_frames; i++)
  {
      video >> frame;
  }

  // End Time
  time(&end);

  // Time elapsed
  double seconds = difftime (end, start);
  cout << "Time taken : " << seconds << " seconds" << endl;

  // Calculate frames per second
  fps  = num_frames / seconds;
  cout << "Estimated frames per second : " << fps << endl;

  // Release video
  video.release();
  return 0;
}
int fps3(){
  cv::VideoCapture capr(0);  // kamera kanan
  cv::VideoCapture capl(2);  // kamera kiri

  capr.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
  capr.set(CV_CAP_PROP_FRAME_WIDTH, 320);
  capl.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
  capl.set(CV_CAP_PROP_FRAME_WIDTH, 320);

  if (!capr.open(0) || !capl.open(0))
      std::cout << "Problem connecting to cam " << std::endl;
  else
      std::cout << "Successfuly connected to camera " << std::endl;

  long frameCounter = 0;

  std::time_t timeBegin = std::time(0);
  int tick = 0;

  cv::Mat framer;
  cv::Mat framel;

  while (1)
  {
      capr.read(framer);
      capr.read(framel);

      cv::imshow("Kanan", framer);
      cv::imshow("Kiri", framer);
      cv::waitKey(1);

      frameCounter++;

      std::time_t timeNow = std::time(0) - timeBegin;

      if (timeNow - tick >= 1)
      {
          tick++;
          cout << "Frames per second: " << frameCounter << endl;
          frameCounter = 0;
      }
      if(cv::waitKey(30) == 27){
        break;
      }
  }

  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "show_fps");
  ros::NodeHandle nh;  

  // Start default camera
      VideoCapture video(2);

      video.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
      video.set(CV_CAP_PROP_FRAME_WIDTH, 320);

      // With webcam get(CV_CAP_PROP_FPS) does not work.
      // Let's see for ourselves.

      double fps = video.get(CV_CAP_PROP_FPS);
      // If you do not care about backward compatibility
      // You can use the following instead for OpenCV 3
      // double fps = video.get(CAP_PROP_FPS);
      cout << "Frames per second using video.get(CV_CAP_PROP_FPS) : " << fps << endl;


      // Number of frames to capture
      int num_frames = 120;

      // Start and end times
      time_t start, end;

      // Variable for storing video frames
      Mat frame;

      cout << "Capturing " << num_frames << " frames" << endl ;

      // Start time
      time(&start);

      // Grab a few frames
      for(int i = 0; i < num_frames; i++)
      {
          auto start = chrono::system_clock::now();
          video >> frame;
          auto end = chrono::system_clock::now();
          chrono::duration<double> diff = end-start;
          cout << "time per capture: " << diff.count() << endl;
      }

      // End Time
      time(&end);

      // Time elapsed
      double seconds = difftime (end, start);
      cout << "Time taken : " << seconds << " seconds" << endl;

      // Calculate frames per second
      fps  = num_frames / seconds;
      cout << "Estimated frames per second : " << fps << endl;

      // Release video
      video.release();
      return 0;
}
